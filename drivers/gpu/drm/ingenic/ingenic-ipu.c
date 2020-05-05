// SPDX-License-Identifier: GPL-2.0
//
// Ingenic JZ47xx IPU driver
//
// Copyright (C) 2020, Paul Cercueil <paul@crapouillou.net>

#include "ingenic-drm.h"
#include "ingenic-ipu.h"

#include <linux/clk.h>
#include <linux/component.h>
#include <linux/gcd.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/sysfs.h>
#include <linux/time.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_plane.h>
#include <drm/drm_plane_helper.h>
#include <drm/drm_vblank.h>

struct ingenic_ipu {
	struct drm_plane plane;
	struct device *dev, *master;
	struct regmap *map;
	struct clk *clk;

	unsigned int numW, numH, denomW, denomH;

	bool scaling_settings_changed;
};

/* Signed 15.16 fixed-point math (for bicubic scaling coefficients) */
#define I2F(i) ((int32_t)(i) * 65536)
#define F2I(f) ((f) / 65536)
#define FMUL(fa, fb) ((int32_t)(((int64_t)(fa) * (int64_t)(fb)) / 65536))

static bool keep_aspect_ratio = true;
static bool allow_downscaling = true;
static bool integer_upscaling = false;

/*
 * Sharpness settings range is [0,32]
 * 0       : nearest-neighbor
 * 1       : bilinear
 * 2 .. 32 : bicubic (translating to sharpness factor -0.25 .. -4.0 internally)
 */
#define SHARPNESS_INCR (I2F(-1) / 8)
static unsigned int sharpness_upscaling   = 8;      /* -0.125 * 8 = -1.0 */
static unsigned int sharpness_downscaling = 8;      /* -0.125 * 8 = -1.0 */

static inline struct ingenic_ipu *plane_to_ingenic_ipu(struct drm_plane *plane)
{
	return container_of(plane, struct ingenic_ipu, plane);
}

static inline int regmap_set_bits(struct regmap *map,
				  unsigned int reg, unsigned int mask)
{
	return regmap_update_bits(map, reg, mask, mask);
}

static inline int regmap_clear_bits(struct regmap *map,
				    unsigned int reg, unsigned int mask)
{
	return regmap_update_bits(map, reg, mask, 0);
}

/*
 * Apply conventional cubic convolution kernel. Both parameters
 *  and return value are 15.16 signed fixed-point.
 *
 *  @f_a: Sharpness factor, typically in range [-4.0, -0.25].
 *        A larger magnitude increases perceived sharpness, but going past
 *        -2.0 might cause ringing artifacts to outweigh any improvement.
 *        Nice values on a 320x240 LCD are between -0.75 and -2.0.
 *
 *  @f_x: Absolute distance in pixels from 'pixel 0' sample position
 *        along horizontal (or vertical) source axis. Range is [0, +2.0].
 *
 *  returns: Weight of this pixel within 4-pixel sample group. Range is
 *           [-2.0, +2.0]. For moderate (i.e. > -3.0) sharpness factors,
 *           range is within [-1.0, +1.0].
 */
static inline int32_t cubic_conv(int32_t f_a, int32_t f_x)
{
	const int32_t f_1 = I2F(1);
	const int32_t f_2 = I2F(2);
	const int32_t f_3 = I2F(3);
	const int32_t f_4 = I2F(4);
	const int32_t f_x2 = FMUL(f_x, f_x);
	const int32_t f_x3 = FMUL(f_x, f_x2);

	if (f_x <= f_1)
		return FMUL((f_a + f_2), f_x3) - FMUL((f_a + f_3), f_x2) + f_1;
	else if (f_x <= f_2)
		return FMUL(f_a, (f_x3 - 5 * f_x2 + 8 * f_x - f_4));
	else
		return 0;
}

/*
 * On entry, "weight" is a coefficient suitable for bilinear mode,
 *  which is converted to a set of four suitable for bicubic mode.
 *
 * "weight 512" means all of pixel 0;
 * "weight 256" means half of pixel 0 and half of pixel 1;
 * "weight 0" means all of pixel 1;
 *
 * "offset" is increment to next source pixel sample location.
 */
static void ingenic_ipu_set_coefs_reg(struct ingenic_ipu *ipu, unsigned int reg,
				      unsigned int sharpness_setting,
				      unsigned int weight, unsigned int offset)
{
	uint32_t val;
	int32_t w0, w1, w2, w3; /* Pixel weights at X (or Y) offsets -1,0,1,2 */

	weight = clamp_val(weight, 0, 512);

	if (sharpness_setting < 2) {
		/*
		 *  When sharpness setting is 0, emulate nearest-neighbor.
		 *  When sharpness setting is 1, emulate bilinear.
		 */

		if (sharpness_setting == 0)
			weight = weight >= 256 ? 512 : 0;
		w0 = 0;
		w1 = weight;
		w2 = 512 - weight;
		w3 = 0;
	} else {
		const int32_t f_a = SHARPNESS_INCR * sharpness_setting;
		const int32_t f_h = I2F(1) / 2; /* Round up 0.5 */

		/*
		 * Note that always rounding towards +infinity here is intended.
		 * The resulting coefficients match a round-to-nearest-int
		 * double floating-point implementation.
		 */

		weight = 512 - weight;
		w0 = F2I(f_h + 512 * cubic_conv(f_a, I2F(512  + weight) / 512));
		w1 = F2I(f_h + 512 * cubic_conv(f_a, I2F(0    + weight) / 512));
		w2 = F2I(f_h + 512 * cubic_conv(f_a, I2F(512  - weight) / 512));
		w3 = F2I(f_h + 512 * cubic_conv(f_a, I2F(1024 - weight) / 512));
		w0 = clamp_val(w0, -1024, 1023);
		w1 = clamp_val(w1, -1024, 1023);
		w2 = clamp_val(w2, -1024, 1023);
		w3 = clamp_val(w3, -1024, 1023);
	}

	val = ((w1 & 0x7FF) << 17) | ((w0 & 0x7FF) << 6);
	regmap_write(ipu->map, reg, val);
	val = ((w3 & 0x7FF) << 17) | ((w2 & 0x7FF) << 6) | (offset << 1);
	regmap_write(ipu->map, reg, val);
}

static void ingenic_ipu_set_downscale_coefs(struct ingenic_ipu *ipu,
					    unsigned int reg,
					    unsigned int num,
					    unsigned int denom)
{
	unsigned int i, offset, weight, weight_num = denom;

	for (i = 0; i < num; i++) {
		weight_num = num + (weight_num - num) % (num * 2);
		weight = 512 - 512 * (weight_num - num) / (num * 2);
		weight_num += denom * 2;
		offset = (weight_num - num) / (num * 2);

		ingenic_ipu_set_coefs_reg(ipu, reg, sharpness_downscaling,
					  weight, offset);
	}
}

static void ingenic_ipu_set_integer_upscale_coefs(struct ingenic_ipu *ipu,
						  unsigned int reg,
						  unsigned int num)
{
	/*
	 * Force nearest-neighbor scaling and use simple math when upscaling
	 * by an integer ratio. It looks better, and fixes a few problem cases.
	 */
	unsigned int i;

	for (i = 0; i < num; i++)
		ingenic_ipu_set_coefs_reg(ipu, reg, 0, 512, i == num - 1);
}

static void ingenic_ipu_set_upscale_coefs(struct ingenic_ipu *ipu,
					  unsigned int reg,
					  unsigned int num,
					  unsigned int denom)
{
	unsigned int i, offset, weight, weight_num = 0;

	for (i = 0; i < num; i++) {
		weight = 512 - 512 * weight_num / num;
		weight_num += denom;
		offset = weight_num >= num;

		if (offset)
			weight_num -= num;

		ingenic_ipu_set_coefs_reg(ipu, reg, sharpness_upscaling,
					  weight, offset);
	}
}

static void ingenic_ipu_set_coefs(struct ingenic_ipu *ipu, unsigned int reg,
				  unsigned int num, unsigned int denom)
{
	/* Begin programming the LUT */
	regmap_write(ipu->map, reg, 1);

	if (denom > num)
		ingenic_ipu_set_downscale_coefs(ipu, reg, num, denom);
	else if (denom == 1)
		ingenic_ipu_set_integer_upscale_coefs(ipu, reg, num);
	else
		ingenic_ipu_set_upscale_coefs(ipu, reg, num, denom);
}

static int reduce_fraction(unsigned int *num, unsigned int *denom)
{
	unsigned long d = gcd(*num, *denom);

	/* The scaling table has only 31 entries */
	if (*num > 31 * d)
		return -EINVAL;

	*num /= d;
	*denom /= d;
	return 0;
}

static inline bool scaling_required(struct drm_plane_state *state)
{
	return (state->src_w >> 16) != state->crtc_w &&
		(state->src_h >> 16) != state->crtc_h;
}

static inline bool osd_changed(struct drm_plane_state *state,
			       struct drm_plane_state *oldstate)
{
	return state->crtc_x != oldstate->crtc_x ||
		state->crtc_y != oldstate->crtc_y ||
		state->crtc_w != oldstate->crtc_w ||
		state->crtc_h != oldstate->crtc_h;
}

static void ingenic_ipu_plane_atomic_update(struct drm_plane *plane,
					    struct drm_plane_state *oldstate)
{
	struct ingenic_ipu *ipu = plane_to_ingenic_ipu(plane);
	struct drm_plane_state *state = plane->state;
	const struct drm_format_info *finfo;
	u32 ctrl, stride, coef_index = 0, format = 0;
	bool needs_modeset;
	dma_addr_t addr;

	if (!state || !state->fb)
		return;

	ingenic_drm_plane_enable(ipu->master, plane);

	finfo = drm_format_info(state->fb->format->format);

	switch (finfo->format) {
	case DRM_FORMAT_XRGB1555:
		format = JZ_IPU_D_FMT_IN_FMT_RGB555 |
			JZ_IPU_D_FMT_RGB_OUT_OFT_RGB;
		break;
	case DRM_FORMAT_XBGR1555:
		format = JZ_IPU_D_FMT_IN_FMT_RGB555 |
			JZ_IPU_D_FMT_RGB_OUT_OFT_BGR;
		break;
	case DRM_FORMAT_RGB565:
		format = JZ_IPU_D_FMT_IN_FMT_RGB565 |
			JZ_IPU_D_FMT_RGB_OUT_OFT_RGB;
		break;
	case DRM_FORMAT_BGR565:
		format = JZ_IPU_D_FMT_IN_FMT_RGB565 |
			JZ_IPU_D_FMT_RGB_OUT_OFT_BGR;
		break;
	case DRM_FORMAT_XRGB8888:
		format = JZ_IPU_D_FMT_IN_FMT_RGB888 |
			JZ_IPU_D_FMT_RGB_OUT_OFT_RGB;
		break;
	case DRM_FORMAT_XBGR8888:
		format = JZ_IPU_D_FMT_IN_FMT_RGB888 |
			JZ_IPU_D_FMT_RGB_OUT_OFT_BGR;
		break;
	case DRM_FORMAT_YUYV:
		format = JZ_IPU_D_FMT_IN_FMT_YUV422 |
			JZ_IPU_D_FMT_YUV_VY1UY0;
		break;
	case DRM_FORMAT_YVYU:
		format = JZ_IPU_D_FMT_IN_FMT_YUV422 |
			JZ_IPU_D_FMT_YUV_UY1VY0;
		break;
	case DRM_FORMAT_UYVY:
		format = JZ_IPU_D_FMT_IN_FMT_YUV422 |
			JZ_IPU_D_FMT_YUV_Y1VY0U;
		break;
	case DRM_FORMAT_VYUY:
		format = JZ_IPU_D_FMT_IN_FMT_YUV422 |
			JZ_IPU_D_FMT_YUV_Y1UY0V;
		break;
	case DRM_FORMAT_YUV411:
		format = JZ_IPU_D_FMT_IN_FMT_YUV411;
		break;
	case DRM_FORMAT_YUV420:
		format = JZ_IPU_D_FMT_IN_FMT_YUV420;
		break;
	case DRM_FORMAT_YUV422:
		format = JZ_IPU_D_FMT_IN_FMT_YUV422;
		break;
	case DRM_FORMAT_YUV444:
		format = JZ_IPU_D_FMT_IN_FMT_YUV444;
		break;
	default:
		WARN_ONCE(1, "Unsupported format");
		break;
	}

	/* Fix output to RGB888 */
	format |= JZ_IPU_D_FMT_OUT_FMT_RGB888;

	/* Reset all the registers if needed */
	needs_modeset = drm_atomic_crtc_needs_modeset(state->crtc->state);
	if (needs_modeset)
		regmap_set_bits(ipu->map, JZ_REG_IPU_CTRL, JZ_IPU_CTRL_RST);

	/* Enable the chip */
	regmap_set_bits(ipu->map, JZ_REG_IPU_CTRL, JZ_IPU_CTRL_CHIP_EN);

	/* Set pixel format */
	regmap_write(ipu->map, JZ_REG_IPU_D_FMT, format);

	/* Set address and stride */
	addr = drm_fb_cma_get_gem_addr(state->fb, state, 0);
	stride = ((state->src_w >> 16) * finfo->cpp[0])
		<< JZ_IPU_Y_STRIDE_Y_LSB;

	regmap_write(ipu->map, JZ_REG_IPU_Y_ADDR, addr);
	regmap_write(ipu->map, JZ_REG_IPU_Y_STRIDE, stride);
	regmap_write(ipu->map, JZ_REG_IPU_IN_GS,
		     (stride << JZ_IPU_IN_GS_W_LSB) |
		     ((state->src_h >> 16) << JZ_IPU_IN_GS_H_LSB));

	stride = 0;

	if (finfo->num_planes > 2) {
		addr = drm_fb_cma_get_gem_addr(state->fb, state, 2);
		stride = ((state->src_w >> 16) * finfo->cpp[2] / finfo->hsub)
			<< JZ_IPU_UV_STRIDE_V_LSB;

		regmap_write(ipu->map, JZ_REG_IPU_V_ADDR, addr);
	}

	if (finfo->num_planes > 1) {
		addr = drm_fb_cma_get_gem_addr(state->fb, state, 1);
		stride |= ((state->src_w >> 16) * finfo->cpp[1] / finfo->hsub)
			<< JZ_IPU_UV_STRIDE_U_LSB;

		regmap_write(ipu->map, JZ_REG_IPU_U_ADDR, addr);
		regmap_write(ipu->map, JZ_REG_IPU_UV_STRIDE, stride);
	}

	if (!needs_modeset)
		return;

	ctrl = JZ_IPU_CTRL_LCDC_SEL | JZ_IPU_CTRL_FM_IRQ_EN;

	if (finfo->num_planes == 1)
		ctrl |= JZ_IPU_CTRL_SPKG_SEL;
	if (finfo->is_yuv)
		ctrl |= JZ_IPU_CTRL_CSC_EN;

	regmap_update_bits(ipu->map, JZ_REG_IPU_CTRL,
			   JZ_IPU_CTRL_LCDC_SEL | JZ_IPU_CTRL_FM_IRQ_EN |
			   JZ_IPU_CTRL_SPKG_SEL | JZ_IPU_CTRL_CSC_EN, ctrl);

	if (finfo->is_yuv) {
		/*
		 * Offsets for Chroma/Luma.
		 * y = source Y - LUMA,
		 * u = source Cb - CHROMA,
		 * v = source Cr - CHROMA
		 */
		regmap_write(ipu->map, JZ_REG_IPU_CSC_OFFSET,
			     128 << JZ_IPU_CSC_OFFSET_CHROMA_LSB |
			     0 << JZ_IPU_CSC_OFFSET_LUMA_LSB);

		/*
		 * YUV422 to RGB conversion table.
		 * R = C0 / 0x400 * y + C1 / 0x400 * v
		 * G = C0 / 0x400 * y - C2 / 0x400 * u - C3 / 0x400 * v
		 * B = C0 / 0x400 * y + C4 / 0x400 * u
		 */
		regmap_write(ipu->map, JZ_REG_IPU_CSC_C0_COEF, 0x4a8);
		regmap_write(ipu->map, JZ_REG_IPU_CSC_C1_COEF, 0x662);
		regmap_write(ipu->map, JZ_REG_IPU_CSC_C2_COEF, 0x191);
		regmap_write(ipu->map, JZ_REG_IPU_CSC_C3_COEF, 0x341);
		regmap_write(ipu->map, JZ_REG_IPU_CSC_C4_COEF, 0x811);
	}

	if (scaling_required(state)) {
		/*
		 * Must set ZOOM_SEL before programming bicubic LUTs.
		 * The IPU supports both bilinear and bicubic modes, but we use
		 * only bicubic. It can do anything bilinear can and more.
		 */
		regmap_set_bits(ipu->map, JZ_REG_IPU_CTRL,
				JZ_IPU_CTRL_ZOOM_SEL);

		if (ipu->numW != 1 || ipu->denomW != 1) {
			ingenic_ipu_set_coefs(ipu, JZ_REG_IPU_HRSZ_COEF_LUT,
					      ipu->numW, ipu->denomW);
			coef_index |= (ipu->numW - 1) << 16;

			regmap_set_bits(ipu->map, JZ_REG_IPU_CTRL,
					JZ_IPU_CTRL_HRSZ_EN);
		}

		if (ipu->numH != 1 || ipu->denomH != 1) {
			ingenic_ipu_set_coefs(ipu, JZ_REG_IPU_VRSZ_COEF_LUT,
					      ipu->numH, ipu->denomH);
			coef_index |= ipu->numH - 1;

			regmap_set_bits(ipu->map, JZ_REG_IPU_CTRL,
					JZ_IPU_CTRL_VRSZ_EN);
		}
	}

	/* Set the LUT index register */
	regmap_write(ipu->map, JZ_REG_IPU_RSZ_COEF_INDEX, coef_index);

	/* Set the output height/width/stride */
	regmap_write(ipu->map, JZ_REG_IPU_OUT_GS,
		     ((state->crtc_w * 4) << JZ_IPU_OUT_GS_W_LSB)
		     | state->crtc_h << JZ_IPU_OUT_GS_H_LSB);
	regmap_write(ipu->map, JZ_REG_IPU_OUT_STRIDE, state->crtc_w * 4);

	ingenic_drm_plane_config(ipu->master, plane);

	/* Clear STATUS register */
	regmap_write(ipu->map, JZ_REG_IPU_STATUS, 0);

	/* Start IPU */
	regmap_set_bits(ipu->map, JZ_REG_IPU_CTRL, JZ_IPU_CTRL_RUN);

	dev_dbg(ipu->dev, "Scaling %ux%u to %ux%u (%u:%u horiz, %u:%u vert)\n",
		state->src_w >> 16, state->src_h >> 16,
		state->crtc_w, state->crtc_h,
		ipu->numW, ipu->denomW, ipu->numH, ipu->denomH);
}

static int ingenic_ipu_plane_atomic_check(struct drm_plane *plane,
					  struct drm_plane_state *state)
{
	unsigned int numW, denomW, numH, denomH, ratioW, ratioH, xres, yres;
	struct ingenic_ipu *ipu = plane_to_ingenic_ipu(plane);
	struct drm_crtc_state *crtc_state;

	if (!state || !state->crtc)
		return 0;

	crtc_state = drm_atomic_get_existing_crtc_state(state->state,
							state->crtc);
	if (WARN_ON(!crtc_state))
		return -EINVAL;

	if (state->crtc_x < 0 || state->crtc_y < 0 ||
	    state->crtc_x + state->crtc_w > crtc_state->mode.hdisplay ||
	    state->crtc_y + state->crtc_h > crtc_state->mode.vdisplay)
		return -EINVAL;

	/* Input lines must have an even number of pixels. */
	if ((state->src_w >> 16) & 1)
		return -EINVAL;

	/* Output lines must have an even number of pixels. */
	state->crtc_w &= ~1;

	if (ipu->scaling_settings_changed) {
		ipu->scaling_settings_changed = false;
		crtc_state->mode_changed = true;
	}

	if (!scaling_required(state)) {
		if (plane->state && osd_changed(state, plane->state))
			crtc_state->mode_changed = true;

		ipu->numW = ipu->numH = ipu->denomW = ipu->denomH = 1;
		return 0;
	}

	crtc_state->mode_changed = true;

	xres = state->src_w >> 16;
	yres = state->src_h >> 16;

	if (xres < 4)
		xres = 4;
	if (yres < 4)
		yres = 4;

	if (!allow_downscaling) {
		if (xres > crtc_state->mode.hdisplay)
			xres = crtc_state->mode.hdisplay;
		if (yres > crtc_state->mode.vdisplay)
			yres = crtc_state->mode.vdisplay;
	}

	numW = state->crtc_w;
	numH = state->crtc_h;
	denomW = xres;
	denomH = yres;

	if (integer_upscaling && denomW <= numW && denomH <= numH) {
		numW /= denomW;
		numH /= denomH;
		denomW = denomH = 1;
	}

	if (keep_aspect_ratio) {
		ratioW = (UINT_MAX >> 6) * numW / denomW;
		ratioH = (UINT_MAX >> 6) * numH / denomH;

		if (ratioW < ratioH) {
			numH = numW;
			denomH = denomW;
		} else {
			numW = numH;
			denomW = denomH;
		}
	}

	/* Adjust the output size until we find a valid configuration */
	for (; numW <= crtc_state->mode.hdisplay &&
	     reduce_fraction(&numW, &denomW) < 0; numW++);
	if (numW > crtc_state->mode.hdisplay)
		return -EINVAL;

	for (; numH <= crtc_state->mode.vdisplay &&
	     reduce_fraction(&numH, &denomH) < 0; numH++);
	if (numH > crtc_state->mode.vdisplay)
		return -EINVAL;

	ipu->numW = numW;
	ipu->numH = numH;
	ipu->denomW = denomW;
	ipu->denomH = denomH;

	return 0;
}

static void ingenic_ipu_plane_atomic_disable(struct drm_plane *plane,
					     struct drm_plane_state *old_state)
{
	struct ingenic_ipu *ipu = plane_to_ingenic_ipu(plane);

	regmap_clear_bits(ipu->map, JZ_REG_IPU_CTRL, JZ_IPU_CTRL_CHIP_EN);

	ingenic_drm_plane_disable(ipu->master, plane);
}

static const struct drm_plane_helper_funcs ingenic_ipu_plane_helper_funcs = {
	.atomic_update		= ingenic_ipu_plane_atomic_update,
	.atomic_check		= ingenic_ipu_plane_atomic_check,
	.atomic_disable		= ingenic_ipu_plane_atomic_disable,
	.prepare_fb		= drm_gem_fb_prepare_fb,
};

static const struct drm_plane_funcs ingenic_ipu_plane_funcs = {
	.update_plane		= drm_atomic_helper_update_plane,
	.disable_plane		= drm_atomic_helper_disable_plane,
	.reset			= drm_atomic_helper_plane_reset,
	.destroy		= drm_plane_cleanup,

	.atomic_duplicate_state	= drm_atomic_helper_plane_duplicate_state,
	.atomic_destroy_state	= drm_atomic_helper_plane_destroy_state,
};

static irqreturn_t ingenic_ipu_irq_handler(int irq, void *arg)
{
	struct drm_plane *plane = drm_plane_from_index(arg, 0);
	struct ingenic_ipu *ipu = plane_to_ingenic_ipu(plane);
	struct drm_crtc *crtc = drm_crtc_from_index(arg, 0);

	regmap_write(ipu->map, JZ_REG_IPU_STATUS, 0);

	drm_crtc_handle_vblank(crtc);

	return IRQ_HANDLED;
}

static void scaling_settings_change(struct device *dev)
{
	struct drm_plane *plane = dev_get_drvdata(dev);
	struct ingenic_ipu *ipu = plane_to_ingenic_ipu(plane);

	ipu->scaling_settings_changed = true;
}

static ssize_t keep_aspect_ratio_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", (int)keep_aspect_ratio);
}

static ssize_t keep_aspect_ratio_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	bool new_val;

	if (kstrtobool(buf, &new_val))
		return -EINVAL;

	if (keep_aspect_ratio != new_val) {
		keep_aspect_ratio = new_val;
		scaling_settings_change(dev);
	}

	return count;
}

static ssize_t integer_upscaling_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n", (int)integer_upscaling);
}

static ssize_t integer_upscaling_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	bool new_val;

	if (kstrtobool(buf, &new_val))
		return -EINVAL;

	if (integer_upscaling != new_val) {
		integer_upscaling = new_val;
		scaling_settings_change(dev);
	}

	return count;
}

static ssize_t sharpness_upscaling_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", sharpness_upscaling);
}

static ssize_t sharpness_upscaling_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	unsigned int new_val;

	if (kstrtouint(buf, 0, &new_val) || new_val > 32)
		return -EINVAL;

	if (sharpness_upscaling != new_val) {
		sharpness_upscaling = new_val;
		scaling_settings_change(dev);
	}

	return count;
}

static ssize_t sharpness_downscaling_show(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", sharpness_downscaling);
}

static ssize_t sharpness_downscaling_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	unsigned int new_val;

	if (kstrtouint(buf, 0, &new_val) || new_val > 32)
		return -EINVAL;

	if (sharpness_downscaling != new_val) {
		sharpness_downscaling = new_val;
		scaling_settings_change(dev);
	}

	return count;
}

static DEVICE_ATTR_RW(keep_aspect_ratio);
static DEVICE_ATTR_RW(integer_upscaling);
static DEVICE_ATTR_RW(sharpness_upscaling);
static DEVICE_ATTR_RW(sharpness_downscaling);
static DEVICE_BOOL_ATTR(allow_downscaling, 0644, allow_downscaling);

static struct attribute *ingenic_ipu_attrs[] = {
	&dev_attr_keep_aspect_ratio.attr,
	&dev_attr_integer_upscaling.attr,
	&dev_attr_sharpness_upscaling.attr,
	&dev_attr_sharpness_downscaling.attr,
	&dev_attr_allow_downscaling.attr.attr,
	NULL,
};

static const struct attribute_group ingenic_ipu_group = {
	.attrs = ingenic_ipu_attrs,
};

static const u32 ingenic_ipu_formats[] = {
	DRM_FORMAT_XRGB1555,
	DRM_FORMAT_XBGR1555,
	DRM_FORMAT_RGB565,
	DRM_FORMAT_BGR565,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_YUYV,
	DRM_FORMAT_YVYU,
	DRM_FORMAT_UYVY,
	DRM_FORMAT_VYUY,
	DRM_FORMAT_YUV411,
	DRM_FORMAT_YUV420,
	DRM_FORMAT_YUV422,
	DRM_FORMAT_YUV444,
};

static const struct regmap_config ingenic_ipu_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,

	.max_register = JZ_REG_IPU_OUT_PHY_T_ADDR,
};

static int ingenic_ipu_bind(struct device *dev, struct device *master, void *d)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct drm_device *drm = d;
	struct drm_plane *plane;
	struct ingenic_ipu *ipu;
	void __iomem *base;
	int err, irq;

	ipu = devm_kzalloc(dev, sizeof(*ipu), GFP_KERNEL);
	if (!ipu)
		return -ENOMEM;

	ipu->dev = dev;
	ipu->master = master;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base)) {
		dev_err(dev, "Failed to get memory resource\n");
		return PTR_ERR(base);
	}

	ipu->map = devm_regmap_init_mmio(dev, base, &ingenic_ipu_regmap_config);
	if (IS_ERR(ipu->map)) {
		dev_err(dev, "Failed to create regmap\n");
		return PTR_ERR(ipu->map);
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ipu->clk = devm_clk_get(dev, "ipu");
	if (IS_ERR(ipu->clk)) {
		dev_err(dev, "Failed to get pixel clock\n");
		return PTR_ERR(ipu->clk);
	}

	err = devm_request_irq(dev, irq, ingenic_ipu_irq_handler, 0,
			       drm->driver->name, drm);
	if (err) {
		dev_err(dev, "Unable to request IRQ\n");
		return err;
	}

	plane = &ipu->plane;
	dev_set_drvdata(dev, plane);

	err = clk_prepare_enable(ipu->clk);
	if (err) {
		dev_err(dev, "Unable to enable clock\n");
		return err;
	}

	drm_plane_helper_add(plane, &ingenic_ipu_plane_helper_funcs);

	err = drm_universal_plane_init(drm, plane, 1, &ingenic_ipu_plane_funcs,
				       ingenic_ipu_formats,
				       ARRAY_SIZE(ingenic_ipu_formats),
				       NULL, DRM_PLANE_TYPE_PRIMARY, NULL);
	if (err) {
		dev_err(dev, "Failed to init plane: %i\n", err);
		clk_disable_unprepare(ipu->clk);
		return err;
	}

	err = sysfs_create_group(&dev->kobj, &ingenic_ipu_group);
	if (err < 0)
		dev_warn(dev, "Unable to create sysfs files: %d\n", err);

	return 0;
}

static void ingenic_ipu_unbind(struct device *dev,
			       struct device *master, void *d)
{
	struct ingenic_ipu *ipu = dev_get_drvdata(dev);

	sysfs_remove_group(&dev->kobj, &ingenic_ipu_group);
	clk_disable_unprepare(ipu->clk);
}

static const struct component_ops ingenic_ipu_ops = {
	.bind = ingenic_ipu_bind,
	.unbind = ingenic_ipu_unbind,
};

static int ingenic_ipu_probe(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &ingenic_ipu_ops);
}

static int ingenic_ipu_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &ingenic_ipu_ops);
	return 0;
}

static const struct of_device_id ingenic_ipu_of_match[] = {
	{ .compatible = "ingenic,jz4770-ipu" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, ingenic_ipu_of_match);

static struct platform_driver ingenic_ipu_driver = {
	.driver = {
		.name = "ingenic-ipu",
		.of_match_table = ingenic_ipu_of_match,
	},
	.probe = ingenic_ipu_probe,
	.remove = ingenic_ipu_remove,
};
module_platform_driver(ingenic_ipu_driver);

MODULE_AUTHOR("Paul Cercueil <paul@crapouillou.net>");
MODULE_DESCRIPTION("IPU driver for the Ingenic SoCs\n");
MODULE_LICENSE("GPL v2");
