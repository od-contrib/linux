/*
 * JZ4780 Audio Interface Controller
 *
 * Copyright (c) 2013 Imagination Technologies
 * Author: Paul Burton <paul.burton@imgtec.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __SOUND_SOC_JZ4780_AIC_H__
#define __SOUND_SOC_JZ4780_AIC_H__

#define AIC_AICFR	0x00
#define AIC_AICCR	0x04
#define AIC_ACCR1	0x08
#define AIC_ACCR2	0x0c
#define AIC_I2SCR	0x10
#define AIC_AICSR	0x14
#define AIC_ACSR	0x18
#define AIC_I2SSR	0x1c
#define AIC_ACCAR	0x20
#define AIC_ACCDR	0x24
#define AIC_ACSAR	0x28
#define AIC_ACSDR	0x2c
#define AIC_I2SDIV	0x30
#define AIC_AICDR	0x34

/* AIC Configuration Register (AICFR) */
#define AICFR_RFTH_SHIFT	24
#define AICFR_RFTH_MASK		(0xf << AICFR_RFTH_SHIFT)
#define AICFR_TFTH_SHIFT	16
#define AICFR_TFTH_MASK		(0x1f << AICFR_TFTH_SHIFT)
#define AICFR_IBCKD		(1 << 10)
#define AICFR_ISYNCD		(1 << 9)
#define AICFR_DMODE		(1 << 8)
#define AICFR_LSMP		(1 << 6)
#define AICFR_ICDC		(1 << 5)
#define AICFR_AUSEL		(1 << 4)
#define AICFR_RST		(1 << 3)
#define AICFR_BCKD		(1 << 2)
#define AICFR_SYNCD		(1 << 1)
#define AICFR_ENB		(1 << 0)

/* I2S/MSB-justified Control Register (I2SCR) */
#define I2SCR_RFIRST		(1 << 17)
#define I2SCR_SWLH		(1 << 16)
#define I2SCR_ISTPBK		(1 << 13)
#define I2SCR_STPBK		(1 << 12)
#define I2SCR_ESCLK		(1 << 4)
#define I2SCR_AMSL		(1 << 0)

/* (AICCR) */
#define AICCR_PACK16		(1 << 28)
#define AICCR_CHANNEL_SHIFT	24
#define AICCR_CHANNEL_MASK	(0x7 << AICCR_CHANNEL_SHIFT)
#define AICCR_OSS_SHIFT		19
#define AICCR_OSS_MASK		(0x7 << AICCR_OSS_SHIFT)
#define AICCR_ISS_SHIFT		16
#define AICCR_ISS_MASK		(0x7 << AICCR_ISS_SHIFT)
#define AICCR_RDMS		(1 << 15)
#define AICCR_TDMS		(1 << 14)
#define AICCR_M2S		(1 << 11)
#define AICCR_ENDSW		(1 << 10)
#define AICCR_ASVTSU		(1 << 9)
#define AICCR_TFLUSH		(1 << 8)
#define AICCR_RFLUSH		(1 << 7)
#define AICCR_EROR		(1 << 6)
#define AICCR_ETUR		(1 << 5)
#define AICCR_ERFS		(1 << 4)
#define AICCR_ETFS		(1 << 3)
#define AICCR_ENLBF		(1 << 2)
#define AICCR_ERPL		(1 << 1)
#define AICCR_EREC		(1 << 0)

/* ISS & OSS */
#define AICCR_SS_8		0x0
#define AICCR_SS_16		0x1
#define AICCR_SS_18		0x2
#define AICCR_SS_20		0x3
#define AICCR_SS_24		0x4

/* I2S/MSB-justified Clock Divisor Register */
#define I2SDIV_IDV_SHIFT	8
#define I2SDIV_IDV_MASK		(0xf << I2SDIV_IDV_SHIFT)
#define I2SDIV_DV_SHIFT		8
#define I2SDIV_DV_MASK		(0xf << I2SDIV_DV_SHIFT)

#endif /* __SOUND_SOC_JZ4780_AIC_H__ */
