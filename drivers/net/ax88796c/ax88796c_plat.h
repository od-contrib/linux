#ifndef _AX796C_PLAT_H_
#define _AX796C_PLAT_H_

#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/types.h>
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/ioport.h>
//#include <linux/interrupt.h>
#include <linux/dmaengine.h>

//#include <asm/system.h>
//#include <asm/io.h>
//#include <asm/uaccess.h>
//#include <asm/irq.h>
#include <asm/dma.h>

#ifndef TRUE
#define TRUE				1
#endif

#ifndef FALSE
#define FALSE				0
#endif

/* 
 * Configuration options
 */
/* DMA mode only effected on SMDK2440 platform. */
#define TX_DMA_MODE			TRUE
#define RX_DMA_MODE			TRUE
#define AX88796B_PIN_COMPATIBLE		FALSE
#define AX88796C_8BIT_MODE		TRUE
#define DMA_BURST_LEN			DMA_BURST_LEN_8_WORD
#define REG_SHIFT			0x00

#if (AX88796B_PIN_COMPATIBLE)
#define DATA_PORT_ADDR			0x800
#else
#define DATA_PORT_ADDR			0x0020
#endif

/* Exported DMA operations */
int ax88796c_plat_dma_init (unsigned long base_addr,
			    void (*tx_dma_complete)(void *data),
			    void (*rx_dma_complete)(void *data),
			    void *priv);
void ax88796c_plat_dma_release (void);
void dma_start(dma_addr_t dst, int len, u8 tx);

/* Plat Initialise */
void ax88796c_plat_init (void __iomem *confbase, int bus_width);
#endif	/* _AX796C_PLAT_H_ */
