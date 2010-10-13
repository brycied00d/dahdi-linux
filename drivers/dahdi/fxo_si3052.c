/*
 A driver for single-port FXO cards based on Si3052 chip + Si3011/17/18 DAA (Motorola 52-6116-2A, PM560MS etc.)

 Author: Maciej Szmigiero <mhej@o2.pl>

 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
*/

#include <asm/io.h>
#include <linux/time.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/init.h>

#include "fxo_si3052.h"

#include "fxo_modes.h"

static char *region_param;
module_param(region_param, charp, S_IRUGO);
MODULE_PARM_DESC(region_param, "operating region name from fxo_modes.h. required.");
static unsigned int ireg;

static int use_ulaw = 0;
module_param(use_ulaw, bool, S_IRUGO);
MODULE_PARM_DESC(use_ulaw, "use ulaw instead of alaw. default no.");

static unsigned short min_ringer_time = 512;
module_param(min_ringer_time, ushort, S_IRUGO);
MODULE_PARM_DESC(min_ringer_time, "minimum continuous ringing time for ringer validation. between 100 and 1024ms with default of 512ms.");

static int buff_warn = 0;
module_param(buff_warn, bool, S_IRUGO|S_IWUSR);
MODULE_PARM_DESC(buff_warn, "warn about RX/TX buffers being not ready on time. default no.");

static unsigned int num_devices = 0;

/* PCI side register access */
static u32 siregr(si3052_dev *dev, u8 regno) {
    return ioread32(dev->mmio_ptr + regno);
}

static void siregw(si3052_dev *dev, u8 regno, u32 value) {
    iowrite32(value, dev->mmio_ptr + regno);
}

static void siregs(si3052_dev *dev, u8 regno, u32 mask, u32 value) {
    u32 regval;

    regval = siregr(dev, regno);
    regval &= ~mask;
    regval |= (value & mask);
    siregw(dev, regno, regval);
}

/* DAA side register access */
static u8 sidregr(si3052_dev *dev, u8 regno) {
    return ioread8(dev->mmio_ptr + regno);
}

static void sidregw(si3052_dev *dev, u8 regno, u8 value) {
    iowrite8(value, dev->mmio_ptr + regno);
}

static void sidregs(si3052_dev *dev, u8 regno, u8 mask, u8 value) {
    u8 regval;

    regval = sidregr(dev, regno);
    regval &= ~mask;
    regval |= (value & mask);
    sidregw(dev, regno, regval);
}

static int si3052_fill_txbuf(si3052_dev *dev, unsigned int startat, unsigned int howmany) {
    int ret;

    SBENTER;
    
    startat *= (4*DAHDI_CHUNKSIZE); /* now startat points to the beginning */

    while (howmany--) {
        unsigned int n;
        BUG_ON(startat >= S3052_DMABUF_SIZE);

	/* get next chunk */
	ret = dahdi_transmit(&(dev->span));
	if ((ret != 0) && (dev->enable_tx)) {
	    if (printk_ratelimit())
		dev_err(&(dev->pdev->dev), "dahdi_transmit() failed with error %d\n", ret);

	    SBLEAVE;

	    return ret;
	}
	
	for (n = 0; n < (DAHDI_CHUNKSIZE*4); n+=4) {
	    u16 sample;
	    
	    if (dev->enable_tx)
		sample = DAHDI_XLAW(dev->chan.writechunk[n/4], (&(dev->chan)));
	    else 
		sample = 0;

	    /* sample is in lower 16-bits (LE) */
	    dev->txbuf[startat + n + 1] = (sample >> 8) & 0xff;
	    dev->txbuf[startat + n + 0] = sample & 0xff;
	    /* first 16-bits are 0 */
	    dev->txbuf[startat + n + 3] = 0;
	    dev->txbuf[startat + n + 2] = 0;
	}

	/* save original samples for EC */
	if (dev->enable_tx)
	    memcpy(&(dev->txori[startat/4]), dev->chan.writechunk, DAHDI_CHUNKSIZE);
	else 
	    memset(&(dev->txori[startat/4]), DAHDI_LIN2X(0, &(dev->chan)), DAHDI_CHUNKSIZE);

	startat += (4*DAHDI_CHUNKSIZE);
    }

    SBLEAVE;
    
    return 0;
}

static int si3052_rx_buf(si3052_dev *dev, unsigned int startat, unsigned int howmany) {
    SBENTER;

    startat *= (4*DAHDI_CHUNKSIZE); /* now startat points to the beginning */
    while (howmany--) {
        unsigned int n;
        BUG_ON(startat >= S3052_DMABUF_SIZE);

	for (n = 0; n < DAHDI_CHUNKSIZE*4; n+=4) {
	    /* sample is in lower 16-bits (LE) */
	    s16 sample = (dev->rxbuf[startat + n + 1] << 8) | dev->rxbuf[startat + n + 0];
	
	    dev->chan.readchunk[n/4] = DAHDI_LIN2X((sample), &(dev->chan));
	}

	dahdi_ec_chunk(&(dev->chan), dev->chan.readchunk, dev->txori);

	dahdi_receive(&(dev->span));

	startat += (4*DAHDI_CHUNKSIZE);
    }

    SBLEAVE;

    return 0;
}

static int si3052_dma_program(si3052_dev *dev) {
    unsigned int n;

    SENTER;

    /* disable DMA */
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_DMAE, 0);

    /* reset DMA engine */
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_DRST, S3052_DMAI_DRST);
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_DRST, 0);

    /* disable DMA and PCI error interrupts */
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_RBIE|S3052_DMAI_RAIE|S3052_DMAI_WAIE|S3052_DMAI_WBIE|S3052_DMAI_MAIE|S3052_DMAI_TAIE|S3052_DMAI_MTIE, 0);

#if S3052_DMABUF_SIZE < 16*4
#error too small S3052_RXTXBUF_SIZE (must hold at least 16 samples)
#endif

    /* prefill TX FIFO with silence... */
    /* FIFOs are 8 samples deep but it looks like the device won't start RXing until TX FIFO is full */
    memset(dev->txbuf, 0, 16*4);
#ifndef S3052_AUTORESTART_DMA
    /* ... enable DMA autorestart for a moment... */
    /* sometimes it takes even more samples to fill TX FIFO and DMA autorestart will do it for us */
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_DMAM, 0);
#endif
    /* ... program start and stop address... */
    siregw(dev, S3052_REG_DMAWSTART, dev->txbuf_h);
    siregw(dev, S3052_REG_DMAWSTOP, dev->txbuf_h + 16*4 - 4);
    siregw(dev, S3052_REG_DMARSTART, dev->rxbuf_h);
    siregw(dev, S3052_REG_DMARSTOP, dev->rxbuf_h + 16*4 - 4);
    /* ... rewind DMA address... */
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_DMAR, S3052_DMAI_DMAR);
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_DMAR, 0);
    /* ... clear DMA read and write end of buffer status... */
    siregs(dev, S3052_REG_DMAIS, S3052_DMAIS_ALL, S3052_DMAIS_DWB|S3052_DMAIS_DRB);
    /* ... enable DMA ... */
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_DMAE, S3052_DMAI_DMAE);
    
    /* ... wait for FIFOs to be ready... */
    for (n = 0; n < 100; n++) {
	if ((siregr(dev, S3052_REG_DMAIS) & (S3052_DMAIS_WFF|S3052_DMAIS_RFE)) == (S3052_DMAIS_WFF|S3052_DMAIS_RFE))
	    break;
	mdelay(1);
    }

    if (n>=100)
	dev_err(&(dev->pdev->dev), "FIFO preparation timed out\n");

    /* ... disable DMA ... */
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_DMAE, 0);
#ifndef S3052_AUTORESTART_DMA
    /* ... disable DMA autorestart */
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_DMAM, S3052_DMAI_DMAM);
#endif

    /* reset streams */
    si3052_rx_buf(dev, 0, S3052_RXTXBUF_SIZE);
    si3052_fill_txbuf(dev, 0, S3052_RXTXBUF_SIZE);

    dev->txbuf_state |= (S3052_BUF_SECOND | S3052_BUF_FIRST);
    dev->rxbuf_state |= (S3052_BUF_SECOND | S3052_BUF_FIRST);

    /* program tx buffer begin and end address */
    siregw(dev, S3052_REG_DMAWSTART, dev->txbuf_h);
    siregw(dev, S3052_REG_DMAWSTOP, dev->txbuf_h + S3052_DMABUF_SIZE - 4);
    /* program refill interrupt address */
    siregw(dev, S3052_REG_DMAWINTERR, dev->txbuf_h + S3052_DMABUF_SPLIT - 4);

    /* program rx buffer begin and end address */
    siregw(dev, S3052_REG_DMARSTART, dev->rxbuf_h);
    siregw(dev, S3052_REG_DMARSTOP, dev->rxbuf_h + S3052_DMABUF_SIZE - 4);
    /* program flush interrupt address */
    siregw(dev, S3052_REG_DMARINTERR, dev->rxbuf_h + S3052_DMABUF_SPLIT - 4);

#ifndef S3052_AUTORESTART_DMA
    /* reset current DMA address */
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_DMAR, S3052_DMAI_DMAR);
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_DMAR, 0);
#endif

    /* clear DMA and PCI interrupt flags */
    siregs(dev, S3052_REG_DMAIS, S3052_DMAIS_ALL,
				 S3052_DMAIS_MTO|S3052_DMAIS_PTA|S3052_DMAIS_PMA|S3052_DMAIS_DRB|S3052_DMAIS_DRA|S3052_DMAIS_DWA|S3052_DMAIS_DWB);

    /* enable r/w end of buffer, flush/refill and PCI error interrupts */
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_RBIE|S3052_DMAI_RAIE|S3052_DMAI_WAIE|S3052_DMAI_WBIE|S3052_DMAI_MAIE|S3052_DMAI_TAIE|S3052_DMAI_MTIE,
				S3052_DMAI_RBIE|S3052_DMAI_RAIE|S3052_DMAI_WAIE|S3052_DMAI_WBIE|S3052_DMAI_MAIE|S3052_DMAI_TAIE|S3052_DMAI_MTIE);

    /* enable DMA */
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_DMAE, S3052_DMAI_DMAE);

    SLEAVE;

    return 0;
}

static void si3052_dma_restart(unsigned long arg) {
    unsigned long flags;
    si3052_dev *dev = (si3052_dev *)arg;

    SENTER;

    spin_lock_irqsave(&(dev->lock), flags);
    
    si3052_dma_program(dev);

    spin_unlock_irqrestore(&(dev->lock), flags);

    SLEAVE;
}

static void si3052_dma_buffers(unsigned long arg) {
#ifdef S3052_BUFFER_TASKLETS
    unsigned long flags;
#endif
    si3052_dev *dev = (si3052_dev *)arg;
    int which = 0;
    
    SBENTER;

#ifdef S3052_BUFFER_TASKLETS
    spin_lock_irqsave(&(dev->lock), flags);
#endif

    /* check which parts of buffers need processing */
    if ((!(dev->txbuf_state & S3052_BUF_FIRST)) && (!(dev->rxbuf_state & S3052_BUF_FIRST))) { /* first */
	si3052_rx_buf(dev, 0, S3052_DMABUF_SPLIT / DAHDI_CHUNKSIZE / 4);
	si3052_fill_txbuf(dev, 0, S3052_DMABUF_SPLIT / DAHDI_CHUNKSIZE / 4);
	dev->txbuf_state |= S3052_BUF_FIRST;
	dev->rxbuf_state |= S3052_BUF_FIRST;
	which++;
    }
    
    if ((!(dev->txbuf_state & S3052_BUF_SECOND)) && (!(dev->rxbuf_state & S3052_BUF_SECOND))) { /* second */
	si3052_rx_buf(dev, S3052_DMABUF_SPLIT / DAHDI_CHUNKSIZE / 4, S3052_DMABUF_SPLIT / DAHDI_CHUNKSIZE / 4);
	si3052_fill_txbuf(dev, S3052_DMABUF_SPLIT / DAHDI_CHUNKSIZE / 4, S3052_DMABUF_SPLIT / DAHDI_CHUNKSIZE / 4);
	dev->txbuf_state |= S3052_BUF_SECOND;
	dev->rxbuf_state |= S3052_BUF_SECOND;
	which++;
    }
    
    if (which == 2) { /* both buffers processed */
#ifndef S3052_AUTORESTART_DMA
	/* reset current DMA addresses to the beginning */
	siregs(dev, S3052_REG_DMAI, S3052_DMAI_DMAR, S3052_DMAI_DMAR);
	siregs(dev, S3052_REG_DMAI, S3052_DMAI_DMAR, 0);
#endif

	/* enable DMA */
	siregs(dev, S3052_REG_DMAI, S3052_DMAI_DMAE, S3052_DMAI_DMAE);
	
	if (buff_warn)
	    if (printk_ratelimit())
		dev_warn(&(dev->pdev->dev), "buffer not ready on time\n");
    }


#ifdef S3052_BUFFER_TASKLETS
    spin_unlock_irqrestore(&(dev->lock), flags);
#endif

    SBLEAVE;
}

static void si3052_onhook_timer(unsigned long arg) {
    unsigned long flags;
    unsigned long now;
    si3052_dev *dev = (si3052_dev *)arg;

    SENTER;

    spin_lock_irqsave(&(dev->lock), flags);

    /* in case we are scheduled after going back onhook */
    if (dev->onhook) 
	goto back_onhook;

    /* reenable overload protection after offhook transition */
    sidregs(dev, S3052_REG_IC2, S3052_IC2_OPE, S3052_IC2_OPE);

    dev->enable_tx = 1;

    now = jiffies;
    dev->lastbatttime = now;
    /* start timer to check if there is a bettery present b/c we did not do it when we were onhook */
    if (!timer_pending(&(dev->battery_timer)))
	mod_timer(&(dev->battery_timer), now + msecs_to_jiffies(S3052_BATTERY_CHECK_TIME));

back_onhook:
    spin_unlock_irqrestore(&(dev->lock), flags);

    SLEAVE;
}


static int si3052_hooksig(struct dahdi_chan *chan, enum dahdi_txsig txsig) {
    unsigned long flags;
    si3052_dev *dev = (si3052_dev *)chan->pvt;

    SENTER;

    spin_lock_irqsave(&(dev->lock), flags);

    if ((txsig == DAHDI_TXSIG_OFFHOOK) && (dev->onhook)) {
	unsigned long now;
	
	SDPRN("DAHDI wants offhook\n");

	dev->onhook = 0;
	
	/* disable overload protection for offhook transition */
	sidregs(dev, S3052_REG_IC2, S3052_IC2_OPE, 0);
	
	/* go offhook */
	sidregs(dev, S3052_REG_DAAC1, S3052_DAAC1_OH, S3052_DAAC1_OH);
	
	now = jiffies;
	dev->onhook_timer.expires = now + msecs_to_jiffies(
				    128 /* programmed offhook switch latency */ 
				    + 17 /* resistor calibration */ 
				    + 256 /* ADC calibration */ 
				    + 2 /* 0.25ms signal propagation delay + 1.5ms filter delay */
				   );
	add_timer(&(dev->onhook_timer));
    } else if ((txsig == DAHDI_TXSIG_ONHOOK) && (!(dev->onhook))) {
	SDPRN("DAHDI wants onhook\n");

	dev->onhook = 1;
	dev->enable_tx = 0;

	/* in case it not fired yet */
	del_timer(&(dev->onhook_timer));

	/* go on hook */
	sidregs(dev, S3052_REG_DAAC1, S3052_DAAC1_OH, 0);
	
	/* reenable overload protection in case onhook timer did not run yet */
	sidregs(dev, S3052_REG_IC2, S3052_IC2_OPE, S3052_IC2_OPE);
	
	/* reset alarms */
	dev->lastbatt_signaledstate = 1;
	dahdi_alarm_channel(&(dev->chan), 0);
    }

    spin_unlock_irqrestore(&(dev->lock), flags);

    SLEAVE;

    return 0;
}

static void si3052_ringer_timer(unsigned long arg) {
    unsigned long flags;
    si3052_dev *dev = (si3052_dev *)arg;
    u8 reg;

    SENTER;

    spin_lock_irqsave(&(dev->lock), flags);

    /* in case we are scheduled after going offhook */
    if (!(dev->onhook)) 
	goto offhook;

    reg = sidregr(dev, S3052_REG_DAAC1); 
    if ((reg & S3052_DAAC1_RDT) != 0) {
	unsigned long now;

	/* SDPRN("it is still ringing\n"); */ /* too verbose */
	
	/* recheck later */
	now = jiffies;
	mod_timer(&(dev->ringer_timer), now + msecs_to_jiffies(S3052_RINGER_CHECK_TIME));
    } else {
	SDPRN("ringer gone\n");
	dahdi_hooksig(&(dev->chan), DAHDI_RXSIG_ONHOOK);
    }
    
offhook:
    spin_unlock_irqrestore(&(dev->lock), flags);

    SLEAVE;
}

static void si3052_battery_timer(unsigned long arg) {
    unsigned long flags;
    si3052_dev *dev = (si3052_dev *)arg;
    u8 reg;
    unsigned long now;

/*    SENTER; */ /* too verbose */

    spin_lock_irqsave(&(dev->lock), flags);

    if (dev->onhook) 
	goto back_onhook;

    reg = sidregr(dev, S3052_REG_LSS); 
    if ((reg & S3052_LSS_LCS) == S3052_LSS_LCS_BELOW_MIN) {
	if (dev->lastbatt_signaledstate) { /* if last state that we signaled was battery present */
	    now = jiffies;

	    /* check if battery is gone for enough time */
	    if (time_after(now, (dev->lastbatttime)+msecs_to_jiffies(fxo_modes[ireg].battdebounce))) {
		SDPRN("line power gone for sure\n");
		/* mark it so we won't signal it again */
		dev->lastbatt_signaledstate = 0; 
		dahdi_alarm_channel(&(dev->chan), DAHDI_ALARM_RED);
	    }
	}
    } else
	if (!(dev->lastbatt_signaledstate)) { 
	    SDPRN("line power back\n");
	    dev->lastbatttime = jiffies;

	    dev->lastbatt_signaledstate = 1;
	    dahdi_alarm_channel(&(dev->chan), 0);
	}

    /* reschedule ourselves to check again after a period of time */
    now = jiffies;
    mod_timer(&(dev->battery_timer), now + msecs_to_jiffies(S3052_BATTERY_CHECK_TIME));

back_onhook:
    spin_unlock_irqrestore(&(dev->lock), flags);

/*    SLEAVE; */
}

static void si3052_daa_irq(struct work_struct *work) {
    unsigned long flags;
    unsigned long now;
    si3052_dev *dev = container_of(work, si3052_dev, daa_irq_work);
    u8 reg, reg2;

    SENTER;

    spin_lock_irqsave(&(dev->lock), flags);

    /* read also current DAA interrupt register in case new one came between main irq handler and this function */
    reg = dev->daa_is | sidregr(dev, S3052_REG_IS);
    /* clear them */
    sidregw(dev, S3052_REG_IS, 0);

    if ((reg & S3052_IS_DODI) != 0) {
	/* useless */
    } 

    if (dev->onhook) {
	if ((reg & S3052_IS_RDTI) != 0) {
	    /* check for actual ring */
	    reg2 = sidregr(dev, S3052_REG_DAAC1); 
	    if ((reg2 & S3052_DAAC1_RDT) != 0) {
		SDPRN("got ringer\n");
	    
		dahdi_hooksig(&(dev->chan), DAHDI_RXSIG_RING);
	
		/* start or delay end of ring timer */
		now = jiffies;
		mod_timer(&(dev->ringer_timer), now + msecs_to_jiffies(S3052_RINGER_CHECK_TIME));
	    }
	}
    } else {
	reg2 = sidregr(dev, S3052_REG_IC4); 
	
	if ((reg & S3052_IS_ROVI) != 0) {
		/* check for actual overload */
		if ((reg2 & S3052_IC4_OVL) != 0) {
		    SDPRN("receive overload\n");
		    if (printk_ratelimit())
			dev_warn(&(dev->pdev->dev), "receive overload\n");
		}
	}
	
	if ((reg & S3052_IS_LCSI) != 0) {
		/* check for actual overload */
		if ((reg2 & S3052_IC4_OPD) != 0) {
		    SDPRN("line overload\n");
		    /* reset by zeroing OPE for a moment */
		    reg2 = sidregr(dev, S3052_REG_IC2);
		    sidregs(dev, S3052_REG_IC2, S3052_IC2_OPE, 0);
		    /* write back previous setting */
		    sidregw(dev, S3052_REG_IC2, reg2);
		    if (printk_ratelimit())
			dev_warn(&(dev->pdev->dev), "line feed overload\n");
		}
	}
    }

    spin_unlock_irqrestore(&(dev->lock), flags);

    SLEAVE;
}

static irqreturn_t si3052_irq(int irq, void *dev_id) {
    si3052_dev *dev = dev_id;
    irqreturn_t iret = IRQ_NONE;
    u32 reg, clear_ints = 0;
    int buffend = 0, buffmiddle = 0;

    spin_lock(&(dev->lock));

    reg = siregr(dev, S3052_REG_DMAIS);
	if ((reg & (S3052_DMAIS_MTO|S3052_DMAIS_PTA|S3052_DMAIS_PMA)) != 0) {
	    dev_err(&(dev->pdev->dev), "PCI problem, DMAIS = 0x%x\n", reg);
	    goto dma_problem;
	} else {
	    if ((reg & S3052_DMAIS_DWB) != 0) {
		SBDPRN("TX buffer empty, RX left=%u\n", siregr(dev, S3052_REG_DMARSTOP)-siregr(dev, S3052_REG_DMARCURR));
		
		/* mark tx buffer second part as empty */
		dev->txbuf_state &= ~S3052_BUF_SECOND;
		
		/* clear interrupt status */
		clear_ints |= S3052_DMAIS_DWB;

		buffend = 1;
		iret = IRQ_HANDLED;
	    } 
	    
	    if ((reg & S3052_DMAIS_DWA) != 0) {
		SBDPRN("TX buffer half empty, RX left=%u\n", siregr(dev, S3052_REG_DMARSTOP)-siregr(dev, S3052_REG_DMARCURR));

		/* mark tx buffer first part as empty */
		dev->txbuf_state &= ~S3052_BUF_FIRST;
	
		/* clear interrupt status */
		clear_ints |= S3052_DMAIS_DWA;
	
		buffmiddle = 1;
		iret = IRQ_HANDLED;
	    }

	    if ((reg & S3052_DMAIS_DRB) != 0) {
	    	SBDPRN("RX buffer empty, TX left=%u\n", siregr(dev, S3052_REG_DMAWSTOP)-siregr(dev, S3052_REG_DMAWCURR));
	
		/* mark rx buffer second part as full */
		dev->rxbuf_state &= ~S3052_BUF_SECOND;

		/* clear interrupt status */
		clear_ints |= S3052_DMAIS_DRB;

		buffend = 1;
		iret = IRQ_HANDLED;
	    } 
	    
	    if ((reg & S3052_DMAIS_DRA) != 0) {
	    	SBDPRN("RX buffer half empty, TX left=%u\n", siregr(dev, S3052_REG_DMAWSTOP)-siregr(dev, S3052_REG_DMAWCURR));

		/* mark rx buffer first part as full */
		dev->rxbuf_state &= ~S3052_BUF_FIRST;

		/* clear interrupt status */
		clear_ints |= S3052_DMAIS_DRA;
	
		buffmiddle = 1;
		iret = IRQ_HANDLED;
	    }
	}
    
	if ((buffend) && (!(dev->txbuf_state & S3052_BUF_SECOND)) && (!(dev->rxbuf_state & S3052_BUF_SECOND))) { /* both buffers reached their ends, try to rewind */
	    if ((dev->txbuf_state & S3052_BUF_FIRST) && (dev->rxbuf_state & S3052_BUF_FIRST)) { /* are the first parts of both buffers ready on time? */
		SBDPRN("first buffer part ready on time\n");

#ifndef S3052_AUTORESTART_DMA
		/* reset current DMA addresses to the beginning */
		siregs(dev, S3052_REG_DMAI, S3052_DMAI_DMAR, S3052_DMAI_DMAR);
		siregs(dev, S3052_REG_DMAI, S3052_DMAI_DMAR, 0);
#endif

		/* now second parts of buffers can be filled/flushed */
#ifdef S3052_BUFFER_TASKLETS
		tasklet_hi_schedule(&(dev->dma_buffers_task));
#else
		si3052_dma_buffers(dev);
#endif
	    } else { /* buffers not ready */
		SBDPRN("first buffer part not ready on time\n");

		/* disable DMA */
		siregs(dev, S3052_REG_DMAI, S3052_DMAI_DMAE, 0);

#ifdef S3052_BUFFER_TASKLETS
		tasklet_hi_schedule(&(dev->dma_buffers_task));
#else
		si3052_dma_buffers(dev);
#endif
	    }
	} else if ((buffmiddle) && (!(dev->txbuf_state & S3052_BUF_FIRST)) && (!(dev->rxbuf_state & S3052_BUF_FIRST))) {
	    SBDPRN("first buffer part ready to be processed\n");

	    /* first parts of both buffers can be filled/flushed */
#ifdef S3052_BUFFER_TASKLETS
		tasklet_hi_schedule(&(dev->dma_buffers_task));
#else
		si3052_dma_buffers(dev);
#endif
	}

    if ((reg & S3052_DMAIS_DIS) != 0) {
	SDPRN("DAA interrupt\n");

	/* save interrupt status for tasklet */
	dev->daa_is = sidregr(dev, S3052_REG_IS);

	/* clear DAA interrupts */
	sidregw(dev, S3052_REG_IS, 0);

	/* clear master DAA interrupt */
	clear_ints |= S3052_DMAIS_DIS;

	/* schedule handling */
	schedule_work(&(dev->daa_irq_work));
	    
	iret = IRQ_HANDLED;
    }

    /* now clear serviced interrupts in register */
    siregs(dev, S3052_REG_DMAIS, S3052_DMAIS_ALL, clear_ints);

    spin_unlock(&(dev->lock));

    return iret;

dma_problem:
    /* disable DMA */
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_DMAE, 0);

    /* disable DMA-related interrupts */
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_RBIE|S3052_DMAI_RAIE|S3052_DMAI_WAIE|S3052_DMAI_WBIE|S3052_DMAI_MAIE|S3052_DMAI_TAIE|S3052_DMAI_MTIE, 0);

    /* clear DMA and PCI interrupt status */
    siregs(dev, S3052_REG_DMAIS, S3052_DMAIS_ALL,
				     S3052_DMAIS_MTO|S3052_DMAIS_PTA|S3052_DMAIS_PMA|S3052_DMAIS_DRB|S3052_DMAIS_DRA|S3052_DMAIS_DWB|S3052_DMAIS_DWA);

    /* schedule DMA reprogram */
    tasklet_schedule(&(dev->dma_restart_task));

    spin_unlock(&(dev->lock));

    return IRQ_HANDLED;
}

static int si3052_init_daa(si3052_dev *dev) {
    unsigned int n;
    u32 val;
    char ver;

    for (n = 0; n < 10; n++) {
	if ((siregr(dev, S3052_REG_DMAIS) & S3052_DMAIS_DAA) == 0)
	    break;
	msleep(100);
    }

    if (n>=10) {
	dev_err(&(dev->pdev->dev), "timed out waiting for DAA\n");
	return -ETIMEDOUT;
    }

    /* reset DAA registers */
    sidregs(dev, S3052_REG_CTRL1, S3052_CTRL1_SR, S3052_CTRL1_SR);

    /* enable enhanced register set */
    sidregs(dev, S3052_REG_CTRL1, S3052_CTRL1_MAP, S3052_CTRL1_MAP);

    /* set 8000Hz rate */
    sidregs(dev, S3052_REG_SRC, S3052_SRC_SRC, S3052_SRC_SRC_8000);

    /* power up line side chip */
    sidregs(dev, S3052_REG_DAAC2, S3052_DAAC2_PDL, 0);

    for (n = 0; n < 10; n++) {
	if ((sidregr(dev, S3052_REG_LSS) & S3052_LSS_FDT) != 0)
	    break;
	msleep(100);
    }

    if (n>=10) {
	dev_err(&(dev->pdev->dev), "timed out waiting for ISOcap link frame lock\n");
	return -ETIMEDOUT;
    }

    val = sidregr(dev, S3052_REG_SSR);
    val &= S3052_SSR_LSID;
    switch (val) {
	case S3052_SSR_LSID_3017:
	    ver = '7';
	    dev->daa_type = S3052_DAA_3017;
	    break;

	case S3052_SSR_LSID_3018:
	    ver = '8';
	    dev->daa_type = S3052_DAA_3018;
	    break;

	case S3052_SSR_LSID_3011:
	    ver = '1';
	    dev->daa_type = S3052_DAA_3011;
	    break;
    
	default:
	    dev_err(&(dev->pdev->dev), "unknown line-side DAA 0x%x\n", val);
	    return -EINVAL;
    }
    
    dev_info(&(dev->pdev->dev), "Si301%c line-side DAA detected\n", ver);

    /* disable all DAA interrupt sources */
    sidregs(dev, S3052_REG_IM, S3052_IM_LCSM|S3052_IM_DODM|S3052_IM_BTDM|S3052_IM_FDTM|S3052_IM_ROVM|S3052_IM_RDTM, 0);

    /* clear DAA interrupt status */
    sidregs(dev, S3052_REG_IS, S3052_IS_LCSI|S3052_IS_BTDI|S3052_IS_FDTI|S3052_IS_ROVI|S3052_IS_RDTI, 0);

    /* disable on-board speaker and digital loopback*/
    sidregs(dev, S3052_REG_CTRL1, S3052_CTRL1_PWME|S3052_CTRL1_PWME, 0);
    
    /* enable hybrid and receive path, disable analog loopback and DAA watchdog */
    sidregs(dev, S3052_REG_CTRL2, S3052_CTRL2_RXE|S3052_CTRL2_HBE|S3052_CTRL2_AL|S3052_CTRL2_WDTE, S3052_CTRL2_RXE|S3052_CTRL2_HBE);
    
    /* enable on-hook line monitor and place line device on-hook */
    sidregs(dev, S3052_REG_DAAC1, S3052_DAAC1_OH|S3052_DAAC1_ONHM, S3052_DAAC1_ONHM);

    dev->onhook = 1;
    dev->enable_tx = 0;

    /* disable digital data loopback */
    sidregs(dev, S3052_REG_DAAC3, S3052_DAAC3_DDL, 0);

    /* normal calibration operation, auto-calibration enabled, overload protection enabled, billing tone protection disabled */
    sidregs(dev, S3052_REG_IC2, S3052_IC2_CALZ|S3052_IC2_CALD|S3052_IC2_OPE|S3052_IC2_BTE, S3052_IC2_OPE);

    /* resistor calibration enabled */
    sidregs(dev, S3052_REG_RCAL, S3052_RCAL_RCALD, 0);

    /* 128 ms on hook speed */
    sidregs(dev, S3052_REG_DAAC4, S3052_DAAC4_FOH, S3052_DAAC4_FOH_128);

    /* set OHS, OHS2, SQ, RZ, RT, ILIM, DCV, MINI, ACT and ACT2 */
    /* warn user when trying to set a non-supported setting */
    sidregs(dev, S3052_REG_IC1, S3052_IC1_RT, 0);
    if (fxo_modes[ireg].rt) {
	if (dev->daa_type != S3052_DAA_3018) 
	    dev_warn(&(dev->pdev->dev), "your current region setting (%s) requests RT but current line-side DAA doesn't support it", region_param);
	else 
	    sidregs(dev, S3052_REG_IC1, S3052_IC1_RT, S3052_IC1_RT);
    }
    
    sidregs(dev, S3052_REG_IC1, S3052_IC1_RZ, 0);
    if (fxo_modes[ireg].rz) {
	if (dev->daa_type != S3052_DAA_3018) 
	    dev_warn(&(dev->pdev->dev), "your current region setting (%s) requests RZ but current line-side DAA doesn't support it", region_param);
	else 
	    sidregs(dev, S3052_REG_IC1, S3052_IC1_RZ, S3052_IC1_RZ);
    }

    sidregs(dev, S3052_REG_IC1, S3052_IC1_OHS, 0);
    sidregs(dev, S3052_REG_DAAC4, S3052_DAAC4_OHS2, 0);
    sidregs(dev, S3052_REG_SQ, S3052_SQ_SQ, S3052_SQ_NORM);
    if ((fxo_modes[ireg].ohs) || (fxo_modes[ireg].ohs2)) {
	if (dev->daa_type != S3052_DAA_3018) 
	    dev_warn(&(dev->pdev->dev), "your current region setting (%s) requests OHS and/or OHS2 but current line-side DAA doesn't support it", region_param);
	else {
	    sidregs(dev, S3052_REG_IC1, S3052_IC1_OHS, fxo_modes[ireg].ohs ? S3052_IC1_OHS : 0);
	    sidregs(dev, S3052_REG_DAAC4, S3052_DAAC4_OHS2, fxo_modes[ireg].ohs2 ? S3052_DAAC4_OHS2 : 0);
	    if (fxo_modes[ireg].ohs) /* Australia mode, set spark quenching */
		sidregs(dev, S3052_REG_SQ, S3052_SQ_SQ, S3052_SQ_AUSTRALIA);
	}
    }
    
    sidregs(dev, S3052_REG_DCT, S3052_DCT_ILIM, 0);
    if (fxo_modes[ireg].ilim) {
	if (dev->daa_type == S3052_DAA_3017) 
	    dev_warn(&(dev->pdev->dev), "your current region setting (%s) requests ILIM but current line-side DAA doesn't support it", region_param);
	else 
	    sidregs(dev, S3052_REG_DCT, S3052_DCT_ILIM, S3052_DCT_ILIM);
    }

    sidregs(dev, S3052_REG_DCT, S3052_DCT_DCV, S3052_DCT_DCV_335);
    if (fxo_modes[ireg].dcv != (S3052_DCT_DCV_335 >> S3052_DCT_DCV_SHIFT)) {
	if (dev->daa_type != S3052_DAA_3018)
	    dev_warn(&(dev->pdev->dev), "your current region setting (%s) requests DCV!=3.35V but current line-side DAA doesn't support that", region_param);
	else 
	    sidregs(dev, S3052_REG_DCT, S3052_DCT_DCV, fxo_modes[ireg].dcv << S3052_DCT_DCV_SHIFT);
    }

    sidregs(dev, S3052_REG_DCT, S3052_DCT_MINI, S3052_DCT_MINI_10);
    if (fxo_modes[ireg].mini != (S3052_DCT_MINI_10 >> S3052_DCT_MINI_SHIFT)) {
	if (dev->daa_type != S3052_DAA_3018)
	    dev_warn(&(dev->pdev->dev), "your current region setting (%s) requests MINI!=10mA but current line-side DAA doesn't support that", region_param);
	else 
	    sidregs(dev, S3052_REG_DCT, S3052_DCT_MINI, fxo_modes[ireg].mini << S3052_DCT_MINI_SHIFT);
    }

    sidregs(dev, S3052_REG_IC1, S3052_IC1_ACT|S3052_IC1_ACT2, 0);
    if ((fxo_modes[ireg].act) || (fxo_modes[ireg].act2)) {
	if ((dev->daa_type == S3052_DAA_3017) || 
	    ((dev->daa_type == S3052_DAA_3011) && ((fxo_modes[ireg].act!=0) || (fxo_modes[ireg].act2!=1))))
	    dev_warn(&(dev->pdev->dev), "your current region setting (%s) requests AC termination setting which current line-side DAA doesn't support", region_param);
	else {
	    sidregs(dev, S3052_REG_IC1, S3052_IC1_ACT|S3052_IC1_ACT2, (fxo_modes[ireg].act ? S3052_IC1_ACT : 0) | (fxo_modes[ireg].act2 ? S3052_IC1_ACT2 : 0));
	}
    }

    /* set 50 ohm DC termination */
    sidregs(dev, S3052_REG_DCT, S3052_DCT_DCR, 0);

    /* set no RX or TX gain */
    sidregs(dev, S3052_REG_TRG, S3052_TRG_ATX|S3052_TRG_ARX, S3052_TRG_ATX_0|S3052_TRG_ARX_0);

    /* set maximum time between zero crossings in 2ms units. 25 = 50ms = 10 Hz min. ringer frequency */
    sidregs(dev, S3052_REG_RVC3, S3052_RVC3_RAS, 25<<S3052_RVC3_RAS_SHIFT);

    /* set minimum time between zero crossings. formula is in datasheet, 22 means 83.3 Hz max. ringer frequency */
    sidregs(dev, S3052_REG_RVC1, S3052_RVC1_RMX, 22<<S3052_RVC1_RMX_SHIFT);

    /* set minimum ringing time */
    if (min_ringer_time < 150)
	sidregs(dev, S3052_REG_RVC2, S3052_RVC2_RCC, S3052_RVC2_RCC_100);
    else if (min_ringer_time < 200)
	sidregs(dev, S3052_REG_RVC2, S3052_RVC2_RCC, S3052_RVC2_RCC_150);
    else if (min_ringer_time < 256)
	sidregs(dev, S3052_REG_RVC2, S3052_RVC2_RCC, S3052_RVC2_RCC_200);
    else if (min_ringer_time < 384)
	sidregs(dev, S3052_REG_RVC2, S3052_RVC2_RCC, S3052_RVC2_RCC_256);
    else if (min_ringer_time < 512)
	sidregs(dev, S3052_REG_RVC2, S3052_RVC2_RCC, S3052_RVC2_RCC_384);
    else if (min_ringer_time < 640)
	sidregs(dev, S3052_REG_RVC2, S3052_RVC2_RCC, S3052_RVC2_RCC_512);
    else
	sidregs(dev, S3052_REG_RVC2, S3052_RVC2_RCC, S3052_RVC2_RCC_1024);

    /* set ring timeout to 128 ms after last ring threshold crossing. 128ms units. */
    sidregs(dev, S3052_REG_RVC2, S3052_RVC2_RTO, 1<<S3052_RVC2_RTO_SHIFT);

    /* no delay before raising ringer interrupt. 256ms units. */
    sidregs(dev, S3052_REG_RVC2, S3052_RVC2_RDLY2, 0);
    sidregs(dev, S3052_REG_RVC1, S3052_RVC1_RDLY, 0<<S3052_RVC1_RDLY_SHIFT);

    /* enable ring validation */
    sidregs(dev, S3052_REG_RVC3, S3052_RVC3_RNGV, S3052_RVC3_RNGV);

    /* RDT bit follows ring validation */
    sidregs(dev, S3052_REG_IC3, S3052_IC3_RFWE, 0);

    /* enable DAA interrupt sources ring detect, receive overload, line drop and line overload */
    sidregs(dev, S3052_REG_IM, S3052_IM_LCSM|S3052_IM_DODM|S3052_IM_ROVM|S3052_IM_RDTM, S3052_IM_LCSM|S3052_IM_DODM|S3052_IM_ROVM|S3052_IM_RDTM);
    
    return 0;
}

static int si3052_init(si3052_dev *dev) {
    int ret;
    
    ret = pci_enable_device(dev->pdev);
    if (ret != 0) {
	dev_err(&(dev->pdev->dev), "pci_enable_device() failed with error %d\n", ret);
	goto ret_err;
    }

    /* BAR0 should be a MMIO interface */
    if (!(pci_resource_flags(dev->pdev, 0) & IORESOURCE_MEM)) {
	dev_err(&(dev->pdev->dev), "first BAR is not MMIO\n");
	ret = -ENODEV;
	goto ret_disable;
    }
    
    if (pci_resource_len(dev->pdev, 0) < S3052_REG_SQ) {
	dev_err(&(dev->pdev->dev), "MMIO too short\n");
	ret = -ENODEV;
	goto ret_disable;
    }

    dev->mmio = pci_resource_start(dev->pdev, 0);
    dev->mmio_ptr = ioremap(dev->mmio, pci_resource_len(dev->pdev, 0));

    /* disable all interrupt sources */
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_MTIE|S3052_DMAI_DIE|S3052_DMAI_WIE|S3052_DMAI_TAIE|S3052_DMAI_MAIE|S3052_DMAI_RBIE|S3052_DMAI_RAIE|S3052_DMAI_WAIE|S3052_DMAI_WBIE, 0);
    
    /* set direct data mode */
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_DM, S3052_DMAI_DM_DIRECT);
    
    /* set DMA disable, DMA mode, sticky DMA status bits */
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_DMAE|S3052_DMAI_DMAM|S3052_DMAI_DMAI, 
#ifndef S3052_AUTORESTART_DMA
    S3052_DMAI_DMAM
#else
    0
#endif
    );

    /* powerup DAA */
    siregs(dev, S3052_REG_DAAC2, S3052_DAAC2_PDN, 0);

    /* reset DMA engine and DAA */
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_DRST|S3052_DMAI_PRST, S3052_DMAI_DRST|S3052_DMAI_PRST);
    
    /* disable PCI chip watchdog */
    siregs(dev, S3052_REG_WATCH, S3052_WATCH_WTC, 0);

    /* unreset DMA engine and DAA */
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_DRST|S3052_DMAI_PRST, 0);

    /* set crystal value to 32kHz */
    siregs(dev, S3052_REG_WATCH, S3052_WATCH_XTAL, S3052_WATCH_XTAL);

    /* clear PCI chip interrupt status */
    siregs(dev, S3052_REG_DMAIS, S3052_DMAIS_ALL,
				S3052_DMAIS_MTO|S3052_DMAIS_DIS|S3052_DMAIS_WIS|S3052_DMAIS_PTA|S3052_DMAIS_PMA|S3052_DMAIS_DRB|S3052_DMAIS_DRA|S3052_DMAIS_DWA|S3052_DMAIS_DWB);

    ret = si3052_init_daa(dev);
    if (ret != 0) {
	dev_err(&(dev->pdev->dev), "DAA init failed with error %d\n", ret);
	goto ret_unmap;
    }

    ret = request_irq(dev->pdev->irq, si3052_irq, IRQF_SHARED, dev_name(&(dev->pdev->dev)), dev);
    if (ret != 0) {
	dev_err(&(dev->pdev->dev), "request_irq() failed with error %d\n", ret);
	goto ret_unmap;
    }

    /* enable PCI bus master */
    pci_set_master(dev->pdev);

    return 0;

ret_unmap:
    iounmap(dev->mmio_ptr);

ret_disable:
    pci_disable_device(dev->pdev);

ret_err:
    return ret;
}

static const struct dahdi_span_ops si3052_span_ops = {
        .owner = THIS_MODULE,
        .hooksig = si3052_hooksig /*,
        .open = si3052_open,
        .close = si3052_close, */
};

static int si3052_probe(struct pci_dev *pdev, const struct pci_device_id *id) {
    int ret;
    si3052_dev *dev;
    
    dev = kzalloc(sizeof(*dev), GFP_KERNEL);
    if (!dev) {
	dev_err(&(pdev->dev), "kzalloc() failed\n");
	return -ENOMEM;
    }
    
    dev->pdev = pdev;
    dev->devno = num_devices++;
    dev->lock = SPIN_LOCK_UNLOCKED;

    /* alloc DMA buffers */
    dev->rxbuf = dma_alloc_coherent(&(pdev->dev), S3052_DMABUF_SIZE, &(dev->rxbuf_h), GFP_KERNEL);
    if (!(dev->rxbuf)) {
	dev_err(&(pdev->dev), "dma_alloc_coherent() for RX failed\n");
	ret = -ENOMEM;
	goto ret_free;
    }

    dev->txbuf = dma_alloc_coherent(&(pdev->dev), S3052_DMABUF_SIZE, &(dev->txbuf_h), GFP_KERNEL);
    if (!(dev->txbuf)) {
	dev_err(&(pdev->dev), "dma_alloc_coherent() for TX failed\n");
	ret = -ENOMEM;
	goto ret_unrdma;
    }

    pci_set_drvdata(pdev, dev);

    tasklet_init(&(dev->dma_restart_task), si3052_dma_restart, dev);
#ifdef S3052_BUFFER_TASKLETS
    tasklet_init(&(dev->dma_buffers_task), si3052_dma_buffers, dev);
#endif

    INIT_WORK(&(dev->daa_irq_work), si3052_daa_irq);

    init_timer(&(dev->onhook_timer));
    dev->onhook_timer.function = si3052_onhook_timer;
    dev->onhook_timer.data = dev;
    init_timer(&(dev->ringer_timer));
    dev->ringer_timer.function = si3052_ringer_timer;
    dev->ringer_timer.data = dev;
    init_timer(&(dev->battery_timer));
    dev->battery_timer.function = si3052_battery_timer;
    dev->battery_timer.data = dev;

    /* no alarms signaled by default */
    dev->lastbatt_signaledstate = 1;

    /* init device */
    ret = si3052_init(dev);
    if (ret != 0)
	goto ret_untsk;

    dev->span.ops = &si3052_span_ops;
    snprintf(dev->span.name, sizeof(dev->span.name), S3052_DRV_NAME "/%u", dev->devno);
    snprintf(dev->span.desc, sizeof(dev->span.desc), S3052_DRV_NAME " card number %u", dev->devno);
    dev->span.spantype = "FXO";
    dev->span.irq = pdev->irq;
    dev->span.manufacturer = "SiLabs";
    snprintf(dev->span.devicetype, sizeof(dev->span.devicetype), "Si3052");
    snprintf(dev->span.location, sizeof(dev->span.location), "%s", dev_name(&(pdev->dev)));
    dev->span.flags = DAHDI_FLAG_RBS;
    dev->span.deflaw = use_ulaw ? DAHDI_LAW_MULAW : DAHDI_LAW_ALAW;
    dev->chans = &(dev->chan);
    dev->span.chans = &(dev->chans);
    dev->span.channels = 1;

    dev->chan.chanpos = 1;
    snprintf(dev->chan.name, sizeof(dev->chan.name), S3052_DRV_NAME "/%u/0", dev->devno);
    dev->chan.sigcap = DAHDI_SIG_FXSKS | DAHDI_SIG_FXSLS;
    dev->chan.pvt = dev;

    ret = dahdi_register(&(dev->span), 0);
    if (ret != 0) {
	dev_err(&(pdev->dev), "dahdi_register() with error %d\n", ret);
	goto ret_untsk;
    }

    /* program DMA address, prefill tx buffer and enable DMA interrupts */
    si3052_dma_program(dev);

    /* enable DAA interrupt */
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_DIE, S3052_DMAI_DIE);

    return 0;

ret_untsk:
    del_timer_sync(&(dev->battery_timer));
    del_timer_sync(&(dev->onhook_timer));
    del_timer_sync(&(dev->ringer_timer));
    
    tasklet_kill(&(dev->dma_restart_task));
#ifdef S3052_BUFFER_TASKLETS
    tasklet_kill(&(dev->dma_buffers_task));
#endif

    cancel_work_sync(&(dev->daa_irq_work));
    dma_free_coherent(&(pdev->dev), S3052_DMABUF_SIZE, dev->txbuf, dev->txbuf_h);

ret_unrdma:
    dma_free_coherent(&(pdev->dev), S3052_DMABUF_SIZE, dev->rxbuf, dev->rxbuf_h);

ret_free:
    num_devices--;
    kfree(dev);

    return ret;
}

static void si3052_remove(struct pci_dev *pdev) {
    si3052_dev *dev = pci_get_drvdata(pdev);

    if (!dev) 
	return;

    /* disable all interrupt sources */
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_MTIE|S3052_DMAI_DIE|S3052_DMAI_WIE|S3052_DMAI_TAIE|S3052_DMAI_MAIE|S3052_DMAI_RBIE|S3052_DMAI_RAIE|S3052_DMAI_WAIE|S3052_DMAI_WBIE, 0);

    free_irq(pdev->irq, dev);

    tasklet_kill(&(dev->dma_restart_task));
#ifdef S3052_BUFFER_TASKLETS
    tasklet_kill(&(dev->dma_buffers_task));
#endif

    /* disable interrupt sources yet again, in case DMA restart tasklet reenabled them */
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_MTIE|S3052_DMAI_DIE|S3052_DMAI_WIE|S3052_DMAI_TAIE|S3052_DMAI_MAIE|S3052_DMAI_RBIE|S3052_DMAI_RAIE|S3052_DMAI_WAIE|S3052_DMAI_WBIE, 0);

    /* disable DMA */
    siregs(dev, S3052_REG_DMAI, S3052_DMAI_DMAE, 0);

    cancel_work_sync(&(dev->daa_irq_work));

    /* set onhook so onhook timer won't schedule battery timer */
    dev->onhook = 1;

    /* ringer timer and battery timer can call DAHDI code */
    del_timer_sync(&(dev->ringer_timer));
    del_timer_sync(&(dev->battery_timer));

    dahdi_unregister(&(dev->span));

    del_timer_sync(&(dev->onhook_timer));

    /* go onhook */
    sidregs(dev, S3052_REG_DAAC1, S3052_DAAC1_OH, 0);

    iounmap(dev->mmio_ptr);

    dma_free_coherent(&(pdev->dev), S3052_DMABUF_SIZE, dev->txbuf, dev->txbuf_h);
    dma_free_coherent(&(pdev->dev), S3052_DMABUF_SIZE, dev->rxbuf, dev->rxbuf_h);

    pci_disable_device(dev->pdev);

    num_devices--;
    kfree(dev);
}

static const struct pci_device_id si3052_pci_ids[] = {
    { PCI_DEVICE(0x1057, 0x3052) },
    { /* END */ }
};

static struct pci_driver si3052_driver = {
    .name = S3052_DRV_NAME,
    .id_table = si3052_pci_ids,
    .probe = si3052_probe,
    .remove = si3052_remove /* ,
    .suspend = si3052_suspend,
    .resume = si3052_resume */
};

static int __init si3052_module_init(void) {    
    if ((!region_param) || (!region_param[0])) {
	printk(KERN_ERR S3052_DRV_NAME ": operating region unset\n");
    } else  {
	for (ireg = 0; ireg < sizeof(fxo_modes)/sizeof(fxo_modes[0]); ireg++)
	    if (strcmp(fxo_modes[ireg].name, region_param) == 0)
		goto region_ok;

	if (ireg >= sizeof(fxo_modes)/sizeof(fxo_modes[0])) 
	    printk(KERN_ERR S3052_DRV_NAME ": operating region %s not found\n", region_param);
    }

    printk(KERN_ERR S3052_DRV_NAME ": load the driver with region_param=<REGION>, where region is a region from fxo_modes.h\n");
    return -EINVAL;
    
region_ok:
    if (min_ringer_time > 1024) {
	printk(KERN_ERR S3052_DRV_NAME ": maximum supported by DAA minimum ringer time is 1024ms\n");
	return -EINVAL;
    } else if (min_ringer_time < 100) {
	printk(KERN_ERR S3052_DRV_NAME ": minimum supported by DAA ringer time is 100ms\n");
	return -EINVAL;
    }
    
    return pci_register_driver(&si3052_driver);
}

static void __exit si3052_module_exit(void) {
    pci_unregister_driver(&si3052_driver);
}

module_init(si3052_module_init);
module_exit(si3052_module_exit);
MODULE_AUTHOR("Maciej Szmigiero <mhej@o2.pl>");
MODULE_DESCRIPTION(S3052_DRV_NAME ": driver for single-port FXO cards based on Si3052 chip + Si3011/17/18 DAA");
MODULE_DEVICE_TABLE(pci, si3052_pci_ids);
MODULE_LICENSE("GPL");
