/*
 * Rhino RCB FXO / FXS Interface Driver
 *
 * Release version 10/10/10
 *
 * Written by
 *          Bob Conklin <helpdesk@rhinoequipment.com>
 *          Bryce Chidester <helpdesk@rhinoequipment.com>
 *          Matthew Gessner <helpdesk@rhinoequipment.com>
 *
 * Based on Digium's TDM400P TDM FXS/FXO
 * and Zapata Telephony's Zaptel Telephony Interface
 *
 * Copyright (C) 2005-2010, Rhino Equipment Corp.
 * Copyright (C) 2001-2005, Digium, Inc.
 * Copyright (C) 2001 Jim Dixon / Zapata Telephony.
 *
 * All rights reserved.
 *
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
 * * * * * * * * * * * * * NO WARRANTY * * * * * * * * * * * * * * * * *
 *
 * THE PROGRAM IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED INCLUDING, WITHOUT
 * LIMITATION, ANY WARRANTIES OR CONDITIONS OF TITLE, NON-INFRINGEMENT,
 * MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE. Each Recipient is
 * solely responsible for determining the appropriateness of using and
 * distributing the Program and assumes all risks associated with its
 * exercise of rights under this Agreement, including but not limited to
 * the risks and costs of program errors, damage to or loss of data,
 * programs or equipment, and unavailability or interruption of operations.
 *
 * * * * * * * * * * * * DISCLAIMER OF LIABILITY * * * * * * * * * * * *
 *
 * NEITHER RECIPIENT NOR ANY CONTRIBUTORS SHALL HAVE ANY LIABILITY FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING WITHOUT LIMITATION LOST PROFITS), HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 * USE OR DISTRIBUTION OF THE PROGRAM OR THE EXERCISE OF ANY RIGHTS GRANTED
 * HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#define USE_G168_DSP

/* <linux>/include */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/fcntl.h>
#include <linux/string.h>
#include <linux/delay.h>

#include <asm/types.h>
#include <asm/mman.h>
#include <asm/io.h>
#include <asm/stat.h>
#include <asm/page.h>
#include <linux/firmware.h>

#include "rcbfx.h"
#include <dahdi/rcbfx_ioctl.h>

#ifdef USE_G168_DSP
#include "GpakCust.h"
#include "GpakApi.h"
#endif

struct rcb_card_desc {
	char *name;
	unsigned int type_idx;
	unsigned int num_chans;
	unsigned int num_slots;
	unsigned int hw_ver_min;
};

#define FLAG_2CHAN_SPI (1 << 0)

static struct rcb_card_desc rcb8fxx = { "Rhino RCB8FXX", FLAG_2CHAN_SPI, 8, 8, 8 };
static struct rcb_card_desc rcb24fxs = { "Rhino RCB24FXS", FLAG_2CHAN_SPI, 24, 32, 8 };
static struct rcb_card_desc rcb24fxx = { "Rhino RCB24FXX", FLAG_2CHAN_SPI, 24, 32, 8 };
static struct rcb_card_desc rcb24fxo = { "Rhino RCB24FXO", FLAG_2CHAN_SPI, 24, 32, 8 };
static struct rcb_card_desc rcb4fxo = { "Rhino RCB4FXO", FLAG_2CHAN_SPI, 4, 4, 7 };

static struct rcb_card_t *ifaces[RH_MAX_IFACES];

static int show_pointers = 0;
static int debug = 0;
/* static int debug = (DEBUG_MAIN | DEBUG_INTS | DEBUG_DSP); */
/* static int debug = (DEBUG_MAIN | DEBUG_INTS | DEBUG_DSP | DEBUG_SIG); */
static int ec_fxo_alg = 0;		/* B side on */
static int ec_fxs_alg = 0xffffff;	/* A side on */
static int use_ec_fxo = 0;		/* B side from alg */
static int use_ec_fxs = 0xffffff;	/* A side from alg */
static int use_ec_zap = -1;		/* Enable zaptel echo can */
static int use_ec = -1;			/* Switch in DSP's TDM bus */
static int force_fw = 0;
static int no_ec = 0;
static int nlp_type = 3;
/* Internal results of calculations */
static int zt_ec_chanmap = 0;
static int fxs_alg_chanmap = 0;
static int fxo_alg_chanmap = 0;
static int use_fxs_chanmap = 0;
static int use_fxo_chanmap = 0;
static int lvs[24];
static int reg_val[24];
static int battime = 100;
static int reg_addr = 0;
static int arr_argc = 24;

void *mmap(void *, size_t, int, int, int, off_t);
int open(const char *path, int oflags);

static const char *rcbfx_firmware = "rcbfx.fw";

static int rcbfx_echocan_create(struct dahdi_chan *chan, struct dahdi_echocanparams *ecp,
								struct dahdi_echocanparam *p,
								struct dahdi_echocan_state **ec);
static void rcbfx_echocan_free(struct dahdi_chan *chan, struct dahdi_echocan_state *ec);

static const struct dahdi_echocan_features my_ec_features = {
	.NLP_automatic = 1,
	.CED_tx_detect = 1,
	.CED_rx_detect = 1,
};

static const struct dahdi_echocan_ops my_ec_ops = {
	.name = "RCBFX_HWEC",
	.echocan_free = rcbfx_echocan_free,
};

static inline void rcb_card_transmit(struct rcb_card_t *rcb_card, unsigned char ints)
{
	dahdi_transmit(&rcb_card->span);
}

static inline void rcb_card_receive(struct rcb_card_t *rcb_card, unsigned char ints)
{
	if (!rcb_card->dsp_up)
		dahdi_ec_span(&rcb_card->span);
	dahdi_receive(&rcb_card->span);
}

static int rcb_dahdi_chan_rbsbits(struct dahdi_chan *chan, int bits)
{
	int channum, high_nib, regnum;
	struct dahdi_span *span = chan->span;
	struct rcb_card_t *rcb_card = container_of(span, struct rcb_card_t, span);
	unsigned long flags;

	if (debug & DEBUG_SIG)
		printk("rcbfx %d: RBS bits Setting bits to %d on channel %s\n", rcb_card->pos + 1,
			   bits, chan->name);

	spin_lock_irqsave(&rcb_card->lock, flags);

	channum = chan->chanpos - 1;	/* Zero base */
	high_nib = 4 * (channum & 1);
	regnum = channum >> 1;

	*(volatile __u8 *) (rcb_card->memaddr + RCB_TXSIG0 + regnum) &= ~(0xF << high_nib);
	*(volatile __u8 *) (rcb_card->memaddr + RCB_TXSIG0 + regnum) |= bits << high_nib;
	if (debug & DEBUG_SIG)
		printk("rcbfx %d: txsig0 = %x\n", rcb_card->pos + 1,
			   *(volatile __u32 *) (rcb_card->memaddr + RCB_TXSIG0));

	spin_unlock_irqrestore(&rcb_card->lock, flags);

	return 0;
}

static void rcb_card_check_sigbits(struct rcb_card_t *rcb_card, int status)
{
	int rxs, regnum;

	if (debug & DEBUG_SIG)
		printk("rcbfx %d: Checking sigbits %x configged %x\n", rcb_card->pos + 1, status,
			   rcb_card->chans_configed);

	if (rcb_card->chans_configed) {
		for (regnum = 0; regnum < 12; regnum++) {	/* 12 bytes of sig data to check */
			if (status & (1 << regnum)) {
				rxs = *(volatile __u8 *) (rcb_card->memaddr + RCB_RXSIG0 + regnum);
				if (rcb_card->num_chans > (regnum * 2)) {	/* make sure the chan exists */
					if (debug & DEBUG_SIG)
						printk("rcbfx %d: rbsbits channel: %x, bits: %x\n",
							   rcb_card->pos + 1, regnum * 2, (rxs & 0xf));
					dahdi_rbsbits(rcb_card->span.chans[regnum * 2], (rxs & 0xf));
				} else if (debug & DEBUG_SIG)
					printk
						("rcbfx %d: invalid signaling data recieved channel: %d, bits: %x\n",
						 rcb_card->pos + 1, regnum * 2, (rxs & 0xf));

				if (rcb_card->num_chans > (regnum * 2 + 1)) {	/* make sure the chan exists */
					if (debug & DEBUG_SIG)
						printk("rcbfx %d: rbsbits channel: %x, bits: %x\n",
							   rcb_card->pos + 1, regnum * 2 + 1, ((rxs & 0xf0) >> 4));
					dahdi_rbsbits(rcb_card->span.chans[regnum * 2 + 1],
								  (rxs & 0xf0) >> 4);
				} else if (debug & DEBUG_SIG)
					printk
						("rcbfx %d: invalid signaling data recieved channel: %d, bits: %x\n",
						 rcb_card->pos + 1, regnum * 2 + 1, ((rxs & 0xf0) >> 4));
			}
		}
	} else if (debug & DEBUG_SIG)
		printk("rcbfx %d: Channels not configured for signaling yet !!\n",
			   rcb_card->pos + 1);

}

static unsigned short int rcb_card_dsp_ping(struct rcb_card_t *rcb_card);

DAHDI_IRQ_HANDLER(rcb_card_interrupt)
{
	struct rcb_card_t *rcb_card = dev_id;
	int status, upd_state, regnum;
	__u8 ints, regval;

	/* read flancter */
	status = *(volatile __u16 *) (rcb_card->memaddr + RCB_RXSIGSTAT);

	if (status) {
		if (debug & DEBUG_SIG)
			printk("RXCHANGE flags = %x\n", status);
		/* reset flancter */
		*(volatile __u16 *) (rcb_card->memaddr + RCB_RXSIGSTAT) = status;
		if (debug & DEBUG_SIG)
			printk("RXCHANGE flags = %x\n",
				   *(volatile __u16 *) (rcb_card->memaddr + RCB_RXSIGSTAT));

		rcb_card_check_sigbits(rcb_card, status);
	}

	else if (*(volatile __u8 *) (rcb_card->memaddr + RCB_INTSTAT) & 0x02) {
		/* int acknowledge */
		*(volatile __u8 *) (rcb_card->memaddr + RCB_CONTROL) |= INT_ACK;
		*(volatile __u8 *) (rcb_card->memaddr + RCB_CONTROL) &= ~INT_ACK;

		/* read DMA address pointer MSB */
		ints = *(volatile __u8 *) (rcb_card->memaddr + RCB_INTSTAT) & 0x01;

		if ((rcb_card->intcount < 10) && (debug))
			printk("rcbfx %d: INT count %d PTR %x\n", rcb_card->pos + 1,
				   rcb_card->intcount,
				   *(volatile __u32 *) (rcb_card->memaddr + RCB_TDM_PTR));

		rcb_card->intcount++;
		rcb_card_receive(rcb_card, ints);
		rcb_card_transmit(rcb_card, ints);


		if ((rcb_card->intcount & RCB_LVSSAMP) == 0) {
			for (regnum = 0; regnum < 24; regnum++) {
				regval = *(volatile __u8 *) (rcb_card->memaddr + RCB_LVSTAB + regnum);
				if (regval & 0x80) {	/* negative */
					regval &= 0x7f;
					lvs[regnum] = -(0x80 - regval);
				} else
					lvs[regnum] = regval;
			}
		}

		if (reg_addr != rcb_card->oldreg_addr) {
			if (debug)
				printk("New reg_addr = %x\n", reg_addr);
			rcb_card->oldreg_addr = reg_addr;
			*(volatile __u8 *) (rcb_card->memaddr + RCB_REGADDR) = reg_addr;
			rcb_card->read_on_int = rcb_card->intcount + RCB_REGTIME;
		}

		if (rcb_card->intcount == rcb_card->read_on_int) {
			if (debug)
				printk("updating valuesfor reg %x\n", reg_addr);
			for (regnum = 0; regnum < 24; regnum++) {
				reg_val[regnum] =
					*(volatile __u8 *) (rcb_card->memaddr + RCB_REGTAB + regnum);
			}
		}

	}

	else
		return IRQ_NONE;

	upd_state = (*(volatile __u32 *) (rcb_card->memaddr + RCB_TXSIGSTAT) & 0x08000);
	if ((!upd_state) && (rcb_card->param_upd_state)) {
		printk("parameters accepted and set\n");
		rcb_card->param_upd_state = 0;
	}
	if ((upd_state) && (!rcb_card->param_upd_state)) {
		printk("parameters loaded to card\n");
		rcb_card->param_upd_state = 1;
	}

	return IRQ_RETVAL(1);
}

static int rcb_dahdi_chan_ioctl(struct dahdi_chan *chan, unsigned int cmd,
								unsigned long data)
{
	int regnum;
	struct rcb_card_params_t rcb_card_params;
	struct dahdi_span *span = chan->span;
	struct rcb_card_t *rcb_card = container_of(span, struct rcb_card_t, span);
	int num_chans = rcb_card->num_chans;
	struct rcb_chan_echo_coefs coefs;

	switch (cmd) {
	case DAHDI_ONHOOKTRANSFER:
		return -EINVAL;
		break;
	case DAHDI_SETPOLARITY:
		return -EINVAL;
		break;
	case RCB_CHAN_SET_CBPARAMS:
		if (debug)
			printk("rcbfx %d: Setting cbfx parameters: \n", rcb_card->pos + 1);
		if (*(volatile __u32 *) (rcb_card->memaddr + RCB_TXSIGSTAT) & 0x09000) {
			printk("rcbfx %d: Board not ready for params -- not setting\n",
				   rcb_card->pos + 1);
			return -EFAULT;
			break;
		}
		if (copy_from_user
			(&rcb_card_params, (struct rcb_card_params_t *) data,
			 sizeof(rcb_card_params)))
			return -EFAULT;

		printk("rcbfx %d: Recieved cbfx parameters: \n", rcb_card->pos + 1);

		for (regnum = 0; regnum < P_TBL_CNT; regnum++) {
			if (debug)
				printk("rcbfx %d: IO Address: %x, Regnum: %x, Data: %x \n",
					   rcb_card->pos + 1, PARAM_TBL + regnum, regnum,
					   (__u8) (rcb_card_params.settings[regnum]));
			*(volatile __u8 *) (rcb_card->memaddr + PARAM_TBL + regnum) =
				(__u8) (rcb_card_params.settings[regnum]);
		}
		/* notify of parameter update */
		*(volatile __u32 *) (rcb_card->memaddr + RCB_TXSIGSTAT) = 0x08000;
		break;
	case RCB_CHAN_GET_BDINFO:
		if (debug)
			printk("rcbfx %d: ioctl RCB_CHAN_GET_BDINFO sending %d\n", rcb_card->pos + 1,
				   num_chans);
		if (copy_to_user((int *) data, &num_chans, sizeof(num_chans)))
			return -EFAULT;
		break;

	case RCB_CHAN_SET_ECHOTUNE:
		if (debug) {
			printk("rcbfx %d: ioctl RCB_CHAN_SET_ECHOTUNE sending\n", rcb_card->pos + 1);
			printk("chan %x, ac %x, 1 %x, 2 %x, 3 %x, 4 %x, 5 %x, 6 %x, 7 %x, 8 %x,\n",
				   chan->chanpos, coefs.acim, coefs.coef1, coefs.coef2, coefs.coef3,
				   coefs.coef4, coefs.coef5, coefs.coef6, coefs.coef7, coefs.coef8);
		}
		if (*(volatile __u32 *) (rcb_card->memaddr + RCB_TXSIGSTAT) & 0x05000) {
			printk("rcbfx %d: Board not ready for params -- not setting %x\n",
				   rcb_card->pos + 1,
				   *(volatile __u32 *) (rcb_card->memaddr + RCB_TXSIGSTAT));
			return -EFAULT;
			break;
		}

		if (copy_from_user(&coefs, (struct rcb_chan_echo_coefs *) data, sizeof(coefs)))
			return -EFAULT;

		if (rcb_card->modtype[chan->chanpos - 1] == MOD_TYPE_FXO) {
			*(volatile __u8 *) (rcb_card->memaddr + PARAM_TBL + RCB_CHAN_REG) =
				chan->chanpos;
			*(volatile __u8 *) (rcb_card->memaddr + PARAM_TBL + RCB_CHAN_REG + 1) =
				coefs.acim;
			*(volatile __u8 *) (rcb_card->memaddr + PARAM_TBL + RCB_CHAN_REG + 2) =
				coefs.coef1;
			*(volatile __u8 *) (rcb_card->memaddr + PARAM_TBL + RCB_CHAN_REG + 3) =
				coefs.coef2;
			*(volatile __u8 *) (rcb_card->memaddr + PARAM_TBL + RCB_CHAN_REG + 4) =
				coefs.coef3;
			*(volatile __u8 *) (rcb_card->memaddr + PARAM_TBL + RCB_CHAN_REG + 5) =
				coefs.coef4;
			*(volatile __u8 *) (rcb_card->memaddr + PARAM_TBL + RCB_CHAN_REG + 6) =
				coefs.coef5;
			*(volatile __u8 *) (rcb_card->memaddr + PARAM_TBL + RCB_CHAN_REG + 7) =
				coefs.coef6;
			*(volatile __u8 *) (rcb_card->memaddr + PARAM_TBL + RCB_CHAN_REG + 8) =
				coefs.coef7;
			*(volatile __u8 *) (rcb_card->memaddr + PARAM_TBL + RCB_CHAN_REG + 9) =
				coefs.coef8;
		} else {
			return -EINVAL;
		}

		/* notify of parameter update */
		*(volatile __u32 *) (rcb_card->memaddr + RCB_TXSIGSTAT) = 0x04000;
		msleep(100);
		break;

	default:
		return -ENOTTY;
		break;
	}
	return 0;
}

static int rcb_card_update_fw(struct rcb_card_t *rcb_card,
							  const struct firmware *firmware)
{
	long end_jiffies;
	unsigned int block_count = 0;
	unsigned int total_blocks, frac_block, char_count, zeros = 0;
	unsigned char block_sum;

	total_blocks = (firmware->size - 2) >> 8;	/* 256 char blocks */
	frac_block = (firmware->size - 2) & 0xff;

	printk("rcbfx %d: New Firmware is %xh bytes - loading into 800h to %xh\n",
		   rcb_card->pos + 1, (unsigned int) (firmware->size - 2),
		   (unsigned int) (firmware->size - 2) + 0x800);
	printk("Firmware Size is %d blocks plus %d more bytes\n", total_blocks, frac_block);
	if ((firmware->size - 2) > 0x7600) {
		printk("rcbfx %d: Firmware file to large %x GT %x\n", rcb_card->pos + 1,
			   (unsigned int) (firmware->size - 2), 0x7600);
		return -2;
	}
	if ((firmware->size - 2) < 0x5000) {
		printk("rcbfx %d: Firmware file too small %x\n", rcb_card->pos + 1,
			   (unsigned int) (firmware->size - 2));
		return -2;
	}
	for (char_count = 2; char_count < firmware->size; char_count++) {
		if (firmware->data[char_count] == 0)
			zeros++;
		else
			break;
	}
	if (zeros > 20) {
		printk("rcbfx %d: %d too many leading zeros\n", rcb_card->pos + 1, zeros);
		return -2;
	}
	/* reset and set boot code to bootloader */
	*(volatile __u8 *) (rcb_card->memaddr + FW_BOOT) = FW_BOOT_LOAD | FW_BOOT_RST;
	end_jiffies = jiffies + 50;
	while (end_jiffies > jiffies);
	*(volatile __u8 *) (rcb_card->memaddr + FW_BOOT) = FW_BOOT_LOAD;	/* release reset */
	end_jiffies = jiffies + 50;
	while (end_jiffies > jiffies);	/* wait for reset */
	if (debug)
		printk("rcbfx %d: Starting to send firmware\n", rcb_card->pos + 1);
	for (block_count = 1; block_count <= total_blocks; block_count++) {
		schedule();
		/* send block */
		block_sum = 0;
		for (char_count = 0; char_count < 0x100; char_count++) {
			*(volatile __u8 *) (rcb_card->memaddr + PARAM_TBL + char_count) =
				firmware->data[char_count + 2 + ((block_count - 1) << 8)];
			block_sum =
				(unsigned char) (block_sum + firmware->data[char_count + 2 +
															((block_count - 1) << 8)]);
		}
		/* update block counter */
		*(volatile __u8 *) (rcb_card->memaddr + FW_COMOUT) = block_count;
		*(volatile __u8 *) (rcb_card->memaddr + FW_SUM) = block_sum;

		end_jiffies = jiffies + 2000;
		/* wait for block to take */
		while ((*(volatile __u8 *) (rcb_card->memaddr + FW_COMIN) != block_count) &&
			   (end_jiffies > jiffies));
		if (*(volatile __u8 *) (rcb_card->memaddr + FW_COMIN) != block_count) {
			printk("rcbfx %d: Time out!!!! %d blocks transfered %d blocks taken\n",
				   rcb_card->pos + 1, block_count,
				   *(volatile __u8 *) (rcb_card->memaddr + FW_COMIN));
			return -1;
		}
		if (debug)
			printk("rcbfx %d: Acked block %x of %x  ", rcb_card->pos + 1, block_count,
				   total_blocks);
	}
	/* clear block for checksum purposes */
	for (char_count = 0; char_count < 0x100; char_count++)
		*(volatile __u8 *) (rcb_card->memaddr + PARAM_TBL + char_count) = 0;
	/* send the partial block */
	block_sum = 0;
	for (char_count = 0; char_count < frac_block; char_count++) {
		*(volatile __u8 *) (rcb_card->memaddr + PARAM_TBL + char_count) =
			firmware->data[char_count + 2 + ((block_count - 1) << 8)];
		block_sum = (unsigned char) (block_sum + firmware->data[char_count + 2 +
																((block_count -
																  1) << 8)]);
	}
	/* update block counter */
	*(volatile __u8 *) (rcb_card->memaddr + FW_COMOUT) = block_count;
	*(volatile __u8 *) (rcb_card->memaddr + FW_SUM) = block_sum;
	if (debug)
		printk("rcbfx %d: Sent partial block %x\n", rcb_card->pos + 1, block_count);

	end_jiffies = jiffies + 2000;
	/* wait for block to take */
	while ((*(volatile __u8 *) (rcb_card->memaddr + FW_COMIN) != block_count) &&
		   (end_jiffies > jiffies));
	if (*(volatile __u8 *) (rcb_card->memaddr + FW_COMIN) != block_count) {
		printk("rcbfx %d: Time out!!!! %d blocks transfered %d blocks taken\n",
			   rcb_card->pos + 1, block_count,
			   *(volatile __u8 *) (rcb_card->memaddr + FW_COMIN));
		return -1;
	}
	if (debug)
		printk("rcbfx %d: Acked partial block %x\n", rcb_card->pos + 1, block_count);
	/* say done */
	*(volatile __u8 *) (rcb_card->memaddr + FW_BOOT) = 0;
	*(volatile __u8 *) (rcb_card->memaddr + FW_COMOUT) = 0xff;
	/* wait for it */
	end_jiffies = jiffies + 100;
	while ((*(volatile __u8 *) (rcb_card->memaddr + FW_COMIN) != 0xff) ||
		   (end_jiffies > jiffies));

	if (*(volatile __u8 *) (rcb_card->memaddr + FW_COMIN) != 0xff) {
		printk("rcbfx %d: All blocks transfered no ACK\n", rcb_card->pos + 1);
		return -1;
	} else
		printk("rcbfx %d: All blocks transfered and acked\n", rcb_card->pos + 1);

	*(volatile __u8 *) (rcb_card->memaddr + FW_DATA) = 0x00;
	*(volatile __u8 *) (rcb_card->memaddr + FW_COMOUT) = 0x00;

	return 0;
}

static int rcb_dahdi_chan_open(struct dahdi_chan *chan)
{
	struct dahdi_span *span = chan->span;
	struct rcb_card_t *rcb_card = container_of(span, struct rcb_card_t, span);
	if (rcb_card->num_chans < (chan->chanpos - 1))	/* make sure the chan exists */
		return -ENODEV;
	if (rcb_card->dead)
		return -ENODEV;
	rcb_card->usecount++;
	if (debug)
		printk("rcbfx %d: Use Count %x\n", rcb_card->pos + 1, rcb_card->usecount);
	try_module_get(THIS_MODULE);
	return 0;
}

static void rcb_card_restart_dma(struct rcb_card_t *rcb_card)
{
	if (debug)
		printk("rcbfx %d: Restarting DMA\n", rcb_card->pos + 1);
	*(volatile __u8 *) (rcb_card->memaddr + RCB_CONTROL) &= ~BIT_DMAGO;
	*(volatile __u8 *) (rcb_card->memaddr + RCB_CONTROL) |= BIT_DMAGO;
	return;
}

static int rcb_dahdi_span_watchdog(struct dahdi_span *span, int event)
{
	struct rcb_card_t *rcb_card = container_of(span, struct rcb_card_t, span);
	rcb_card_restart_dma(rcb_card);
	return 0;
}

static void rcb_card_release(struct rcb_card_t *rcb_card)
{
	*(volatile __u8 *) (rcb_card->memaddr + RCB_STATOUT) &= ~0x00;
	if (debug)
		printk("rcbfx %d: Statout = %x\n", rcb_card->pos + 1,
			   *(volatile __u8 *) (rcb_card->memaddr + RCB_STATOUT));
	dahdi_unregister(&rcb_card->span);
	if (rcb_card->freeregion)
		release_region(rcb_card->baseaddr, rcb_card->memlen);
	printk("rcbfx %d: Released a Rhino\n", rcb_card->pos + 1);
	kfree(rcb_card);
}

static int rcb_dahdi_chan_close(struct dahdi_chan *chan)
{
	struct dahdi_span *span = chan->span;
	struct rcb_card_t *rcb_card = container_of(span, struct rcb_card_t, span);
	rcb_card->usecount--;
	if (debug)
		printk("rcbfx %d: Use Count %d\n", rcb_card->pos + 1, rcb_card->usecount);
	module_put(THIS_MODULE);
	/* If we're dead, release us now */
	if (!rcb_card->usecount && rcb_card->dead)
		rcb_card_release(rcb_card);
	return 0;
}

static int rcb_card_initialize(struct rcb_card_t *rcb_card)
{
static struct dahdi_span_ops ops = {
	.rbsbits = rcb_dahdi_chan_rbsbits,
	.open = rcb_dahdi_chan_open,
	.close = rcb_dahdi_chan_close,
	.ioctl = rcb_dahdi_chan_ioctl,
	.watchdog = rcb_dahdi_span_watchdog,
	.echocan_create = rcbfx_echocan_create,
	.owner = THIS_MODULE,
	};

	int chan_num;
	/* dahdi stuff */

	sprintf(rcb_card->span.name, "%s/%d", rcb_card->variety, rcb_card->pos + 1);
	sprintf(rcb_card->span.desc, "%s/%d", rcb_card->variety, rcb_card->pos + 1);
	rcb_card->span.deflaw = DAHDI_LAW_MULAW;

	for (chan_num = 0; chan_num < rcb_card->num_chans; chan_num++) {

		rcb_card->chans[chan_num]->writechunk =
			(u_char *) (rcb_card->writechunk + (chan_num * DAHDI_CHUNKSIZE));
		rcb_card->chans[chan_num]->readchunk =
			(u_char *) (rcb_card->readchunk + (chan_num * DAHDI_CHUNKSIZE));
		if (show_pointers)
			printk("rcbfx %d: Chan %d writechunk %lx readchunk %lx\n",
				   rcb_card->pos + 1, chan_num,
				   (long unsigned int) rcb_card->chans[chan_num]->writechunk,
				   (long unsigned int) rcb_card->chans[chan_num]->readchunk);

		if (rcb_card->chanflag & (1 << chan_num)) {
			if (rcb_card->modtype[chan_num] == MOD_TYPE_FXO) {
				sprintf(rcb_card->chans[chan_num]->name, "FXO/%d/%d",
						rcb_card->pos + 1, chan_num);
				rcb_card->chans[chan_num]->sigcap =
					DAHDI_SIG_FXSKS | DAHDI_SIG_FXSLS | DAHDI_SIG_SF | DAHDI_SIG_CLEAR;
			} else {
				sprintf(rcb_card->chans[chan_num]->name, "FXS/%d/%d",
						rcb_card->pos + 1, chan_num);
				rcb_card->chans[chan_num]->sigcap =
					DAHDI_SIG_FXOKS | DAHDI_SIG_FXOLS | DAHDI_SIG_FXOGS | DAHDI_SIG_SF |
					DAHDI_SIG_EM | DAHDI_SIG_CLEAR;
			}
		} else {
			sprintf(rcb_card->chans[chan_num]->name, "---/%d/%d",
					rcb_card->pos + 1, chan_num);
			rcb_card->chans[chan_num]->sigcap =
				DAHDI_SIG_FXOKS | DAHDI_SIG_FXOLS | DAHDI_SIG_FXOGS | DAHDI_SIG_SF |
				DAHDI_SIG_EM | DAHDI_SIG_CLEAR | DAHDI_SIG_FXSKS | DAHDI_SIG_FXSLS;
		}

		rcb_card->chans[chan_num]->chanpos = chan_num + 1;
		rcb_card->chans[chan_num]->pvt = rcb_card;
	}

	rcb_card->span.manufacturer = "Rhino Equipment Corp.";
	rcb_card->span.channels = rcb_card->num_chans;
	rcb_card->span.chans = rcb_card->chans;
	rcb_card->span.flags = DAHDI_FLAG_RBS;
	init_waitqueue_head(&rcb_card->span.maintq);

	rcb_card->span.ops = &ops;

	if (dahdi_register(&rcb_card->span, 0)) {
		printk("rcbfx %d: Unable to register span with Parent\n", rcb_card->pos + 1);
		return -1;
	}

	rcb_card->chans_configed = 1;
	return 0;
}

static int rcb_card_hardware_init(struct rcb_card_t *rcb_card)
{
	/* Hardware stuff */
	__u16 bd_pres;
	__u32 is_fxo;
	long end_jiffies;
	int chan_num, time_out, res, tries = 0;
	int fwv_register, fwv_file;
	int done = 0;
	static const struct firmware *firmware_rcb;

	/* reset DSP */
	*(volatile __u8 *) (rcb_card->memaddr + FW_BOOT) = DSP_RST;
	end_jiffies = jiffies + 30000;
	time_out = 0;

	/* ping!! */
	*(volatile __u32 *) (rcb_card->memaddr + RCB_TXSIGSTAT) = 0x02000;
	/* check sign of life and move on */
	if (*(volatile __u32 *) (rcb_card->memaddr + RCB_TXSIGSTAT) & 0x02000) {
		printk("rcbfx %d: Waiting for response from card ......... \n",
			   rcb_card->pos + 1);
		while ((*(volatile __u32 *) (rcb_card->memaddr + RCB_TXSIGSTAT) & 0x02000) &&
			   (time_out == 0)) {
			if (jiffies > end_jiffies)
				time_out = 1;
		}
	}

	/* release DSP reset */
	*(volatile __u8 *) (rcb_card->memaddr + FW_BOOT) = 0x00;

	fwv_register = *(volatile __u16 *) (rcb_card->memaddr + FW_VER);
	printk("rcbfx %d: Firmware Version %x.%x\n", rcb_card->pos + 1,
		   (fwv_register & 0xff00) >> 8, (fwv_register & 0xff));

	if ((request_firmware(&firmware_rcb, rcbfx_firmware, &rcb_card->dev->dev) != 0) ||
		!firmware_rcb) {
		printk("rcbfx %d: firmware %s not available from userspace\n", rcb_card->pos + 1,
			   rcbfx_firmware);
	} else {
		fwv_file = ((firmware_rcb->data[0] << 8) | (firmware_rcb->data[1]));
		printk("rcbfx %d: Firmware File Version is %x.%x\n", rcb_card->pos + 1,
			   (fwv_file & 0xff00) >> 8, fwv_file & 0xff);
		if ((fwv_file > fwv_register) || (force_fw)) {
			if (force_fw)
				printk("rcbfx %d: Firmware Upgrade beeing forced\n", rcb_card->pos + 1);
			printk(KERN_ALERT
				   "rcbfx %d: Firmware Uprgrade In Progress -- Do Not Interrupt!!\n",
				   rcb_card->pos + 1);
			while (!done) {
				res = rcb_card_update_fw(rcb_card, firmware_rcb);
				tries++;
				if ((res == 0) || (res == -2))
					done = 1;
				else if ((res == -1) && (tries > 3))
					done = 1;
			}
		}

		release_firmware(firmware_rcb);
		*(volatile __u8 *) (rcb_card->memaddr + FW_DATA) = 0x00;
		*(volatile __u8 *) (rcb_card->memaddr + FW_COMOUT) = 0x00;
	}

	*(volatile __u8 *) (rcb_card->memaddr + RCB_BATTTIME) = battime;

	printk("rcbfx %d: Hardware version %d\n", rcb_card->pos + 1,
		   *(volatile __u16 *) (rcb_card->memaddr + RCB_VERSION));
	if (*(volatile __u16 *) (rcb_card->memaddr + RCB_VERSION) < rcb_card->hw_ver_min) {
		if (debug)
			printk("rcbfx %d: YOU ARE NOT USING THE LATEST HARDWARE VERSION\n",
				   rcb_card->pos + 1);
	}
	if (done) {					/* updated */
		end_jiffies = jiffies + 30000;
		time_out = 0;
		/* ping!! */
		*(volatile __u32 *) (rcb_card->memaddr + RCB_TXSIGSTAT) = 0x02000;

		/* check sign of life and move on */
		if (*(volatile __u32 *) (rcb_card->memaddr + RCB_TXSIGSTAT) & 0x02000) {
			printk("Waiting for response from card ......... \n");
			while ((*(volatile __u32 *) (rcb_card->memaddr + RCB_TXSIGSTAT) &
					(__u32) (0x02000)) && (time_out == 0)) {
				schedule();
				if (jiffies > end_jiffies)
					time_out = 1;
			}
		}
	}

	if ((time_out == 0) && (debug))
		printk("rcbfx %d: Got response from card\n", rcb_card->pos + 1);
	if (time_out == 1) {
		printk("rcbfx %d: Not responding !!!!\n", rcb_card->pos + 1);
	}

	bd_pres = *(volatile __u16 *) (rcb_card->memaddr + RCB_CHANPRES);
	is_fxo = *(volatile __u32 *) (rcb_card->memaddr + RCB_CHANTYPE);
	if (debug) {
		printk("rcbfx %d: Channels Present: %x, Channel Type: %x\n", rcb_card->pos + 1,
			   bd_pres, is_fxo);
		printk("rcbfx %d: num_chans: %d\n", rcb_card->pos + 1, rcb_card->num_chans);
	}

	rcb_card->fxs_chanmap = 0;
	rcb_card->fxo_chanmap = 0;

	for (chan_num = 0; chan_num < rcb_card->num_chans; chan_num++) {
		if ((1 << (chan_num >> 1)) & bd_pres) {
			rcb_card->chanflag |= (1 << chan_num);

			if ((1 << chan_num) & is_fxo) {
				rcb_card->modtype[chan_num] = MOD_TYPE_FXO;
				rcb_card->fxo_chanmap |= (1 << chan_num);
			} else {
				rcb_card->modtype[chan_num] = MOD_TYPE_FXS;
				rcb_card->fxs_chanmap |= (1 << chan_num);
			}
		}
	}

#ifdef USE_G168_DSP
	/* Set the global variables here */
	if ((ec_fxo_alg > 0) || (ec_fxo_alg == 0))	/* forced value */
		fxo_alg_chanmap = ec_fxo_alg;
	else
		fxo_alg_chanmap = rcb_card->fxo_chanmap;	/* On in B diraction */

	if ((ec_fxs_alg > 0) || (ec_fxs_alg == 0))	/* forced value */
		fxs_alg_chanmap = ec_fxs_alg;
	else
		fxs_alg_chanmap = rcb_card->fxs_chanmap;	/* On in A diraction */

	if ((use_ec_fxo > 0) || (use_ec_fxo == 0))	/* forced value */
		use_fxo_chanmap = use_ec_fxo;
	else
		use_fxo_chanmap = rcb_card->fxo_chanmap;	/* Off for fxs */

	if ((use_ec_fxs > 0) || (use_ec_fxs == 0))	/* forced value */
		use_fxs_chanmap = use_ec_fxs;
	else
		use_fxs_chanmap = rcb_card->fxs_chanmap;	/* off for fxo */
#endif

	/* Setup DMA Addresses */
	*(volatile __u32 *) (rcb_card->memaddr + RCB_TXBUFSTART) = (__u32) rcb_card->writedma;
	*(volatile __u32 *) (rcb_card->memaddr + RCB_RXBUFSTART) = (__u32) rcb_card->readdma;
	*(volatile __u16 *) (rcb_card->memaddr + RCB_BUFLEN) =
		((__u16) (DAHDI_CHUNKSIZE * rcb_card->num_chans * 2) >> 2);

	return 0;
}

static void rcb_card_start_dma(struct rcb_card_t *rcb_card)
{
	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(1);
	*(volatile __u8 *) (rcb_card->memaddr + RCB_CONTROL) |= BIT_DMAGO;
}


static void rcb_card_stop_dma(struct rcb_card_t *rcb_card)
{
	*(volatile __u8 *) (rcb_card->memaddr + RCB_CONTROL) &= ~BIT_DMAGO;
	*(volatile __u8 *) (rcb_card->memaddr + RCB_STATOUT) &= ~0x00;
	if (debug)
		printk("rcbfx %d: Statout = %x\n", rcb_card->pos + 1,
			   *(volatile __u8 *) (rcb_card->memaddr + RCB_STATOUT));
	*(volatile __u8 *) (rcb_card->memaddr + FW_DATA) = 0x00;
	*(volatile __u8 *) (rcb_card->memaddr + FW_COMOUT) = 0x00;
}

static void rcb_card_cleaner(struct rcb_card_t *rcb_card, int flags)
{
	if (rcb_card) {

		if (flags & STOP_DMA) {
			*(volatile __u8 *) (rcb_card->memaddr + RCB_CONTROL) &= ~BIT_DMAGO;
		}
		if (flags & FREE_DMA) {
			if (rcb_card->writechunk) {
				pci_free_consistent(rcb_card->dev,
									DAHDI_MAX_CHUNKSIZE * rcb_card->num_chans * 2 * 2,
									(void *) rcb_card->writechunk, rcb_card->writedma);
				rcb_card->writechunk = NULL;
			}
		}
		if (flags & IOUNMAP) {
			if (rcb_card->freeregion) {
				rcb_card->freeregion = -1;
				release_region(rcb_card->baseaddr, rcb_card->memlen);
			}
			iounmap(rcb_card->memaddr);
		}
		if (flags & ZUNREG) {
			if (!rcb_card->usecount)
				dahdi_unregister(&rcb_card->span);
			else
				rcb_card->dead = 1;
		}
		if (flags & FREE_INT)
			free_irq(rcb_card->dev->irq, rcb_card);
		if (flags & RH_KFREE)
			kfree(rcb_card);
		if (flags & PCI_FREE)
			pci_set_drvdata(rcb_card->dev, NULL);
	}
}

#ifdef USE_G168_DSP

static GpakPortConfig_t Gpak_24_chan_port_config = {

	/* GpakSlotCfg_t         SlotsSelect1          port 1 Slot selection */
	SlotCfg2Groups,
	/* unsigned short int    FirstBlockNum1        port 1 first group Block Number */
	0x0000,
	/* unsigned short int    FirstSlotMask1        port 1 first group Slot Mask */
	0xffff,
	/* unsigned short int    SecBlockNum1          port 1 second group Block Number */
	0x0001,
	/* unsigned short int    SecSlotMask1          port 1 second group Slot Mask */
	0x00ff,
	/* GpakSerWordSize_t     SerialWordSize1       port 1 serial word size */
	SerWordSize8,
	/* GpakCompandModes      CompandingMode1       port 1 companding mode */
	cmpNone,
	/* GpakSerFrameSyncPol_t TxFrameSyncPolarity1  port 1 Tx Frame Sync Polarity */
	FrameSyncActHigh,
	/* GpakSerFrameSyncPol_t RxFrameSyncPolarity1  port 1 Rx Frame Sync Polarity */
	FrameSyncActHigh,
	/* GpakSerClockPol_t     TxClockPolarity1      port 1 Tx Clock Polarity */
	SerClockActHigh,
	/* GpakSerClockPol_t     RxClockPolarity1      port 1 Rx Clock Polarity */
	SerClockActLow,
	/* GpakSerDataDelay_t    TxDataDelay1          port 1 Tx data delay */
	DataDelay0,
	/* GpakSerDataDelay_t    RxDataDelay1          port 1 Rx data delay */
	DataDelay0,
	/* GpakActivation        DxDelay1              port 1 DX Delay */
	Disabled,
	/* unsigned short int    ThirdSlotMask1        port 1 3rd group Slot Mask */
	0x0000,
	/* unsigned short int    FouthSlotMask1        port 1 4th group Slot Mask */
	0x0000,
	/* unsigned short int    FifthSlotMask1        port 1 5th group Slot Mask */
	0x0000,
	/* unsigned short int    SixthSlotMask1        port 1 6th group Slot Mask */
	0x0000,
	/* unsigned short int    SevenSlotMask1        port 1 7th group Slot Mask */
	0x0000,
	/* unsigned short int    EightSlotMask1        port 1 8th group Slot Mask */
	0x0000,
	/* GpakSlotCfg_t         SlotsSelect2          port 2 Slot selection */
	SlotCfg2Groups,
	/* unsigned short int    FirstBlockNum2        port 2 first group Block Number */
	0x0000,
	/* unsigned short int    FirstSlotMask2        port 2 first group Slot Mask */
	0xffff,
	/* unsigned short int    SecBlockNum2          port 2 second group Block Number */
	0x0001,
	/* unsigned short int    SecSlotMask2          port 2 second group Slot Mask */
	0x00ff,
	/* GpakSerWordSize_t     SerialWordSize2       port 2 serial word size */
	SerWordSize8,
	/* GpakCompandModes      CompandingMode2       port 2 companding mode */
	cmpNone,
	/* GpakSerFrameSyncPol_t TxFrameSyncPolarity2  port 2 Tx Frame Sync Polarity */
	FrameSyncActHigh,
	/* GpakSerFrameSyncPol_t RxFrameSyncPolarity2  port 2 Rx Frame Sync Polarity */
	FrameSyncActHigh,
	/* GpakSerClockPol_t     TxClockPolarity2      port 2 Tx Clock Polarity */
	SerClockActHigh,
	/* GpakSerClockPol_t     RxClockPolarity2      port 2 Rx Clock Polarity */
	SerClockActLow,
	/* GpakSerDataDelay_t    TxDataDelay2          port 2 Tx data delay */
	DataDelay0,
	/* GpakSerDataDelay_t    RxDataDelay2          port 2 Rx data delay */
	DataDelay0,
	/* GpakActivation        DxDelay2              port 2 DX Delay */
	Disabled,
	/* unsigned short int    ThirdSlotMask2        port 2 3rd group Slot Mask */
	0x0000,
	/* unsigned short int    FouthSlotMask2        port 2 4th group Slot Mask */
	0x0000,
	/* unsigned short int    FifthSlotMask2        port 2 5th group Slot Mask */
	0x0000,
	/* unsigned short int    SixthSlotMask2        port 2 6th group Slot Mask */
	0x0000,
	/* unsigned short int    SevenSlotMask2        port 2 7th group Slot Mask */
	0x0000,
	/* unsigned short int    EightSlotMask2        port 2 8th group Slot Mask */
	0x0000,
	/* GpakSlotCfg_t         SlotsSelect3          port 3 Slot selection */
	SlotCfgNone,
	/* unsigned short int    FirstBlockNum3        port 3 first group Block Number */
	0x0000,
	/* unsigned short int    FirstSlotMask3        port 3 first group Slot Mask */
	0x0000,
	/* unsigned short int    SecBlockNum3          port 3 second group Block Number */
	0x0000,
	/* unsigned short int    SecSlotMask3          port 3 second group Slot Mask */
	0x0000,
	/* GpakSerWordSize_t     SerialWordSize3       port 3 serial word size */
	SerWordSize8,
	/* GpakCompandModes      CompandingMode3       port 3 companding mode */
	cmpNone,
	/* GpakSerFrameSyncPol_t TxFrameSyncPolarity3  port 3 Tx Frame Sync Polarity */
	FrameSyncActHigh,
	/* GpakSerFrameSyncPol_t RxFrameSyncPolarity3  port 3 Rx Frame Sync Polarity */
	FrameSyncActHigh,
	/* GpakSerClockPol_t     TxClockPolarity3      port 3 Tx Clock Polarity */
	SerClockActHigh,
	/* GpakSerClockPol_t     RxClockPolarity3      port 3 Rx Clock Polarity */
	SerClockActHigh,
	/* GpakSerDataDelay_t    TxDataDelay3          port 3 Tx data delay */
	DataDelay0,
	/* GpakSerDataDelay_t    RxDataDelay3          port 3 Rx data delay */
	DataDelay0,
	/* GpakActivation        DxDelay3              port 3 DX Delay */
	Disabled,
	/* unsigned short int    ThirdSlotMask3        port 3 3rd group Slot Mask */
	0x0000,
	/* unsigned short int    FouthSlotMask3        port 3 4th group Slot Mask */
	0x0000,
	/* unsigned short int    FifthSlotMask3        port 3 5th group Slot Mask */
	0x0000,
	/* unsigned short int    SixthSlotMask3        port 3 6th group Slot Mask */
	0x0000,
	/* unsigned short int    SevenSlotMask3        port 3 7th group Slot Mask */
	0x0000,
	/* unsigned short int    EightSlotMask3        port 3 8th group Slot Mask */
	0x0000,
};

static GpakChannelConfig_t Gpak_chan_config = {

	/* GpakSerialPort_t    PCM Input Serial Port A Id */
	SerialPort1,
	/* unsigned short int  PCM Input Time Slot */
	0,
	/* GpakSerialPort_t    PCM Output Serial Port A Id */
	SerialPortNull,
	/* unsigned short int  PCM Output Time Slot */
	0,
	/* GpakSerialPort_t    PCM Input Serial Port B Id */
	SerialPort2,
	/* unsigned short int  PCM Input Time Slot */
	0,
	/* GpakSerialPort_t    PCM Output Serial Port B Id */
	SerialPort1,
	/* unsigned short int  PCM Output Time Slot */
	0,
	/* GpakToneTypes       ToneTypesA A side Tone Detect Types */
	Null_tone,
	/* GpakToneTypes       ToneTypesB B side Tone Detect Types */
	Null_tone,
	/* GpakActivation      Echo Cancel A Enabled */
	Disabled,
	/* GpakActivation      Echo Cancel B Enabled */
	Disabled,

	{
	 /* short int  Echo Can Num Taps (tail length) 64 = 512 32 = 256 */
	 1024,
	 /* short int  Echo Can NLP Type */
	 3,
	 /* short int  Echo Can Adapt Enable flag */
	 1,
	 /* short int  Echo Can G165 Detect Enable flag */
	 1,
	 /* short int  Echo Can Double Talk threshold */
	 4,
	 /* short int  Echo Can NLP threshold */
	 21,
	 /* short int  Dynamic NLP control, NLP limit when EC about to converged */
	 18,
	 /* short int  Dynamic NLP control, NLP limit when EC not converged yet */
	 12,
	 /* short int  suppression level for NLP_SUPP mode */
	 0,
	 /* short int  Echo Can CNG Noise threshold */
	 50,
	 /* short int  Echo Can Max Adapts per frame */
	 50,
	 /* short int  Echo Can Cross Correlation limit */
	 20,
	 /* short int  Echo Can Num FIR Segments */
	 3,
	 /* short int  Echo Can FIR Segment Length */
	 64,
	 },

	{
	 /* short int  Echo Can Num Taps (tail length) */
	 1024,
	 /* short int  Echo Can NLP Type */
	 3,
	 /* short int  Echo Can Adapt Enable flag */
	 1,
	 /* short int  Echo Can G165 Detect Enable flag */
	 1,
	 /* short int  Echo Can Double Talk threshold */
	 4,
	 /* short int  Echo Can NLP threshold */
	 21,
	 /* short int  Dynamic NLP control, NLP limit when EC about to converged */
	 18,
	 /* short int  Dynamic NLP control, NLP limit when EC not converged yet */
	 12,
	 /* short int  suppression level for NLP_SUPP mode */
	 0,
	 /* short int  Echo Can CNG Noise threshold */
	 50,
	 /* short int  Echo Can Max Adapts per frame */
	 50,
	 /* short int  Echo Can Cross Correlation limit */
	 20,
	 /* short int  Echo Can Num FIR Segments */
	 3,
	 /* short int  Echo Can FIR Segment Length */
	 64,
	 },

	/* GpakCompandModes    software companding */
	cmpNone,
	/* GpakRate_t          Gpak Frame Rate */
	rate2ms,

};

static void rcb_card_dsp_show_portconfig(GpakPortConfig_t PortConfig)
{
	if (debug & DEBUG_DSP) {
		printk("%x = %s\n", PortConfig.SlotsSelect1, "SlotsSelect1");
		printk("%x = %s\n", PortConfig.FirstBlockNum1, "FirstBlockNum1");
		printk("%x = %s\n", PortConfig.FirstSlotMask1, "FirstSlotMask1");
		printk("%x = %s\n", PortConfig.SecBlockNum1, "SecBlockNum1");
		printk("%x = %s\n", PortConfig.SecSlotMask1, "SecSlotMask1");
		printk("%x = %s\n", PortConfig.SerialWordSize1, "SerialWordSize1");
		printk("%x = %s\n", PortConfig.CompandingMode1, "CompandingMode1");
		printk("%x = %s\n", PortConfig.TxFrameSyncPolarity1, "TxFrameSyncPolarity1");
		printk("%x = %s\n", PortConfig.RxFrameSyncPolarity1, "RxFrameSyncPolarity1");
		printk("%x = %s\n", PortConfig.TxClockPolarity1, "TxClockPolarity1");
		printk("%x = %s\n", PortConfig.RxClockPolarity1, "RxClockPolarity1");
		printk("%x = %s\n", PortConfig.TxDataDelay1, "TxDataDelay1");
		printk("%x = %s\n", PortConfig.RxDataDelay1, "RxDataDelay1");
		printk("%x = %s\n", PortConfig.DxDelay1, "DxDelay1");

		printk("%x = %s\n", PortConfig.ThirdSlotMask1, "ThirdSlotMask1");
		printk("%x = %s\n", PortConfig.FouthSlotMask1, "FouthSlotMask1");
		printk("%x = %s\n", PortConfig.FifthSlotMask1, "FifthSlotMask1");
		printk("%x = %s\n", PortConfig.SixthSlotMask1, "SixthSlotMask1");
		printk("%x = %s\n", PortConfig.SevenSlotMask1, "SevenSlotMask1");
		printk("%x = %s\n", PortConfig.EightSlotMask1, "EightSlotMask1");

		printk("%x = %s\n", PortConfig.SlotsSelect2, "SlotsSelect2");
		printk("%x = %s\n", PortConfig.FirstBlockNum2, "FirstBlockNum2");
		printk("%x = %s\n", PortConfig.FirstSlotMask2, "FirstSlotMask2");
		printk("%x = %s\n", PortConfig.SecBlockNum2, "SecBlockNum2");
		printk("%x = %s\n", PortConfig.SecSlotMask2, "SecSlotMask2");
		printk("%x = %s\n", PortConfig.SerialWordSize2, "SerialWordSize2");
		printk("%x = %s\n", PortConfig.CompandingMode2, "CompandingMode2");
		printk("%x = %s\n", PortConfig.TxFrameSyncPolarity2, "TxFrameSyncPolarity2");
		printk("%x = %s\n", PortConfig.RxFrameSyncPolarity2, "RxFrameSyncPolarity2");
		printk("%x = %s\n", PortConfig.TxClockPolarity2, "TxClockPolarity2");
		printk("%x = %s\n", PortConfig.RxClockPolarity2, "RxClockPolarity2");
		printk("%x = %s\n", PortConfig.TxDataDelay2, "TxDataDelay2");
		printk("%x = %s\n", PortConfig.RxDataDelay2, "RxDataDelay2");
		printk("%x = %s\n", PortConfig.DxDelay2, "DxDelay2");

		printk("%x = %s\n", PortConfig.ThirdSlotMask2, "ThirdSlotMask2");
		printk("%x = %s\n", PortConfig.FouthSlotMask2, "FouthSlotMask2");
		printk("%x = %s\n", PortConfig.FifthSlotMask2, "FifthSlotMask2");
		printk("%x = %s\n", PortConfig.SixthSlotMask2, "SixthSlotMask2");
		printk("%x = %s\n", PortConfig.SevenSlotMask2, "SevenSlotMask2");
		printk("%x = %s\n", PortConfig.EightSlotMask2, "EightSlotMask2");

		printk("%x = %s\n", PortConfig.SlotsSelect3, "SlotsSelect3");
		printk("%x = %s\n", PortConfig.FirstBlockNum3, "FirstBlockNum3");
		printk("%x = %s\n", PortConfig.FirstSlotMask3, "FirstSlotMask3");
		printk("%x = %s\n", PortConfig.SecBlockNum3, "SecBlockNum3");
		printk("%x = %s\n", PortConfig.SecSlotMask3, "SecSlotMask3");
		printk("%x = %s\n", PortConfig.SerialWordSize3, "SerialWordSize3");
		printk("%x = %s\n", PortConfig.CompandingMode3, "CompandingMode3");
		printk("%x = %s\n", PortConfig.TxFrameSyncPolarity3, "TxFrameSyncPolarity3");
		printk("%x = %s\n", PortConfig.RxFrameSyncPolarity3, "RxFrameSyncPolarity3");
		printk("%x = %s\n", PortConfig.TxClockPolarity3, "TxClockPolarity3");
		printk("%x = %s\n", PortConfig.RxClockPolarity3, "RxClockPolarity3");
		printk("%x = %s\n", PortConfig.TxDataDelay3, "TxDataDelay3");
		printk("%x = %s\n", PortConfig.RxDataDelay3, "RxDataDelay3");
		printk("%x = %s\n", PortConfig.DxDelay3, "DxDelay3");

		printk("%x = %s\n", PortConfig.ThirdSlotMask3, "ThirdSlotMask3");
		printk("%x = %s\n", PortConfig.FouthSlotMask3, "FouthSlotMask3");
		printk("%x = %s\n", PortConfig.FifthSlotMask3, "FifthSlotMask3");
		printk("%x = %s\n", PortConfig.SixthSlotMask3, "SixthSlotMask3");
		printk("%x = %s\n", PortConfig.SevenSlotMask3, "SevenSlotMask3");
		printk("%x = %s\n", PortConfig.EightSlotMask3, "EightSlotMask3");

	}
	return;
}

static void rcb_card_dsp_show_chanconfig(GpakChannelConfig_t ChanConfig)
{
	if (debug & DEBUG_DSP) {
		printk("%x = %s\n", ChanConfig.PcmInPortA, "PcmInPortA");
		printk("%x = %s\n", ChanConfig.PcmInSlotA, "PcmInSlotA");
		printk("%x = %s\n", ChanConfig.PcmOutPortA, "PcmOutPortA");
		printk("%x = %s\n", ChanConfig.PcmOutSlotA, "PcmOutSlotA");
		printk("%x = %s\n", ChanConfig.PcmInPortB, "PcmInPortB");
		printk("%x = %s\n", ChanConfig.PcmInSlotB, "PcmInSlotB");
		printk("%x = %s\n", ChanConfig.PcmOutPortB, "PcmOutPortB");
		printk("%x = %s\n", ChanConfig.PcmOutSlotB, "PcmOutSlotB");

		printk("%x = %s\n", ChanConfig.ToneTypesA, "ToneTypesA");
		printk("%x = %s\n", ChanConfig.ToneTypesB, "ToneTypesB");

		printk("%x = %s\n", ChanConfig.EcanEnableA, "EcanEnableA");
		printk("%x = %s\n", ChanConfig.EcanEnableB, "EcanEnableB");
		printk("%x = %s\n", ChanConfig.EcanParametersA.EcanTapLength,
			   "EcanParametersA.EcanTapLength");
		printk("%x = %s\n", ChanConfig.EcanParametersA.EcanNlpType,
			   "EcanParametersA.EcanNlpType");
		printk("%x = %s\n", ChanConfig.EcanParametersA.EcanAdaptEnable,
			   "EcanParametersA.EcanAdaptEnable");
		printk("%x = %s\n", ChanConfig.EcanParametersA.EcanG165DetEnable,
			   "EcanParametersA.EcanG165DetEnable");
		printk("%x = %s\n", ChanConfig.EcanParametersA.EcanDblTalkThresh,
			   "EcanParametersA.EcanDblTalkThresh");
		printk("%x = %s\n", ChanConfig.EcanParametersA.EcanNlpThreshold,
			   "EcanParametersA.EcanNlpThreshold");
		printk("%x = %s\n", ChanConfig.EcanParametersA.EcanNlpConv,
			   "EcanParametersA.EcanNlpConv");
		printk("%x = %s\n", ChanConfig.EcanParametersA.EcanNlpUnConv,
			   "EcanParametersA.EcanNlpUnConv");
		printk("%x = %s\n", ChanConfig.EcanParametersA.EcanNlpMaxSuppress,
			   "EcanParametersA.EcanNlpMaxSuppress");
		printk("%x = %s\n", ChanConfig.EcanParametersA.EcanCngThreshold,
			   "EcanParametersA.EcanCngThreshold");
		printk("%x = %s\n", ChanConfig.EcanParametersA.EcanAdaptLimit,
			   "EcanParametersA.EcanAdaptLimit");
		printk("%x = %s\n", ChanConfig.EcanParametersA.EcanCrossCorrLimit,
			   "EcanParametersA.EcanCrossCorrLimit");
		printk("%x = %s\n", ChanConfig.EcanParametersA.EcanNumFirSegments,
			   "EcanParametersA.EcanNumFirSegments");
		printk("%x = %s\n", ChanConfig.EcanParametersA.EcanFirSegmentLen,
			   "EcanParametersA.EcanFirSegmentLen");
		printk("%x = %s\n", ChanConfig.EcanParametersB.EcanTapLength,
			   "EcanParametersB.EcanTapLength");
		printk("%x = %s\n", ChanConfig.EcanParametersB.EcanNlpType,
			   "EcanParametersB.EcanNlpType");
		printk("%x = %s\n", ChanConfig.EcanParametersB.EcanAdaptEnable,
			   "EcanParametersB.EcanAdaptEnable");
		printk("%x = %s\n", ChanConfig.EcanParametersB.EcanG165DetEnable,
			   "EcanParametersB.EcanG165DetEnable");
		printk("%x = %s\n", ChanConfig.EcanParametersB.EcanDblTalkThresh,
			   "EcanParametersB.EcanDblTalkThresh");
		printk("%x = %s\n", ChanConfig.EcanParametersB.EcanNlpThreshold,
			   "EcanParametersB.EcanNlpThreshold");
		printk("%x = %s\n", ChanConfig.EcanParametersB.EcanNlpConv,
			   "EcanParametersB.EcanNlpConv");
		printk("%x = %s\n", ChanConfig.EcanParametersB.EcanNlpUnConv,
			   "EcanParametersB.EcanNlpUnConv");
		printk("%x = %s\n", ChanConfig.EcanParametersB.EcanNlpMaxSuppress,
			   "EcanParametersB.EcanNlpMaxSuppress");
		printk("%x = %s\n", ChanConfig.EcanParametersB.EcanCngThreshold,
			   "EcanParametersB.EcanCngThreshold");
		printk("%x = %s\n", ChanConfig.EcanParametersB.EcanAdaptLimit,
			   "EcanParametersB.EcanAdaptLimit");
		printk("%x = %s\n", ChanConfig.EcanParametersB.EcanCrossCorrLimit,
			   "EcanParametersB.EcanCrossCorrLimit");
		printk("%x = %s\n", ChanConfig.EcanParametersB.EcanNumFirSegments,
			   "EcanParametersB.EcanNumFirSegments");
		printk("%x = %s\n", ChanConfig.EcanParametersB.EcanFirSegmentLen,
			   "EcanParametersB.EcanFirSegmentLen");
		printk("%x = %s\n", ChanConfig.SoftwareCompand, "SoftwareCompand");
		printk("%x = %s\n", ChanConfig.FrameRate, "FrameRate");

	}
	return;
}

static unsigned short int rcb_card_dsp_ping(struct rcb_card_t *rcb_card)
{

	gpakPingDspStat_t ping_stat;
	unsigned short int dsp_ver;

	ping_stat = gpakPingDsp(rcb_card, rcb_card->pos, &dsp_ver);

	if (debug & DEBUG_DSP) {
		if (ping_stat == PngSuccess)
			printk("rcbfx %d: G168 DSP Ping DSP Version %x\n", rcb_card->pos + 1,
				   dsp_ver);
		else
			printk("rcbfx %d: G168 DSP Ping Error %d\n", rcb_card->pos + 1, ping_stat);
	}

	if (ping_stat == PngSuccess)
		return dsp_ver;
	else
		return 0;
}

static void rcbfx_chan_ec_enable(struct rcb_card_t *rcb_card, int chan_num)
{
	gpakAlgControlStat_t a_c_stat;
	GPAK_AlgControlStat_t a_c_err;
	unsigned short int DspId = rcb_card->pos;

	if (debug & DEBUG_DSP)
		printk("rcbfx: %d: Echo Can enable DSP %d EC Chan %d\n", rcb_card->pos + 1, 1,
			   chan_num - 1);

	if ((a_c_stat = gpakAlgControl(rcb_card, DspId, chan_num, EnableEcanB, &a_c_err)))
		printk("rcbfx: %d: G168 DSP Enable Alg Control failed res = %d error = %d\n",
			   rcb_card->pos + 1, a_c_stat, a_c_err);

	return;
}

static void rcbfx_chan_ec_disable(struct rcb_card_t *rcb_card, int chan_num)
{
	gpakAlgControlStat_t a_c_stat;
	GPAK_AlgControlStat_t a_c_err;
	unsigned short int DspId = rcb_card->pos;

	if (debug & DEBUG_DSP)
		printk("rcbfx: %d: Echo Can disable DSP %d EC Chan %d\n", rcb_card->pos + 1, 1,
			   chan_num);

	if ((a_c_stat = gpakAlgControl(rcb_card, DspId, chan_num, BypassEcanB, &a_c_err)))
		printk("rcbfx: %d: G168 DSP Enable Alg Control failed res = %d error = %d\n",
			   rcb_card->pos + 1, a_c_stat, a_c_err);

	return;
}

/*static int rcbfx_dahdi_chan_echocan(struct dahdi_chan *chan, int eclen)*/
static int rcbfx_echocan_create(struct dahdi_chan *chan, struct dahdi_echocanparams *ecp,
								struct dahdi_echocanparam *p,
								struct dahdi_echocan_state **ec)
{
	struct dahdi_span *span = chan->span;
	struct rcb_card_t *rcb_card = container_of(span, struct rcb_card_t, span);
	int chan_num;

	chan_num = chan->chanpos - 1;

	if (ecp->param_count > 0) {
		printk(KERN_WARNING
			   "rcbfx: echo canceller does not support parameters; failing request\n");
		return -EINVAL;
	}

	if (debug & DEBUG_DSP) {
		printk("rcbfx: %d Echo Can control Span %d Chan %d daddy chan %d\n",
			   rcb_card->pos + 1, 1, chan_num, chan->channo);
		printk("DSP up %x\n", rcb_card->dsp_up);
	}
	if (rcb_card->dsp_up == 1) {
		*ec = rcb_card->ec[chan_num];
		(*ec)->ops = &my_ec_ops;
		(*ec)->features = my_ec_features;
		rcb_card->nextec |= (1 << chan_num);
		queue_work(rcb_card->wq, &rcb_card->work);
	}
	return 0;
}

static void rcbfx_echocan_free(struct dahdi_chan *chan, struct dahdi_echocan_state *ec)
{
	struct dahdi_span *span = chan->span;
	struct rcb_card_t *rcb_card = container_of(span, struct rcb_card_t, span);
	int chan_num;

	memset(ec, 0, sizeof(*ec));
	chan_num = chan->chanpos - 1;

	if (debug & DEBUG_DSP) {
		printk("rcbfx: %d Echo Can control Span %d Chan %d daddy chan %d\n",
			   rcb_card->pos + 1, 1, chan_num, chan->channo);
		printk("DSP up %x\n", rcb_card->dsp_up);
	}

	if (rcb_card->dsp_up == 1) {
		rcb_card->nextec &= ~(1 << chan_num);
		queue_work(rcb_card->wq, &rcb_card->work);
	}
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void echocan_bh(void *data)
{
	struct rcb_card_t *rcb_card = data;
#else
static void echocan_bh(struct work_struct *data)
{
	struct rcb_card_t *rcb_card = container_of(data, struct rcb_card_t, work);
#endif
	unsigned int todo, chan_num;

	todo = rcb_card->nextec ^ rcb_card->currec;
	if (debug & DEBUG_DSP) {
		printk("rcbfx %d Echo Can control bh change %x to %x\n", rcb_card->pos + 1, todo,
			   (rcb_card->nextec & todo));
		printk("nextec %x currec %x\n", rcb_card->nextec, rcb_card->currec);
	}

	for (chan_num = 0; chan_num < rcb_card->num_chans; chan_num++) {

		if (todo & (1 << chan_num)) {
			if (rcb_card->nextec & (1 << chan_num)) {
				rcbfx_chan_ec_enable(rcb_card, chan_num);
				rcb_card->currec |= (1 << chan_num);
				schedule();
			} else {
				rcb_card->currec &= ~(1 << chan_num);
				schedule();
			}
		}
	}
}


static int rcb_card_dsp_init(struct rcb_card_t *rcb_card)
{
	gpakDownloadStatus_t dl_res = 0;
	gpakConfigPortStatus_t cp_res;
	GPAK_PortConfigStat_t cp_error;
	GPAK_ChannelConfigStat_t chan_config_err;
	int chan_num;
	gpakConfigChanStatus_t chan_conf_stat;
	int dsp_chans = 0;
	int dsp_in_use = 0;
	gpakReadFramingStatsStatus_t framing_status_status;
	unsigned short int ec1, ec2, ec3, dmaec, slips;

	*(volatile __u32 *) (rcb_card->memaddr + RCB_EC_ENA) = 0;
	*(volatile __u32 *) (rcb_card->memaddr + RCB_EC_ENB) = 0;

	if (no_ec)
		return 0;

	rcb_card->hpi_fast = 0;
	*(volatile __u32 *) (rcb_card->memaddr + RCB_HPIC) = 0;
	rcb_card_wait_hpi(rcb_card, RCB_HRDY);
	rcb_card->hpi_xadd = 0;

	/* quit here if no dsp channels on */
	if (!(dsp_chans = rcb_card->chanflag)) {
		printk("rcbfx %d: G168 DSP Disabled\n", rcb_card->pos + 1);
		return 0;
	}

	if (rcb_card->dsp_type == DSP_5507) {

		/* Load the loader file */
		if ((dl_res = gpakDownloadLoader(rcb_card, rcb_card->pos, loader_file))) {
			printk("rcbfx %d: G168 DSP Loader Loader Failed %d\n", rcb_card->pos + 1,
				   dl_res);
			return -1;
		}
		if (debug & DEBUG_DSP)
			printk("rcbfx %d: G168 DSP Loader Loader Sucess\n", rcb_card->pos + 1);

		/* execute the loader */
		rcb_card_dsp_set(rcb_card, DSP_ENTRY_ADDR_LO, (0xFFFF & BL_DSP_BOOTLOADER_ENTRY));
		rcb_card_dsp_set(rcb_card, DSP_ENTRY_ADDR_HI,
						 (0xFF00 | (0xFF & (BL_DSP_BOOTLOADER_ENTRY >> 16))));

		if (debug & DEBUG_DSP) {
			printk("HPIA 0x0061 HPID %x\n", rcb_card_dsp_get(rcb_card, 0x0061));
			printk("HPIA 0x0060 HPID %x\n", rcb_card_dsp_get(rcb_card, 0x0060));
			printk("HPIA 0x3800 HPID %x\n", rcb_card_dsp_get(rcb_card, 0x3800));
			printk("HPIA 0x3801 HPID %x\n", rcb_card_dsp_get(rcb_card, 0x3801));
			printk("HPIA 0x3802 HPID %x\n", rcb_card_dsp_get(rcb_card, 0x3802));
			printk("HPIA 0x3803 HPID %x\n", rcb_card_dsp_get(rcb_card, 0x3803));
			printk("HPIA 0x3804 HPID %x\n", rcb_card_dsp_get(rcb_card, 0x3804));
			printk("HPIA 0x3805 HPID %x\n", rcb_card_dsp_get(rcb_card, 0x3805));
		}

	}

	if (rcb_card->dsp_type == DSP_5507) {
		if ((dl_res = gpakDownloadDsp_5507(rcb_card, rcb_card->pos, app_file))) {
			printk("rcbfx %d: G168 DSP App Loader Failed %d\n", rcb_card->pos + 1,
				   dl_res);
			return -1;
		}
	}

	if (rcb_card->dsp_type == DSP_5510) {
		if ((dl_res = gpakDownloadDsp_5510(rcb_card, rcb_card->pos, app_file))) {
			printk("rcbfx %d: G168 DSP App Loader Failed %d\n", rcb_card->pos + 1,
				   dl_res);
			return -1;
		}
	}

	if (debug & DEBUG_DSP)
		printk("rcbfx %d: G168 DSP App Loader Sucess %d\n", rcb_card->pos + 1, dl_res);

	if (rcb_card->dsp_type == DSP_5510) {
		*(volatile __u32 *) (rcb_card->memaddr + RCB_HPIC) = RCB_BL_GO;
		rcb_card_wait_hpi(rcb_card, RCB_HRDY);
	}

	{
		long end_jiffies;
		end_jiffies = jiffies + 10;
		while (end_jiffies > jiffies);
	}

	rcb_card_dsp_ping(rcb_card);
	rcb_card_dsp_ping(rcb_card);
	rcb_card_dsp_ping(rcb_card);
	printk("rcbfx %d: G168 DSP Ping DSP Version %x\n", rcb_card->pos + 1,
		   rcb_card_dsp_ping(rcb_card));

	if (debug & DEBUG_DSP) {
		framing_status_status =
			gpakReadFramingStats(rcb_card, rcb_card->pos, &ec1, &ec2, &ec3, &dmaec,
								 &slips);
		if (framing_status_status == RfsSuccess) {
			printk("rcbfx %d: G168 DSP Framing Status Sucess %d\n", rcb_card->pos + 1,
				   framing_status_status);
			printk("rcbfx %d: G168 DSP Framing Status %d %d %d %d %d\n",
				   rcb_card->pos + 1, ec1, ec2, ec3, dmaec, slips);
		} else {
			printk("rcbfx %d: G168 DSP Framing Status Failed %d\n", rcb_card->pos + 1,
				   framing_status_status);
			printk("rcbfx %d: G168 DSP Framing Status %d %d %d %d %d\n",
				   rcb_card->pos + 1, ec1, ec2, ec3, dmaec, slips);
		}
	}

	Gpak_24_chan_port_config.FirstSlotMask1 = (rcb_card->chanflag & 0xffff);
	Gpak_24_chan_port_config.SecSlotMask1 = ((rcb_card->chanflag >> 16) & 0xffff);

	Gpak_24_chan_port_config.FirstSlotMask2 = (rcb_card->chanflag & 0xffff);
	Gpak_24_chan_port_config.SecSlotMask2 = ((rcb_card->chanflag >> 16) & 0xffff);

	rcb_card_dsp_show_portconfig(Gpak_24_chan_port_config);


	if ((cp_res =
		 gpakConfigurePorts(rcb_card, rcb_card->pos, &Gpak_24_chan_port_config,
							&cp_error))) {
		printk("rcbfx %d: G168 DSP Port Config failed res = %d error = %d\n",
			   rcb_card->pos + 1, cp_res, cp_error);
		return -1;
	}


	if (debug & DEBUG_DSP)
		printk("rcbfx %d: G168 DSP Port Config success %d\n", rcb_card->pos + 1, cp_res);

	rcb_card_dsp_ping(rcb_card);

	if (debug & DEBUG_DSP) {
		framing_status_status =
			gpakReadFramingStats(rcb_card, rcb_card->pos, &ec1, &ec1, &ec3, &dmaec,
								 &slips);
		if (framing_status_status == RfsSuccess) {
			printk("rcbfx %d: G168 DSP Framing Status Sucess %d\n", rcb_card->pos + 1,
				   framing_status_status);
			printk("rcbfx %d: G168 DSP Framing Status %d %d %d %d %d\n",
				   rcb_card->pos + 1, ec1, ec2, ec3, dmaec, slips);
		} else {
			printk("rcbfx %d: G168 DSP Framing Status Failed %d\n", rcb_card->pos + 1,
				   framing_status_status);
			printk("rcbfx %d: G168 DSP Framing Status %d %d %d %d %d\n",
				   rcb_card->pos + 1, ec1, ec2, ec3, dmaec, slips);
		}
	}

	Gpak_chan_config.EcanParametersA.EcanNlpType = nlp_type;
	Gpak_chan_config.EcanParametersB.EcanNlpType = nlp_type;

	for (chan_num = 0; chan_num < rcb_card->num_chans; chan_num++) {

		if (rcb_card->chanflag & (1 << chan_num)) {

			dsp_in_use++;
			Gpak_chan_config.EcanEnableA = Enabled;
			Gpak_chan_config.SoftwareCompand = cmpPCMU;

			Gpak_chan_config.PcmInSlotA = chan_num;
			Gpak_chan_config.PcmOutSlotA = chan_num;
			Gpak_chan_config.PcmInSlotB = chan_num;
			Gpak_chan_config.PcmOutSlotB = chan_num;

			rcb_card_dsp_show_chanconfig(Gpak_chan_config);

			if ((chan_conf_stat =
				 gpakConfigureChannel(rcb_card, rcb_card->pos, chan_num, tdmToTdm,
									  &Gpak_chan_config, &chan_config_err))) {
				printk("rcbfx %d: Chan %d G168 DSP Chan Config failed error = %d  %d\n",
					   rcb_card->pos + 1, chan_num + 1, chan_config_err, chan_conf_stat);
				return -1;
			}

			else if (debug & DEBUG_DSP)
				printk("rcbfx %d: G168 DSP Chan %d Config success %d\n",
					   rcb_card->pos + 1, chan_num, chan_conf_stat);

			rcb_card_dsp_ping(rcb_card);

			rcbfx_chan_ec_disable(rcb_card, chan_num);

		}
	}

	/* switch to dsp audio stream */
	if ((use_ec == -1) || (use_ec == 1))
		*(volatile __u8 *) (rcb_card->memaddr + EC_CNTL) |= EC_ON;

	rcb_card->dsp_up = 1;

	*(volatile __u32 *) (rcb_card->memaddr + RCB_EC_ENA) = 0xffffffff;
	*(volatile __u32 *) (rcb_card->memaddr + RCB_EC_ENB) = 0;

	rcb_card->wq = create_singlethread_workqueue("rcbfx_ec");

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
	INIT_WORK(&rcb_card->work, echocan_bh, rcb_card);
#else
	INIT_WORK(&rcb_card->work, echocan_bh);
#endif


	printk("rcbfx %d: G168 DSP Active and Servicing %d Channels - %x\n",
		   rcb_card->pos + 1, dsp_in_use, dsp_chans);

	return dsp_chans;
}

#endif

static int __devinit rcb_card_init_one(struct pci_dev *pdev,
									   const struct pci_device_id *ent)
{
	int res, regnum;
	static struct rcb_card_t *rcb_card;
	struct rcb_card_desc *d = (struct rcb_card_desc *) ent->driver_data;
	int x, i;
	static int initd_ifaces = 0;

	if (initd_ifaces) {
		memset((void *) ifaces, 0, (sizeof(struct rcb_card_t *)) * RH_MAX_IFACES);
		initd_ifaces = 1;
	}
	for (x = 0; x < RH_MAX_IFACES; x++)
		if (!ifaces[x])
			break;
	if (x >= RH_MAX_IFACES) {
		/* had some chickens EI */
		printk("Too many interfaces\n");
		return -EIO;
	}

	if (pci_enable_device(pdev)) {
		/* had some cows EI */
		printk("No Rhino spotted\n");
		res = -EIO;
	} else {
		rcb_card = kmalloc(sizeof(struct rcb_card_t), GFP_KERNEL);
		if (rcb_card) {
			int cardcount = 0;
			ifaces[x] = rcb_card;
			memset(rcb_card, 0, sizeof(struct rcb_card_t));

			for (i = 0; i < d->num_chans; i++) {
				printk("alloc chan %x ec %x\n", i, i);
				if (!
					(rcb_card->chans[i] =
					 kmalloc(sizeof(*rcb_card->chans[i]), GFP_KERNEL))) {
					return -ENOMEM;
				}
				memset(rcb_card->chans[i], 0, sizeof(*rcb_card->chans[i]));

				if (!(rcb_card->ec[i] = kmalloc(sizeof(*rcb_card->ec[i]), GFP_KERNEL))) {
					return -ENOMEM;
				}
				memset(rcb_card->ec[i], 0, sizeof(*rcb_card->ec[i]));

			}

			spin_lock_init(&rcb_card->lock);
			rcb_card->curcard = -1;
			rcb_card->baseaddr = pci_resource_start(pdev, 0);
			rcb_card->memlen = pci_resource_len(pdev, 0);
			rcb_card->chans_configed = 0;
			rcb_card->dev = pdev;
			rcb_card->pos = x;

			rcb_card->num_chans = d->num_chans;
			rcb_card->num_slots = d->num_slots;
			rcb_card->variety = d->name;
			rcb_card->hw_ver_min = d->hw_ver_min;
			if (rcb_card->num_chans > 8)
				rcb_card->dsp_type = DSP_5510;
			else
				rcb_card->dsp_type = DSP_5507;

			if (request_region(rcb_card->baseaddr, rcb_card->memlen, rcb_card->variety))
				rcb_card->freeregion = 1;

			rcb_card->memaddr = ioremap_nocache(rcb_card->baseaddr, rcb_card->memlen);
			printk("rcbfx %d: Rhino PCI BAR0 %lx IOMem mapped at %lx\n",
				   rcb_card->pos + 1, (long unsigned int) rcb_card->baseaddr,
				   (long unsigned int) rcb_card->memaddr);

			/*  chunksize * num channels * 2 swap buffers * 2 read and write */
			rcb_card->writechunk =
				(unsigned char *) pci_alloc_consistent(pdev,
													   DAHDI_MAX_CHUNKSIZE *
													   rcb_card->num_slots * 2 * 2,
													   &rcb_card->writedma);

			if (!rcb_card->writechunk) {
				printk("rcbfx %d: Unable to allocate DMA-able memory\n",
					   rcb_card->pos + 1);
				rcb_card_cleaner(rcb_card, IOUNMAP);
				return -ENOMEM;
			}
			/* read starts @ write +  8 samples * 4 channels * 2 buffers later = 0x40 bytes */
			rcb_card->readchunk = rcb_card->writechunk + (DAHDI_MAX_CHUNKSIZE * rcb_card->num_slots * 2);	/* half the total */
			rcb_card->readdma = rcb_card->writedma + (DAHDI_MAX_CHUNKSIZE * rcb_card->num_slots * 2);	/* in bytes */

			pci_set_master(pdev);

			pci_set_drvdata(pdev, rcb_card);

			if (request_irq
				(pdev->irq, rcb_card_interrupt, DAHDI_IRQ_SHARED, rcb_card->variety,
				 rcb_card)) {
				printk("rcbfx %d: Unable to request IRQ %d\n", rcb_card->pos + 1,
					   pdev->irq);
				rcb_card_cleaner(rcb_card,
								 RH_KFREE | PCI_FREE | IOUNMAP | FREE_DMA | STOP_DMA |
								 FREE_INT | ZUNREG);
				return -EIO;	/* had some pigs EI */
			}

			/* sticks in here without up */
			if (rcb_card_hardware_init(rcb_card)) {
				printk("rcbfx %d: Unable to initialize hardware\n", rcb_card->pos + 1);
				rcb_card_cleaner(rcb_card,
								 RH_KFREE | PCI_FREE | IOUNMAP | FREE_DMA | STOP_DMA |
								 FREE_INT | ZUNREG);
				return -EIO;	/* had some ducks EI */
			}

			rcb_card->dsp_up = 0;

#ifdef USE_G168_DSP
			if (rcb_card_dsp_init(rcb_card) < 0)
				printk("rcbfx %d: Unable to initialize G168 DSP\n", rcb_card->pos + 1);
#endif

			if ((use_ec_zap > 0) || (use_ec_zap == 0))	/* forced value */
				zt_ec_chanmap = use_ec_zap;
			else if (rcb_card->dsp_up == 0)	/* Switch on if there is no DSP */
				zt_ec_chanmap = -1;
			else
				zt_ec_chanmap = 0;


			if (use_ec == 1)
				*(volatile __u8 *) (rcb_card->memaddr + EC_CNTL) |= EC_ON;

			if (use_ec == 0)
				*(volatile __u8 *) (rcb_card->memaddr + EC_CNTL) &= ~EC_ON;

			if (rcb_card_initialize(rcb_card)) {
				/* set up and register span and channels */
				printk("rcbfx %d: Unable to initialize \n", rcb_card->pos + 1);
				rcb_card_cleaner(rcb_card,
								 RH_KFREE | IOUNMAP | FREE_DMA | STOP_DMA | FREE_INT);
				return -EIO;	/* had some goats EI */
			}

			printk("rcbfx %d: Starting DMA\n", rcb_card->pos + 1);

			rcb_card_start_dma(rcb_card);

			for (regnum = 0; regnum < P_TBL_CNT; regnum++) {

				*(volatile __u8 *) (rcb_card->memaddr + PARAM_TBL + regnum) =
					rcb_settings_default[regnum];
			}

			/* notify changes */
			*(volatile __u32 *) (rcb_card->memaddr + RCB_TXSIGSTAT) = 0x01000;

			for (x = 0; x < rcb_card->num_chans; x++) {
				if (rcb_card->chanflag & (1 << x))
					cardcount++;
			}
			/* let board run signaling data now */
			/* notify changes */
			*(volatile __u8 *) (rcb_card->memaddr + RCB_STATOUT) |= 0x01;
			if (debug)
				printk("rcbfx %d: Statout = %x\n", rcb_card->pos + 1,
					   *(volatile __u8 *) (rcb_card->memaddr + RCB_STATOUT));
			/* notify changes */
			*(volatile __u32 *) (rcb_card->memaddr + RCB_TXSIGSTAT) = 0x04000;

			if ((rcb_card->dev->device == PCI_DEVICE_RCB4FXO) ||
				(rcb_card->dev->device == PCI_DEVICE_RCB24FXO) ||
				(rcb_card->dev->device == PCI_DEVICE_RCB24FXS))

				printk("rcbfx %d: Spotted a Rhino: %s (%d channels)\n", rcb_card->pos + 1,
					   rcb_card->variety, cardcount);
			else
				printk("rcbfx %d: Spotted a Rhino: %s (%d modules)\n", rcb_card->pos + 1,
					   rcb_card->variety, cardcount / 2);

			res = 0;

		} else					/*  if (!rcb_card) */
			res = -ENOMEM;
	}
	return res;
}

static void __devexit rcb_card_remove_one(struct pci_dev *pdev)
{
	struct rcb_card_t *rcb_card = pci_get_drvdata(pdev);
	if (rcb_card) {
		if (rcb_card->dsp_up) {
			flush_workqueue(rcb_card->wq);
			destroy_workqueue(rcb_card->wq);
		}

		/* reset DSP */
		*(volatile __u8 *) (rcb_card->memaddr + FW_BOOT) = 0x00 | DSP_RST;
		rcb_card_stop_dma(rcb_card);
		*(volatile __u8 *) (rcb_card->memaddr + RCB_STATOUT) &= ~0x00;
		if (debug)
			printk("rcbfx %d: Statout = %x\n", rcb_card->pos + 1,
				   *(volatile __u8 *) (rcb_card->memaddr + RCB_STATOUT));
		*(volatile __u8 *) (rcb_card->memaddr + FW_DATA) = 0x00;
		*(volatile __u8 *) (rcb_card->memaddr + FW_COMOUT) = 0x00;
		pci_free_consistent(pdev, DAHDI_MAX_CHUNKSIZE * 2 * rcb_card->num_chans,
							(void *) rcb_card->writechunk, rcb_card->writedma);
		free_irq(pdev->irq, rcb_card);
		if (!rcb_card->usecount)
			rcb_card_release(rcb_card);
		else
			rcb_card->dead = 1;
	}
}

static struct pci_device_id rcb_card_pci_tbl[] = {
/*    vendor                              subvendor               class   driver_data */
/*                      device                        subdevice      class_mask */
	{PCI_VENDOR_RHINO, PCI_DEVICE_RCB4FXO, PCI_ANY_ID, PCI_ANY_ID, 0, 0,
	 (unsigned long) &rcb4fxo},
	{PCI_VENDOR_RHINO, PCI_DEVICE_RCB8FXX, PCI_ANY_ID, PCI_ANY_ID, 0, 0,
	 (unsigned long) &rcb8fxx},
	{PCI_VENDOR_RHINO, PCI_DEVICE_RCB24FXS, PCI_ANY_ID, PCI_ANY_ID, 0, 0,
	 (unsigned long) &rcb24fxs},
	{PCI_VENDOR_RHINO, PCI_DEVICE_RCB24FXX, PCI_ANY_ID, PCI_ANY_ID, 0, 0,
	 (unsigned long) &rcb24fxx},
	{PCI_VENDOR_RHINO, PCI_DEVICE_RCB24FXO, PCI_ANY_ID, PCI_ANY_ID, 0, 0,
	 (unsigned long) &rcb24fxo},
	{0}
};

MODULE_DEVICE_TABLE(pci, rcb_card_pci_tbl);

static struct pci_driver rcb_driver = {
  name:"rcbfx",
  probe:rcb_card_init_one,
  remove:__devexit_p(rcb_card_remove_one),
  suspend:NULL,
  resume:NULL,
  id_table:rcb_card_pci_tbl,
};

static int __init rcb_card_init(void)
{
	int res;
	res = dahdi_pci_module(&rcb_driver);
	if (res)
		return -ENODEV;
	return 0;
}

static void __exit rcb_card_cleanup(void)
{
	pci_unregister_driver(&rcb_driver);
}

module_param(force_fw, int, 0600);
MODULE_PARM_DESC(force_fw, "Reprogram firmware regardless of version");
module_param(debug, int, 0600);
MODULE_PARM_DESC(debug, "1 for debugging messages");
module_param(nlp_type, int, 0600);
MODULE_PARM_DESC(nlp_type, "0 - off, 1 - mute, 2 - rand, 3 - hoth, 4 - supp");

module_param(zt_ec_chanmap, int, 0600);
module_param(fxs_alg_chanmap, int, 0600);
module_param(fxo_alg_chanmap, int, 0600);
module_param(use_fxs_chanmap, int, 0600);
module_param(use_fxo_chanmap, int, 0600);
#if defined(module_param_array) && LINUX_VERSION_CODE > KERNEL_VERSION(2,6,9)
module_param_array(lvs, int, &arr_argc, 0600);
module_param_array(reg_val, int, &arr_argc, 0600);
#endif /* param arrays */
module_param(battime, int, 0600);
module_param(reg_addr, int, 0600);
module_param(no_ec, int, 0600);

MODULE_DESCRIPTION("Rhino Equipment Modular Analog Interface Driver " DAHDI_VERSION);
MODULE_AUTHOR
	("Bob Conklin <helpdesk@rhinoequipment.com>\n\tBryce Chidester <helpdesk@rhinoequipment.com>\n\tMatthew Gessner <helpdesk@rhinoequipment.com");
MODULE_VERSION(DAHDI_VERSION);

#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif

module_init(rcb_card_init);
module_exit(rcb_card_cleanup);
