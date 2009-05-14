/*
 * Digium, Inc.  Wildcard TE12xP T1/E1 card Driver
 *
 * Written by Michael Spiceland <mspiceland@digium.com>
 *
 * Adapted from the wctdm24xxp and wcte11xp drivers originally
 * written by Mark Spencer <markster@digium.com>
 *            Matthew Fredrickson <creslin@digium.com>
 *            William Meadows <wmeadows@digium.com>
 *
 * Copyright (C) 2007-2009, Digium, Inc.
 *
 * All rights reserved.
 *
 */

/*
 * See http://www.asterisk.org for more information about
 * the Asterisk project. Please do not directly contact
 * any of the maintainers of this project for assistance;
 * the project provides a web site, mailing lists and IRC
 * channels for your use.
 *
 * This program is free software, distributed under the terms of
 * the GNU General Public License Version 2 as published by the
 * Free Software Foundation. See the LICENSE file included with
 * this program for more details.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/proc_fs.h>
#include <linux/moduleparam.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/delay.h>

#include <dahdi/kernel.h>

#include "wct4xxp/wct4xxp.h"	/* For certain definitions */

#include "voicebus/voicebus.h"
#include "wcte12xp.h"

#include "voicebus/GpakCust.h"
#include "voicebus/GpakApi.h"

struct pci_driver te12xp_driver;

int debug = 0;
static int j1mode = 0;
static int alarmdebounce = 0;
static int loopback = 0;
static int t1e1override = -1;
static int unchannelized = 0;
static int latency = VOICEBUS_DEFAULT_LATENCY;
int vpmsupport = 1;
static int vpmtsisupport = 0;

int vpmnlptype = DEFAULT_NLPTYPE;
int vpmnlpthresh = DEFAULT_NLPTHRESH;
int vpmnlpmaxsupp = DEFAULT_NLPMAXSUPP;

static int echocan_create(struct dahdi_chan *chan, struct dahdi_echocanparams *ecp,
			   struct dahdi_echocanparam *p, struct dahdi_echocan_state **ec);
static void echocan_free(struct dahdi_chan *chan, struct dahdi_echocan_state *ec);

static const struct dahdi_echocan_features vpm150m_ec_features = {
	.NLP_automatic = 1,
	.CED_tx_detect = 1,
	.CED_rx_detect = 1,
};

static const struct dahdi_echocan_ops vpm150m_ec_ops = {
	.name = "VPM150M",
	.echocan_free = echocan_free,
};

struct t1 *ifaces[WC_MAX_IFACES];
spinlock_t ifacelock = SPIN_LOCK_UNLOCKED;

struct t1_desc {
	char *name;
	int flags;
};

static struct t1_desc te120p = { "Wildcard TE120P", 0 };
static struct t1_desc te122 = { "Wildcard TE122", 0 };
static struct t1_desc te121 = { "Wildcard TE121", 0 };

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
static kmem_cache_t *cmd_cache;
#else
static struct kmem_cache *cmd_cache;
#endif

static struct command *get_free_cmd(struct t1 *wc)
{
	struct command *cmd;
	cmd = kmem_cache_alloc(cmd_cache, GFP_ATOMIC);
	if (cmd) {
		memset(cmd, 0, sizeof(*cmd));
		INIT_LIST_HEAD(&cmd->node);
	}
	return cmd;
}

static void free_cmd(struct t1 *wc, struct command *cmd)
{
	kmem_cache_free(cmd_cache, cmd);
}

static struct command *get_pending_cmd(struct t1 *wc)
{
	struct command *cmd = NULL;
	unsigned long flags;
	spin_lock_irqsave(&wc->cmd_list_lock, flags);
	if (!list_empty(&wc->pending_cmds)) {
		cmd = list_entry(wc->pending_cmds.next, struct command, node);
		list_move_tail(&cmd->node, &wc->active_cmds);
	}
	spin_unlock_irqrestore(&wc->cmd_list_lock, flags);
	return cmd;
}

static void submit_cmd(struct t1 *wc, struct command *cmd)
{
	unsigned long flags;
	if (cmd->flags & (__CMD_RD | __CMD_PINS))
		init_completion(&cmd->complete);
	spin_lock_irqsave(&wc->cmd_list_lock, flags);
	list_add_tail(&cmd->node, &wc->pending_cmds);
	spin_unlock_irqrestore(&wc->cmd_list_lock, flags);
}

static void resend_cmds(struct t1 *wc)
{
	unsigned long flags;
	spin_lock_irqsave(&wc->cmd_list_lock, flags);
	list_splice_init(&wc->active_cmds, &wc->pending_cmds);
	spin_unlock_irqrestore(&wc->cmd_list_lock, flags);
}

static void cmd_dequeue(struct t1 *wc, volatile unsigned char *writechunk, int eframe, int slot)
{
	struct command *curcmd=NULL;
	u16 address;
	u8 data;
	u32 flags;

	/* Skip audio */
	writechunk += 66;
	/* Search for something waiting to transmit */
	if ((slot < 6) && (eframe) && (eframe < DAHDI_CHUNKSIZE - 1)) { 
		/* only 6 useable cs slots per */

		/* framer */
		curcmd = get_pending_cmd(wc);
		if (curcmd) {
			curcmd->cs_slot = slot;
			curcmd->ident = wc->txident;

			address = curcmd->address;
			data = curcmd->data;
			flags = curcmd->flags;
		} else {
			/* If nothing else, use filler */
			address = 0x4a;
			data = 0;
			flags = __CMD_RD;
		}

		if (flags & __CMD_WR)
			writechunk[CMD_BYTE(slot, 0, 0)] = 0x0c; /* 0c write command */
		else if (flags & __CMD_LEDS)
			writechunk[CMD_BYTE(slot, 0, 0)] = 0x10 | ((address) & 0x0E); /* led set command */
		else if (flags & __CMD_PINS)
			writechunk[CMD_BYTE(slot, 0, 0)] = 0x30; /* CPLD2 pin state */
		else
			writechunk[CMD_BYTE(slot, 0, 0)] = 0x0a; /* read command */
		writechunk[CMD_BYTE(slot, 1, 0)] = address;
		writechunk[CMD_BYTE(slot, 2, 0)] = data;
	}

}

static inline void cmd_decipher(struct t1 *wc, volatile unsigned char *readchunk)
{
	struct command *cmd = NULL;
	unsigned long flags;
	const int IS_VPM = 0;

	/* Skip audio */
	readchunk += 66;
	spin_lock_irqsave(&wc->cmd_list_lock, flags);
	while (!list_empty(&wc->active_cmds)) {
		cmd = list_entry(wc->active_cmds.next, struct command, node);
		if (cmd->ident != wc->rxident)
			break;

		if (cmd->flags & (__CMD_WR | __CMD_LEDS)) {
			/* Nobody is waiting on writes...so let's just
			 * free them here. */
			list_del(&cmd->node);
			free_cmd(wc, cmd);
		} else {
			cmd->data |= readchunk[CMD_BYTE(cmd->cs_slot, 2, IS_VPM)];
			list_del_init(&cmd->node);
			complete(&cmd->complete);
		}
	}
	spin_unlock_irqrestore(&wc->cmd_list_lock, flags);
}

inline void cmd_decipher_vpmadt032(struct t1 *wc, unsigned char *readchunk)
{
	unsigned long flags;
	struct vpmadt032 *vpm = wc->vpmadt032;
	struct vpmadt032_cmd *cmd;

	BUG_ON(!vpm);

	/* If the hardware is not processing any commands currently, then
	 * there is nothing for us to do here. */
	if (list_empty(&vpm->active_cmds)) {
		return;
	}

	spin_lock_irqsave(&vpm->list_lock, flags);
	cmd = list_entry(vpm->active_cmds.next, struct vpmadt032_cmd, node);
	if (wc->rxident == cmd->txident) {
		list_del_init(&cmd->node);
	} else {
		cmd = NULL;
	}
	spin_unlock_irqrestore(&vpm->list_lock, flags);

	if (!cmd) {
		return;
	}

	/* Skip audio */
	readchunk += 66;

	/* Store result */
	cmd->data  = (0xff & readchunk[CMD_BYTE(2, 1, 1)]) << 8;
	cmd->data |= readchunk[CMD_BYTE(2, 2, 1)];
	if (cmd->desc & __VPM150M_WR) {
		/* Writes do not need any acknowledgement */
		list_add_tail(&cmd->node, &vpm->free_cmds);
	} else {
		cmd->desc |= __VPM150M_FIN;
		complete(&cmd->complete);
	}
}

static int config_vpmadt032(struct vpmadt032 *vpm)
{
	int res, channel;
	GpakPortConfig_t portconfig = {0};
	gpakConfigPortStatus_t configportstatus;
	GPAK_PortConfigStat_t pstatus;
	GpakChannelConfig_t chanconfig;
	GPAK_ChannelConfigStat_t cstatus;
	GPAK_AlgControlStat_t algstatus;

	/* First Serial Port config */
	portconfig.SlotsSelect1 = SlotCfgNone;
	portconfig.FirstBlockNum1 = 0;
	portconfig.FirstSlotMask1 = 0x0000;
	portconfig.SecBlockNum1 = 1;
	portconfig.SecSlotMask1 = 0x0000;
	portconfig.SerialWordSize1 = SerWordSize8;
	portconfig.CompandingMode1 = cmpNone;
	portconfig.TxFrameSyncPolarity1 = FrameSyncActHigh;
	portconfig.RxFrameSyncPolarity1 = FrameSyncActHigh;
	portconfig.TxClockPolarity1 = SerClockActHigh;
	portconfig.RxClockPolarity1 = SerClockActHigh;
	portconfig.TxDataDelay1 = DataDelay0;
	portconfig.RxDataDelay1 = DataDelay0;
	portconfig.DxDelay1 = Disabled;
	portconfig.ThirdSlotMask1 = 0x0000;
	portconfig.FouthSlotMask1 = 0x0000;
	portconfig.FifthSlotMask1 = 0x0000;
	portconfig.SixthSlotMask1 = 0x0000;
	portconfig.SevenSlotMask1 = 0x0000;
	portconfig.EightSlotMask1 = 0x0000;

	/* Second Serial Port config */
	portconfig.SlotsSelect2 = SlotCfg8Groups;
	portconfig.FirstBlockNum2 = 0;
	portconfig.FirstSlotMask2 = 0x5554;
	portconfig.SecBlockNum2 = 1;
	portconfig.SecSlotMask2 = 0x5555;
	portconfig.ThirdSlotMask2 = 0x5555;
	portconfig.FouthSlotMask2 = 0x5555;
	portconfig.SerialWordSize2 = SerWordSize8;
	portconfig.CompandingMode2 = cmpNone;
	portconfig.TxFrameSyncPolarity2 = FrameSyncActHigh;
	portconfig.RxFrameSyncPolarity2 = FrameSyncActHigh;
	portconfig.TxClockPolarity2 = SerClockActHigh;
	portconfig.RxClockPolarity2 = SerClockActHigh;
	portconfig.TxDataDelay2 = DataDelay0;
	portconfig.RxDataDelay2 = DataDelay0;
	portconfig.DxDelay2 = Disabled;
	portconfig.FifthSlotMask2 = 0x0001;
	portconfig.SixthSlotMask2 = 0x0000;
	portconfig.SevenSlotMask2 = 0x0000;
	portconfig.EightSlotMask2 = 0x0000;

	/* Third Serial Port Config */
	portconfig.SlotsSelect3 = SlotCfg8Groups;
	portconfig.FirstBlockNum3 = 0;
	portconfig.FirstSlotMask3 = 0x5554;
	portconfig.SecBlockNum3 = 1;
	portconfig.SecSlotMask3 = 0x5555;
	portconfig.SerialWordSize3 = SerWordSize8;
	portconfig.CompandingMode3 = cmpNone;
	portconfig.TxFrameSyncPolarity3 = FrameSyncActHigh;
	portconfig.RxFrameSyncPolarity3 = FrameSyncActHigh;
	portconfig.TxClockPolarity3 = SerClockActHigh;
	portconfig.RxClockPolarity3 = SerClockActLow;
	portconfig.TxDataDelay3 = DataDelay0;
	portconfig.RxDataDelay3 = DataDelay0;
	portconfig.DxDelay3 = Disabled;
	portconfig.ThirdSlotMask3 = 0x5555;
	portconfig.FouthSlotMask3 = 0x5555;
	portconfig.FifthSlotMask3 = 0x0001;
	portconfig.SixthSlotMask3 = 0x0000;
	portconfig.SevenSlotMask3 = 0x0000;
	portconfig.EightSlotMask3 = 0x0000;

	if ((configportstatus = gpakConfigurePorts(vpm->dspid, &portconfig, &pstatus))) {
		printk(KERN_NOTICE "Configuration of ports failed (%d)!\n", configportstatus);
		return -1;
	} else {
		if (vpm->options.debug & DEBUG_ECHOCAN)
			printk(KERN_DEBUG "Configured McBSP ports successfully\n");
	}

	if ((res = gpakPingDsp(vpm->dspid, &vpm->version))) {
		printk(KERN_NOTICE "Error pinging DSP (%d)\n", res);
		return -1;
	}

	for (channel = 0; channel < MAX_CHANNELS_PER_SPAN; ++channel) {
		vpm->curecstate[channel].tap_length = 0;
		vpm->curecstate[channel].nlp_type = vpm->options.vpmnlptype;
		vpm->curecstate[channel].nlp_threshold = vpm->options.vpmnlpthresh;
		vpm->curecstate[channel].nlp_max_suppress = vpm->options.vpmnlpmaxsupp;
		memcpy(&vpm->desiredecstate[channel], &vpm->curecstate[channel], sizeof(vpm->curecstate[channel]));

		vpm->setchanconfig_from_state(vpm, channel, &chanconfig);
		if ((res = gpakConfigureChannel(vpm->dspid, channel, tdmToTdm, &chanconfig, &cstatus))) {
			printk(KERN_NOTICE "Unable to configure channel #%d (%d)", channel, res);
			if (res == 1) {
				printk(", reason %d", cstatus);
			}
			printk("\n");
			return -1;
		}

		if ((res = gpakAlgControl(vpm->dspid, channel, BypassEcanA, &algstatus))) {
			printk(KERN_NOTICE "Unable to disable echo can on channel %d (reason %d:%d)\n", channel + 1, res, algstatus);
			return -1;
		}
	}

	if ((res = gpakPingDsp(vpm->dspid, &vpm->version))) {
		printk(KERN_NOTICE "Error pinging DSP (%d)\n", res);
		return -1;
	}

	set_bit(VPM150M_ACTIVE, &vpm->control);

	return 0;
}

static void cmd_dequeue_vpmadt032(struct t1 *wc, unsigned char *writechunk, int whichframe)
{
	struct vpmadt032_cmd *cmd;
	struct vpmadt032 *vpm = wc->vpmadt032;
	int x;
	unsigned char leds = ~((atomic_read(&wc->txints) / 1000) % 8) & 0x7;

	/* Skip audio */
	writechunk += 66;

	if (test_bit(VPM150M_SPIRESET, &vpm->control) || test_bit(VPM150M_HPIRESET, &vpm->control)) {
		debug_printk(1, "HW Resetting VPMADT032 ...\n");
		for (x = 0; x < 4; x++) {
			if (!x) {
				if (test_and_clear_bit(VPM150M_SPIRESET, &vpm->control))
					writechunk[CMD_BYTE(x, 0, 1)] = 0x08;
				else if (test_and_clear_bit(VPM150M_HPIRESET, &vpm->control))
					writechunk[CMD_BYTE(x, 0, 1)] = 0x0b;
			} else
				writechunk[CMD_BYTE(x, 0, 1)] = 0x00 | leds;
			writechunk[CMD_BYTE(x, 1, 1)] = 0;
			writechunk[CMD_BYTE(x, 2, 1)] = 0x00;
		}
		return;
	}

	if ((cmd = vpmadt032_get_ready_cmd(vpm))) {
		cmd->txident = wc->txident;
#if 0
		printk(KERN_DEBUG "Found command txident = %d, desc = 0x%x, addr = 0x%x, data = 0x%x\n", cmd->txident, cmd->desc, cmd->address, cmd->data);
#endif
		if (cmd->desc & __VPM150M_RWPAGE) {
			/* Set CTRL access to page*/
			writechunk[CMD_BYTE(0, 0, 1)] = (0x8 << 4);
			writechunk[CMD_BYTE(0, 1, 1)] = 0;
			writechunk[CMD_BYTE(0, 2, 1)] = 0x20;

			/* Do a page write */
			if (cmd->desc & __VPM150M_WR) {
				writechunk[CMD_BYTE(1, 0, 1)] = ((0x8 | 0x4) << 4);
			} else {
				writechunk[CMD_BYTE(1, 0, 1)] = ((0x8 | 0x4 | 0x1) << 4);
			}
			writechunk[CMD_BYTE(1, 1, 1)] = 0;
			if (cmd->desc & __VPM150M_WR) {
				writechunk[CMD_BYTE(1, 2, 1)] = cmd->data & 0xf;
			} else {
				writechunk[CMD_BYTE(1, 2, 1)] = 0;
			}

			if (cmd->desc & __VPM150M_WR) {
				/* Fill in buffer to size */
				writechunk[CMD_BYTE(2, 0, 1)] = 0;
				writechunk[CMD_BYTE(2, 1, 1)] = 0;
				writechunk[CMD_BYTE(2, 2, 1)] = 0;
			} else {
				/* Do reads twice b/c of vpmadt032 bug */
				writechunk[CMD_BYTE(2, 0, 1)] = ((0x8 | 0x4 | 0x1) << 4);
				writechunk[CMD_BYTE(2, 1, 1)] = 0;
				writechunk[CMD_BYTE(2, 2, 1)] = 0;
			}

			/* Clear XADD */
			writechunk[CMD_BYTE(3, 0, 1)] = (0x8 << 4);
			writechunk[CMD_BYTE(3, 1, 1)] = 0;
			writechunk[CMD_BYTE(3, 2, 1)] = 0;

			/* Fill in buffer to size */
			writechunk[CMD_BYTE(4, 0, 1)] = 0;
			writechunk[CMD_BYTE(4, 1, 1)] = 0;
			writechunk[CMD_BYTE(4, 2, 1)] = 0;

		} else {
			/* Set address */
			writechunk[CMD_BYTE(0, 0, 1)] = ((0x8 | 0x4) << 4);
			writechunk[CMD_BYTE(0, 1, 1)] = (cmd->address >> 8) & 0xff;
			writechunk[CMD_BYTE(0, 2, 1)] = cmd->address & 0xff;

			/* Send/Get our data */
			if (cmd->desc & __VPM150M_WR) {
				writechunk[CMD_BYTE(1, 0, 1)] = ((0x8 | (0x3 << 1)) << 4);
			} else {
				writechunk[CMD_BYTE(1, 0, 1)] = ((0x8 | (0x3 << 1) | 0x1) << 4);
			}
			writechunk[CMD_BYTE(1, 1, 1)] = (cmd->data >> 8) & 0xff;
			writechunk[CMD_BYTE(1, 2, 1)] = cmd->data & 0xff;

			if (cmd->desc & __VPM150M_WR) {
				/* Fill in */
				writechunk[CMD_BYTE(2, 0, 1)] = 0;
				writechunk[CMD_BYTE(2, 1, 1)] = 0;
				writechunk[CMD_BYTE(2, 2, 1)] = 0;
			} else {
				/* Do this again for reads b/c of the bug in vpmadt032 */
				writechunk[CMD_BYTE(2, 0, 1)] = ((0x8 | (0x3 << 1) | 0x1) << 4);
				writechunk[CMD_BYTE(2, 1, 1)] = (cmd->data >> 8) & 0xff;
				writechunk[CMD_BYTE(2, 2, 1)] = cmd->data & 0xff;
			}

			/* Fill in the rest */
			writechunk[CMD_BYTE(3, 0, 1)] = 0;
			writechunk[CMD_BYTE(3, 1, 1)] = 0;
			writechunk[CMD_BYTE(3, 2, 1)] = 0;

			/* Fill in the rest */
			writechunk[CMD_BYTE(4, 0, 1)] = 0;
			writechunk[CMD_BYTE(4, 1, 1)] = 0;
			writechunk[CMD_BYTE(4, 2, 1)] = 0;
		}
	} else if (test_and_clear_bit(VPM150M_SWRESET, &vpm->control)) {
		debug_printk(1, "Booting  VPMADT032\n");
		for (x = 0; x < 7; x++) {
			if (0 == x)  {
				writechunk[CMD_BYTE(x, 0, 1)] = (0x8 << 4);
			} else {
				writechunk[CMD_BYTE(x, 0, 1)] = 0x00;
			}
			writechunk[CMD_BYTE(x, 1, 1)] = 0;
			if (0 == x) {
				writechunk[CMD_BYTE(x, 2, 1)] = 0x01;
			} else {
				writechunk[CMD_BYTE(x, 2, 1)] = 0x00;
			}
		}
	} else {
		for (x = 0; x < 7; x++) {
			writechunk[CMD_BYTE(x, 0, 1)] = 0x00;
			writechunk[CMD_BYTE(x, 1, 1)] = 0x00;
			writechunk[CMD_BYTE(x, 2, 1)] = 0x00;
		}
	}

	/* Add our leds in */
	for (x = 0; x < 7; x++)
		writechunk[CMD_BYTE(x, 0, 1)] |= leds;

#if 0
	int y;
	for (x = 0; x < 7; x++) {
		for (y = 0; y < 3; y++) {
			if (writechunk[CMD_BYTE(x, y, 1)] & 0x2) {
				module_printk("the test bit is high for byte %d\n", y);
			}
		}
	}
#endif

	/* Now let's figure out if we need to check for DTMF */
	/* polling */
	if (test_bit(VPM150M_ACTIVE, &vpm->control) && !whichframe && !(atomic_read(&wc->txints) % 100))
		schedule_work(&vpm->work);

#if 0
	/* This may be needed sometime in the future to troubleshoot ADT related issues. */
	if (test_bit(VPM150M_ACTIVE, &vpm->control) && !whichframe && !(atomic_read(&wc->txints) % 10000))
		queue_work(vpm->wq, &vpm->work_debug);
#endif
}

static inline int t1_setreg_full(struct t1 *wc, int addr, int val, int vpm_num)
{
	struct command *cmd;
	cmd = get_free_cmd(wc);
	if (!cmd) {
		WARN_ON(1);
		return -ENOMEM;
	}
	cmd->address = addr;
	cmd->data = val;
	cmd->flags |= __CMD_WR;
	if (vpm_num >= 0) {
		cmd->flags |= __CMD_VPM;
		cmd->vpm_num = vpm_num;
	}
	submit_cmd(wc, cmd);
	return 0;
}

static inline int t1_setreg(struct t1 *wc, int addr, int val)
{
	return t1_setreg_full(wc, addr, val, NOT_VPM);
}

static inline int t1_getreg_full(struct t1 *wc, int addr, int vpm_num)
{
	struct command *cmd =  NULL;
	int ret;

	might_sleep();

	cmd = get_free_cmd(wc);
	if (!cmd)
		return -ENOMEM;
	cmd->address = addr;
	cmd->data = 0x00;
	cmd->flags = __CMD_RD;
	if (vpm_num > -1) {
		cmd->flags |= __CMD_VPM;
		cmd->vpm_num = vpm_num;
	}
	submit_cmd(wc, cmd);
	wait_for_completion(&cmd->complete);
	ret = cmd->data;
	free_cmd(wc, cmd);
	return ret;
}

static int t1_getreg(struct t1 *wc, int addr)
{
	return t1_getreg_full(wc, addr, NOT_VPM);
}

static void t1_setleds(struct t1 *wc, int leds)
{
	struct command *cmd;

	leds = (~leds) & 0x0E; /* invert the LED bits (3 downto 1)*/

	cmd = get_free_cmd(wc);
	if (!cmd)
		return;
	cmd->flags |= __CMD_LEDS;
	cmd->address = leds;
	submit_cmd(wc, cmd);
}

static inline int t1_getpins(struct t1 *wc, int inisr)
{
	int ret = 0;
	struct command *cmd;

	cmd = get_free_cmd(wc);
	BUG_ON(!cmd);

	cmd->address = 0x00;
	cmd->data = 0x00;
	cmd->flags = __CMD_PINS;
	submit_cmd(wc, cmd);
	wait_for_completion(&cmd->complete);
	ret = cmd->data;
	free_cmd(wc, cmd);
	return ret;
}

static void __t1xxp_set_clear(struct t1 *wc, int channo)
{
	int i,j;
	int ret;
	unsigned short val=0;
	
	for (i = 0; i < 24; i++) {
		j = (i / 8);
		if (wc->span.chans[i]->flags & DAHDI_FLAG_CLEAR) 
			val |= 1 << (7 - (i % 8));
		if (((i % 8)==7) &&  /* write byte every 8 channels */
		    ((channo < 0) ||    /* channo=-1 means all channels */ 
		     (j == (channo-1)/8) )) { /* only the register for this channo */    
			ret = t1_setreg_full(wc, 0x2f + j, val, NOT_VPM);
			if (ret < 0)
				module_printk("set_clear failed for chan %d!\n",i); 
			val = 0;
		}
	}
}

static void free_wc(struct t1 *wc)
{
	unsigned int x;
	unsigned long flags;
	struct command *cmd;
	LIST_HEAD(list);

	for (x = 0; x < (wc->spantype == TYPE_E1 ? 31 : 24); x++) {
		kfree(wc->chans[x]);
		kfree(wc->ec[x]);
	}

	spin_lock_irqsave(&wc->cmd_list_lock, flags);
	list_splice_init(&wc->active_cmds, &list);
	list_splice_init(&wc->pending_cmds, &list);
	spin_unlock_irqrestore(&wc->cmd_list_lock, flags);
	while (!list_empty(&list)) {
		cmd = list_entry(list.next, struct command, node);
		list_del(&cmd->node);
		free_cmd(wc, cmd);
	}
	kfree(wc);
}

static void t1_release(struct t1 *wc)
{
	dahdi_unregister(&wc->span);
	printk(KERN_INFO "Freed a Wildcard TE12xP.\n");
	free_wc(wc);
}

static void t4_serial_setup(struct t1 *wc)
{
	module_printk("Setting up global serial parameters for %s\n", 
		      wc->spantype == TYPE_E1 ? (unchannelized ? "Unchannelized E1" : "E1") : "T1");

	t1_setreg(wc, 0x85, 0xe0);	/* GPC1: Multiplex mode enabled, FSC is output, active low, RCLK from channel 0 */
	t1_setreg(wc, 0x08, 0x05);	/* IPC: Interrupt push/pull active low */

	/* Global clocks (8.192 Mhz CLK) */
	t1_setreg(wc, 0x92, 0x00);	
	t1_setreg(wc, 0x93, 0x18);
	t1_setreg(wc, 0x94, 0xfb);
	t1_setreg(wc, 0x95, 0x0b);
	t1_setreg(wc, 0x96, 0x00);
	t1_setreg(wc, 0x97, 0x0b);
	t1_setreg(wc, 0x98, 0xdb);
	t1_setreg(wc, 0x99, 0xdf);

	/* Configure interrupts */	
	t1_setreg(wc, 0x46, 0xc0);	/* GCR: Interrupt on Activation/Deactivation of AIX, LOS */

	/* Configure system interface */
	t1_setreg(wc, 0x3e, 0x0a /* 0x02 */);	/* SIC1: 4.096 Mhz clock/bus, double buffer receive / transmit, byte interleaved */
	t1_setreg(wc, 0x3f, 0x00); 	/* SIC2: No FFS, no center receive eliastic buffer, phase 0 */
	t1_setreg(wc, 0x40, 0x04);	/* SIC3: Edges for capture */
	t1_setreg(wc, 0x44, 0x30);	/* CMR1: RCLK is at 8.192 Mhz dejittered */
	t1_setreg(wc, 0x45, 0x00);	/* CMR2: We provide sync and clock for tx and rx. */
	t1_setreg(wc, 0x22, 0x00);	/* XC0: Normal operation of Sa-bits */
	t1_setreg(wc, 0x23, 0x04);	/* XC1: 0 offset */
	t1_setreg(wc, 0x24, 0x00);	/* RC0: Just shy of 255 */
	t1_setreg(wc, 0x25, 0x05);	/* RC1: The rest of RC0 */
	
	/* Configure ports */
	t1_setreg(wc, 0x80, 0x00);	/* PC1: SPYR/SPYX input on RPA/XPA */
	t1_setreg(wc, 0x81, 0x22);	/* PC2: RMFB/XSIG output/input on RPB/XPB */
	t1_setreg(wc, 0x82, 0x65);	/* PC3: Some unused stuff */
	t1_setreg(wc, 0x83, 0x35);	/* PC4: Some more unused stuff */
	t1_setreg(wc, 0x84, 0x31);	/* PC5: XMFS active low, SCLKR is input, RCLK is output */
	t1_setreg(wc, 0x86, 0x03);	/* PC6: CLK1 is Tx Clock output, CLK2 is 8.192 Mhz from DCO-R */
	t1_setreg(wc, 0x3b, 0x00);	/* Clear LCR1 */
}

static void t1_configure_t1(struct t1 *wc, int lineconfig, int txlevel)
{
	unsigned int fmr4, fmr2, fmr1, fmr0, lim2;
	char *framing, *line;
	int mytxlevel;

	if ((txlevel > 7) || (txlevel < 4))
		mytxlevel = 0;
	else
		mytxlevel = txlevel - 4;
	fmr1 = 0x9e; /* FMR1: Mode 0, T1 mode, CRC on for ESF, 2.048 Mhz system data rate, no XAIS */
	fmr2 = 0x22; /* FMR2: no payload loopback, auto send yellow alarm */
	if (loopback)
		fmr2 |= 0x4;

	if (j1mode)
		fmr4 = 0x1c;
	else
		fmr4 = 0x0c; /* FMR4: Lose sync on 2 out of 5 framing bits, auto resync */

	lim2 = 0x21; /* LIM2: 50% peak is a "1", Advanced Loss recovery */
	lim2 |= (mytxlevel << 6);	/* LIM2: Add line buildout */
	t1_setreg(wc, 0x1d, fmr1);
	t1_setreg(wc, 0x1e, fmr2);

	/* Configure line interface */
	if (lineconfig & DAHDI_CONFIG_AMI) {
		line = "AMI";
		fmr0 = 0xa0;
	} else {
		line = "B8ZS";
		fmr0 = 0xf0;
	}
	if (lineconfig & DAHDI_CONFIG_D4) {
		framing = "D4";
	} else {
		framing = "ESF";
		fmr4 |= 0x2;
		fmr2 |= 0xc0;
	}
	t1_setreg(wc, 0x1c, fmr0);

	t1_setreg(wc, 0x20, fmr4);
	t1_setreg(wc, 0x21, 0x40);	/* FMR5: Enable RBS mode */

	t1_setreg(wc, 0x37, 0xf8);	/* LIM1: Clear data in case of LOS, Set receiver threshold (0.5V), No remote loop, no DRS */
	t1_setreg(wc, 0x36, 0x08);	/* LIM0: Enable auto long haul mode, no local loop (must be after LIM1) */

	t1_setreg(wc, 0x02, 0x50);	/* CMDR: Reset the receiver and transmitter line interface */
	t1_setreg(wc, 0x02, 0x00);	/* CMDR: Reset the receiver and transmitter line interface */

	t1_setreg(wc, 0x3a, lim2);	/* LIM2: 50% peak amplitude is a "1" */
	t1_setreg(wc, 0x38, 0x0a);	/* PCD: LOS after 176 consecutive "zeros" */
	t1_setreg(wc, 0x39, 0x15);	/* PCR: 22 "ones" clear LOS */

	if (j1mode)
		t1_setreg(wc, 0x24, 0x80); /* J1 overide */
		
	/* Generate pulse mask for T1 */
	switch (mytxlevel) {
	case 3:
		t1_setreg(wc, 0x26, 0x07);	/* XPM0 */
		t1_setreg(wc, 0x27, 0x01);	/* XPM1 */
		t1_setreg(wc, 0x28, 0x00);	/* XPM2 */
		break;
	case 2:
		t1_setreg(wc, 0x26, 0x8c);	/* XPM0 */
		t1_setreg(wc, 0x27, 0x11);	/* XPM1 */
		t1_setreg(wc, 0x28, 0x01);	/* XPM2 */
		break;
	case 1:
		t1_setreg(wc, 0x26, 0x8c);	/* XPM0 */
		t1_setreg(wc, 0x27, 0x01);	/* XPM1 */
		t1_setreg(wc, 0x28, 0x00);	/* XPM2 */
		break;
	case 0:
	default:
		t1_setreg(wc, 0x26, 0xd7);	/* XPM0 */
		t1_setreg(wc, 0x27, 0x22);	/* XPM1 */
		t1_setreg(wc, 0x28, 0x01);	/* XPM2 */
		break;
	}

	module_printk("Span configured for %s/%s\n", framing, line);
}

static void t1_configure_e1(struct t1 *wc, int lineconfig)
{
	unsigned int fmr2, fmr1, fmr0;
	unsigned int cas = 0;
	char *crc4 = "";
	char *framing, *line;

	fmr1 = 0x46; /* FMR1: E1 mode, Automatic force resync, PCM30 mode, 8.192 Mhz backplane, no XAIS */
	fmr2 = 0x03; /* FMR2: Auto transmit remote alarm, auto loss of multiframe recovery, no payload loopback */
	if (unchannelized)
		fmr2 |= 0x30;
	if (loopback)
		fmr2 |= 0x4;
	if (lineconfig & DAHDI_CONFIG_CRC4) {
		fmr1 |= 0x08;	/* CRC4 transmit */
		fmr2 |= 0xc0;	/* CRC4 receive */
		crc4 = "/CRC4";
	}
	t1_setreg(wc, 0x1d, fmr1);
	t1_setreg(wc, 0x1e, fmr2);

	/* Configure line interface */
	if (lineconfig & DAHDI_CONFIG_AMI) {
		line = "AMI";
		fmr0 = 0xa0;
	} else {
		line = "HDB3";
		fmr0 = 0xf0;
	}
	if (lineconfig & DAHDI_CONFIG_CCS) {
		framing = "CCS";
	} else {
		framing = "CAS";
		cas = 0x40;
	}
	t1_setreg(wc, 0x1c, fmr0);

	if (unchannelized)
		t1_setreg(wc, 0x1f, 0x40);

	t1_setreg(wc, 0x37, 0xf0 /*| 0x6 */ );	/* LIM1: Clear data in case of LOS, Set receiver threshold (0.5V), No remote loop, no DRS */
	t1_setreg(wc, 0x36, 0x08);	/* LIM0: Enable auto long haul mode, no local loop (must be after LIM1) */

	t1_setreg(wc, 0x02, 0x50);	/* CMDR: Reset the receiver and transmitter line interface */
	t1_setreg(wc, 0x02, 0x00);	/* CMDR: Reset the receiver and transmitter line interface */

	/* Condition receive line interface for E1 after reset */
	t1_setreg(wc, 0xbb, 0x17);
	t1_setreg(wc, 0xbc, 0x55);
	t1_setreg(wc, 0xbb, 0x97);
	t1_setreg(wc, 0xbb, 0x11);
	t1_setreg(wc, 0xbc, 0xaa);
	t1_setreg(wc, 0xbb, 0x91);
	t1_setreg(wc, 0xbb, 0x12);
	t1_setreg(wc, 0xbc, 0x55);
	t1_setreg(wc, 0xbb, 0x92);
	t1_setreg(wc, 0xbb, 0x0c);
	t1_setreg(wc, 0xbb, 0x00);
	t1_setreg(wc, 0xbb, 0x8c);
	
	t1_setreg(wc, 0x3a, 0x20);	/* LIM2: 50% peak amplitude is a "1" */
	t1_setreg(wc, 0x38, 0x0a);	/* PCD: LOS after 176 consecutive "zeros" */
	t1_setreg(wc, 0x39, 0x15);	/* PCR: 22 "ones" clear LOS */
	
	t1_setreg(wc, 0x20, 0x9f);	/* XSW: Spare bits all to 1 */
	if (unchannelized)
		t1_setreg(wc, 0x21, 0x3c);
	else
		t1_setreg(wc, 0x21, 0x1c|cas);	/* XSP: E-bit set when async. AXS auto, XSIF to 1 */
	
	
	/* Generate pulse mask for E1 */
	t1_setreg(wc, 0x26, 0x54);	/* XPM0 */
	t1_setreg(wc, 0x27, 0x02);	/* XPM1 */
	t1_setreg(wc, 0x28, 0x00);	/* XPM2 */
	module_printk("Span configured for %s/%s%s\n", framing, line, crc4);
}

static void t1xxp_framer_start(struct t1 *wc, struct dahdi_span *span)
{
	if (wc->spantype == TYPE_E1) { /* if this is an E1 card */
		t1_configure_e1(wc, span->lineconfig);
	} else { /* is a T1 card */
		t1_configure_t1(wc, span->lineconfig, span->txlevel);
		__t1xxp_set_clear(wc, -1);
	}

	set_bit(DAHDI_FLAGBIT_RUNNING, &wc->span.flags);
}

static int t1xxp_startup(struct dahdi_span *span)
{
	struct t1 *wc = span->pvt;
	int i;

	/* initialize the start value for the entire chunk of last ec buffer */
	for (i = 0; i < span->channels; i++) {
		memset(wc->ec_chunk1[i], DAHDI_LIN2X(0, span->chans[i]), DAHDI_CHUNKSIZE);
		memset(wc->ec_chunk2[i], DAHDI_LIN2X(0, span->chans[i]), DAHDI_CHUNKSIZE);
	}

	/* Reset framer with proper parameters and start */
	t1xxp_framer_start(wc, span);
	debug_printk(1, "Calling startup (flags is %lu)\n", span->flags);

	return 0;
}

static int t1xxp_shutdown(struct dahdi_span *span)
{
	struct t1 *wc = span->pvt;
	t1_setreg(wc, 0x46, 0x41);	/* GCR: Interrupt on Activation/Deactivation of AIX, LOS */
	clear_bit(DAHDI_FLAGBIT_RUNNING, &span->flags);
	return 0;
}

static int t1xxp_chanconfig(struct dahdi_chan *chan, int sigtype)
{
	struct t1 *wc = chan->pvt;
	if (test_bit(DAHDI_FLAGBIT_RUNNING, &chan->span->flags) &&
		(wc->spantype != TYPE_E1)) {
		__t1xxp_set_clear(wc, chan->channo);
	}
	return 0;
}

static int t1xxp_spanconfig(struct dahdi_span *span, struct dahdi_lineconfig *lc)
{
	struct t1 *wc = span->pvt;

	/* Do we want to SYNC on receive or not */
	if (lc->sync) {
		set_bit(7, &wc->ctlreg);
		span->syncsrc = span->spanno;
	} else {
		clear_bit(7, &wc->ctlreg);
		span->syncsrc = 0;
	}

	/* If already running, apply changes immediately */
	if (test_bit(DAHDI_FLAGBIT_RUNNING, &span->flags))
		return t1xxp_startup(span);

	return 0;
}

static int t1xxp_rbsbits(struct dahdi_chan *chan, int bits)
{
	u_char m,c;
	int n,b;
	struct t1 *wc = chan->pvt;
	unsigned long flags;
	
	debug_printk(2, "Setting bits to %d on channel %s\n", bits, chan->name);
	if (wc->spantype == TYPE_E1) { /* do it E1 way */
		if (chan->chanpos == 16)
			return 0;

		n = chan->chanpos - 1;
		if (chan->chanpos > 15) n--;
		b = (n % 15);
		spin_lock_irqsave(&wc->reglock, flags);	
		c = wc->txsigs[b];
		m = (n / 15) << 2; /* nibble selector */
		c &= (0xf << m); /* keep the other nibble */
		c |= (bits & 0xf) << (4 - m); /* put our new nibble here */
		wc->txsigs[b] = c;
		spin_unlock_irqrestore(&wc->reglock, flags);
		  /* output them to the chip */
		t1_setreg_full(wc, 0x71 + b, c, NOT_VPM);
	} else if (wc->span.lineconfig & DAHDI_CONFIG_D4) {
		n = chan->chanpos - 1;
		b = (n / 4);
		spin_lock_irqsave(&wc->reglock, flags);	
		c = wc->txsigs[b];
		m = ((3 - (n % 4)) << 1); /* nibble selector */
		c &= ~(0x3 << m); /* keep the other nibble */
		c |= ((bits >> 2) & 0x3) << m; /* put our new nibble here */
		wc->txsigs[b] = c;
		spin_unlock_irqrestore(&wc->reglock, flags);
		/* output them to the chip */
		t1_setreg_full(wc, 0x70 + b, c, NOT_VPM);
		t1_setreg_full(wc, 0x70 + b + 6, c, NOT_VPM);
	} else if (wc->span.lineconfig & DAHDI_CONFIG_ESF) {
		n = chan->chanpos - 1;
		b = (n / 2);
		spin_lock_irqsave(&wc->reglock, flags);	
		c = wc->txsigs[b];
		m = ((n % 2) << 2); /* nibble selector */
		c &= (0xf << m); /* keep the other nibble */
		c |= (bits & 0xf) << (4 - m); /* put our new nibble here */
		wc->txsigs[b] = c;
		spin_unlock_irqrestore(&wc->reglock, flags);
		  /* output them to the chip */
		t1_setreg_full(wc, 0x70 + b, c, NOT_VPM);
	} 
	debug_printk(2,"Finished setting RBS bits\n");

	return 0;
}

static inline void t1_check_sigbits(struct t1 *wc)
{
	int a,i,rxs;

	if (!(test_bit(DAHDI_FLAGBIT_RUNNING, &wc->span.flags)))
		return;
	if (wc->spantype == TYPE_E1) {
		for (i = 0; i < 15; i++) {
			a = t1_getreg(wc, 0x71 + i);
			if (a > -1) {
				/* Get high channel in low bits */
				rxs = (a & 0xf);
				if (!(wc->span.chans[i+16]->sig & DAHDI_SIG_CLEAR)) {
					if (wc->span.chans[i+16]->rxsig != rxs) {
						dahdi_rbsbits(wc->span.chans[i+16], rxs);
					}
				}
				rxs = (a >> 4) & 0xf;
				if (!(wc->span.chans[i]->sig & DAHDI_SIG_CLEAR)) {
					if (wc->span.chans[i]->rxsig != rxs) {
						dahdi_rbsbits(wc->span.chans[i], rxs);
					}
				}
			}
		}
	} else if (wc->span.lineconfig & DAHDI_CONFIG_D4) {
		for (i = 0; i < 24; i+=4) {
			a = t1_getreg(wc, 0x70 + (i>>2));
			if (a > -1) {
				/* Get high channel in low bits */
				rxs = (a & 0x3) << 2;
				if (!(wc->span.chans[i+3]->sig & DAHDI_SIG_CLEAR)) {
					if (wc->span.chans[i+3]->rxsig != rxs) {
						dahdi_rbsbits(wc->span.chans[i+3], rxs);
					}
				}
				rxs = (a & 0xc);
				if (!(wc->span.chans[i+2]->sig & DAHDI_SIG_CLEAR)) {
					if (wc->span.chans[i+2]->rxsig != rxs) {
						dahdi_rbsbits(wc->span.chans[i+2], rxs);
					}
				}
				rxs = (a >> 2) & 0xc;
				if (!(wc->span.chans[i+1]->sig & DAHDI_SIG_CLEAR)) {
					if (wc->span.chans[i+1]->rxsig != rxs) {
						dahdi_rbsbits(wc->span.chans[i+1], rxs);
					}
				}
				rxs = (a >> 4) & 0xc;
				if (!(wc->span.chans[i]->sig & DAHDI_SIG_CLEAR)) {
					if (wc->span.chans[i]->rxsig != rxs) {
						dahdi_rbsbits(wc->span.chans[i], rxs);
					}	
				}
			}
		}
	} else {
		for (i = 0; i < 24; i+=2) {
			a = t1_getreg(wc, 0x70 + (i>>1));
			if (a > -1) {
				/* Get high channel in low bits */
				rxs = (a & 0xf);
				if (!(wc->span.chans[i+1]->sig & DAHDI_SIG_CLEAR)) {
					if (wc->span.chans[i+1]->rxsig != rxs) {
						dahdi_rbsbits(wc->span.chans[i+1], rxs);
					}
				}
				rxs = (a >> 4) & 0xf;
				if (!(wc->span.chans[i]->sig & DAHDI_SIG_CLEAR)) {
					if (wc->span.chans[i]->rxsig != rxs) {
						dahdi_rbsbits(wc->span.chans[i], rxs);
					}
				}
			}
		}
	}
}

static int t1xxp_maint(struct dahdi_span *span, int cmd)
{
	struct t1 *wc = span->pvt;

	if (wc->spantype == TYPE_E1) {
		switch (cmd) {
		case DAHDI_MAINT_NONE:
			module_printk("XXX Turn off local and remote loops E1 XXX\n");
			break;
		case DAHDI_MAINT_LOCALLOOP:
			module_printk("XXX Turn on local loopback E1 XXX\n");
			break;
		case DAHDI_MAINT_REMOTELOOP:
			module_printk("XXX Turn on remote loopback E1 XXX\n");
			break;
		case DAHDI_MAINT_LOOPUP:
			module_printk("XXX Send loopup code E1 XXX\n");
			break;
		case DAHDI_MAINT_LOOPDOWN:
			module_printk("XXX Send loopdown code E1 XXX\n");
			break;
		case DAHDI_MAINT_LOOPSTOP:
			module_printk("XXX Stop sending loop codes E1 XXX\n");
			break;
		default:
			module_printk("Unknown E1 maint command: %d\n", cmd);
			break;
		}
	} else {
		switch (cmd) {
		case DAHDI_MAINT_NONE:
			module_printk("XXX Turn off local and remote loops T1 XXX\n");
			break;
		case DAHDI_MAINT_LOCALLOOP:
			module_printk("XXX Turn on local loop and no remote loop XXX\n");
			break;
		case DAHDI_MAINT_REMOTELOOP:
			module_printk("XXX Turn on remote loopup XXX\n");
			break;
		case DAHDI_MAINT_LOOPUP:
			t1_setreg(wc, 0x21, 0x50);	/* FMR5: Nothing but RBS mode */
			break;
		case DAHDI_MAINT_LOOPDOWN:
			t1_setreg(wc, 0x21, 0x60);	/* FMR5: Nothing but RBS mode */
			break;
		case DAHDI_MAINT_LOOPSTOP:
			t1_setreg(wc, 0x21, 0x40);	/* FMR5: Nothing but RBS mode */
			break;
		default:
			module_printk("Unknown T1 maint command: %d\n", cmd);
			break;
		}
	}

	return 0;
}

static int t1xxp_open(struct dahdi_chan *chan)
{
	if (!try_module_get(THIS_MODULE))
		return -ENXIO;
	else
		return 0;
}

static int t1xxp_close(struct dahdi_chan *chan)
{
	module_put(THIS_MODULE);
	return 0;
}

static int t1xxp_ioctl(struct dahdi_chan *chan, unsigned int cmd, unsigned long data)
{
	switch (cmd) {
	case WCT4_GET_REGS:
		/* Since all register access was moved into the voicebus
		 * module....this was removed.  Although...why does the client
		 * library need access to the registers (debugging)? \todo ..
		 */
		WARN_ON(1);
		return -ENOSYS;
		break;
	default:
		return -ENOTTY;
	}
	return 0;
}

static int echocan_create(struct dahdi_chan *chan, struct dahdi_echocanparams *ecp,
			  struct dahdi_echocanparam *p, struct dahdi_echocan_state **ec)
{
	struct t1 *wc = chan->pvt;
	if (!wc->vpmadt032) {
		return -ENODEV;
	}
	return vpmadt032_echocan_create(wc->vpmadt032, chan->chanpos - 1,
		ecp, p);
}

static void echocan_free(struct dahdi_chan *chan, struct dahdi_echocan_state *ec)
{
	struct t1 *wc = chan->pvt;
	if (!wc->vpmadt032)
		return;

	vpmadt032_echocan_free(wc->vpmadt032, chan, ec);
}

static int t1_software_init(struct t1 *wc)
{
	int x;
	int num;
	struct pci_dev* dev;

	dev = voicebus_get_pci_dev(wc->vb);

	/* Find position */
	for (x = 0; x < sizeof(ifaces) / sizeof(ifaces[0]); x++) {
		if (ifaces[x] == wc) {
			debug_printk(1, "software init for card %d\n",x);
			break;
		}
	}

	if (x == sizeof(ifaces) / sizeof(ifaces[0]))
		return -1;

	t4_serial_setup(wc);

	num = x;
	sprintf(wc->span.name, "WCT1/%d", num);
	snprintf(wc->span.desc, sizeof(wc->span.desc) - 1, "%s Card %d", wc->variety, num);
	wc->span.manufacturer = "Digium";
	strncpy(wc->span.devicetype, wc->variety, sizeof(wc->span.devicetype) - 1);

#if defined(VPM_SUPPORT)
	if (wc->vpmadt032)
		strncat(wc->span.devicetype, " with VPMADT032", sizeof(wc->span.devicetype) - 1);
#endif

	snprintf(wc->span.location, sizeof(wc->span.location) - 1,
		"PCI Bus %02d Slot %02d", dev->bus->number, PCI_SLOT(dev->devfn) + 1);

	wc->span.spanconfig = t1xxp_spanconfig;
	wc->span.chanconfig = t1xxp_chanconfig;
	wc->span.irq = dev->irq;
	wc->span.startup = t1xxp_startup;
	wc->span.shutdown = t1xxp_shutdown;
	wc->span.rbsbits = t1xxp_rbsbits;
	wc->span.maint = t1xxp_maint;
	wc->span.open = t1xxp_open;
	wc->span.close = t1xxp_close;
	wc->span.ioctl = t1xxp_ioctl;
#ifdef VPM_SUPPORT
	if (vpmsupport)
		wc->span.echocan_create = echocan_create;
#endif

	if (wc->spantype == TYPE_E1) {
		if (unchannelized)
			wc->span.channels = 32;
		else
			wc->span.channels = 31;
		wc->span.spantype = "E1";
		wc->span.linecompat = DAHDI_CONFIG_HDB3 | DAHDI_CONFIG_CCS | DAHDI_CONFIG_CRC4;
		wc->span.deflaw = DAHDI_LAW_ALAW;
	} else {
		wc->span.channels = 24;
		wc->span.spantype = "T1";
		wc->span.linecompat = DAHDI_CONFIG_AMI | DAHDI_CONFIG_B8ZS | DAHDI_CONFIG_D4 | DAHDI_CONFIG_ESF;
		wc->span.deflaw = DAHDI_LAW_MULAW;
	}
	wc->span.chans = wc->chans;
	set_bit(DAHDI_FLAGBIT_RBS, &wc->span.flags);
	wc->span.pvt = wc;
	init_waitqueue_head(&wc->span.maintq);
	for (x = 0; x < wc->span.channels; x++) {
		sprintf(wc->chans[x]->name, "WCT1/%d/%d", num, x + 1);
		wc->chans[x]->sigcap = DAHDI_SIG_EM | DAHDI_SIG_CLEAR | DAHDI_SIG_EM_E1 | 
				      DAHDI_SIG_FXSLS | DAHDI_SIG_FXSGS | DAHDI_SIG_MTP2 |
				      DAHDI_SIG_FXSKS | DAHDI_SIG_FXOLS | DAHDI_SIG_DACS_RBS |
				      DAHDI_SIG_FXOGS | DAHDI_SIG_FXOKS | DAHDI_SIG_CAS | DAHDI_SIG_SF;
		wc->chans[x]->pvt = wc;
		wc->chans[x]->chanpos = x + 1;
	}
	if (dahdi_register(&wc->span, 0)) {
		module_printk("Unable to register span with DAHDI\n");
		return -1;
	}

	set_bit(INITIALIZED, &wc->bit_flags);

	return 0;
}

#ifdef VPM_SUPPORT
static inline unsigned char t1_vpm_in(struct t1 *wc, int unit, const unsigned int addr) 
{
		return t1_getreg_full(wc, addr, unit);
}

static inline unsigned char t1_vpm_out(struct t1 *wc, int unit, const unsigned int addr, const unsigned char val) 
{
		return t1_setreg_full(wc, addr, val, unit);
}

#endif

static void setchanconfig_from_state(struct vpmadt032 *vpm, int channel, GpakChannelConfig_t *chanconfig)
{
	const struct vpmadt032_options *options;
	GpakEcanParms_t *p;

	BUG_ON(!vpm);

	options = &vpm->options;

	chanconfig->PcmInPortA = 3;
	chanconfig->PcmInSlotA = (channel + 1) * 2;
	chanconfig->PcmOutPortA = 2;
	chanconfig->PcmOutSlotA = (channel + 1) * 2;
	chanconfig->PcmInPortB = 2;
	chanconfig->PcmInSlotB = (channel + 1) * 2;
	chanconfig->PcmOutPortB = 3;
	chanconfig->PcmOutSlotB = (channel + 1) * 2;
	chanconfig->ToneTypesA = Null_tone;
	chanconfig->MuteToneA = Disabled;
	chanconfig->FaxCngDetA = Disabled;
	chanconfig->ToneTypesB = Null_tone;
	chanconfig->EcanEnableA = Enabled;
	chanconfig->EcanEnableB = Disabled;
	chanconfig->MuteToneB = Disabled;
	chanconfig->FaxCngDetB = Disabled;

	chanconfig->SoftwareCompand = cmpPCMU;

	chanconfig->FrameRate = rate10ms;

	p = &chanconfig->EcanParametersA;

	vpmadt032_get_default_parameters(p);

	p->EcanNlpType = vpm->curecstate[channel].nlp_type;
	p->EcanNlpThreshold = vpm->curecstate[channel].nlp_threshold;
	p->EcanNlpMaxSuppress = vpm->curecstate[channel].nlp_max_suppress;

	memcpy(&chanconfig->EcanParametersB,
		&chanconfig->EcanParametersA,
		sizeof(chanconfig->EcanParametersB));
}

static int t1_hardware_post_init(struct t1 *wc)
{
	struct vpmadt032_options options;
	unsigned int reg;
	int res;
	int x;

	/* T1 or E1 */
	if (t1e1override > -1) {
		if (t1e1override)
			wc->spantype = TYPE_E1;
		else
			wc->spantype = TYPE_T1;
	} else {
		if (t1_getpins(wc,0) & 0x01) /* returns 1 for T1 mode */
			wc->spantype = TYPE_T1;
		else	
			wc->spantype = TYPE_E1;
	}
	debug_printk(1, "spantype: %s\n", wc->spantype==1 ? "T1" : "E1");
	
	/* what version of the FALC are we using? */
	reg = t1_setreg(wc, 0x4a, 0xaa);
	reg = t1_getreg(wc, 0x4a);
	debug_printk(1, "FALC version: %08x\n", reg);

	/* make sure reads and writes work */
	for (x = 0; x < 256; x++) {
		t1_setreg(wc, 0x14, x);
		reg = t1_getreg(wc, 0x14);
		if (reg != x)
			module_printk("Wrote '%x' but read '%x'\n", x, reg);
	}

	t1_setleds(wc, wc->ledstate);

#ifdef VPM_SUPPORT
	if (vpmsupport) {
		memset(&options, 0, sizeof(options));
		options.debug = debug;
		options.vpmnlptype = vpmnlptype;
		options.vpmnlpthresh = vpmnlpthresh;
		options.vpmnlpmaxsupp = vpmnlpmaxsupp;

		wc->vpmadt032 = vpmadt032_alloc(&options, wc->name);
		if (!wc->vpmadt032)
			return -ENOMEM;

		wc->vpmadt032->context = wc;
		wc->vpmadt032->setchanconfig_from_state = setchanconfig_from_state;
		wc->vpmadt032->span = &wc->span;

		res = vpmadt032_init(wc->vpmadt032, wc->vb);
		if (res) {
			vpmadt032_free(wc->vpmadt032);
			wc->vpmadt032=NULL;
			return -EIO;
		}

		config_vpmadt032(wc->vpmadt032);

		module_printk("VPM present and operational (Firmware version %x)\n", wc->vpmadt032->version);
		wc->ctlreg |= 0x10; /* turn on vpm (RX audio from vpm module) */
		if (vpmtsisupport) {
			debug_printk(1, "enabling VPM TSI pin\n");
			wc->ctlreg |= 0x01; /* turn on vpm timeslot interchange pin */
		}
	} else {
		module_printk("VPM Support Disabled\n");
		wc->vpmadt032 = NULL;
	}
#endif
	return 0;
}

static inline void t1_check_alarms(struct t1 *wc)
{
	unsigned char c,d;
	int alarms;
	int x,j;
	unsigned char fmr4; /* must read this always */

	if (!(test_bit(DAHDI_FLAGBIT_RUNNING, &wc->span.flags)))
		return;

	c = t1_getreg(wc, 0x4c);
	fmr4 = t1_getreg(wc, 0x20); /* must read this even if we don't use it */
	d = t1_getreg(wc, 0x4d);

	/* Assume no alarms */
	alarms = 0;

	/* And consider only carrier alarms */
	wc->span.alarms &= (DAHDI_ALARM_RED | DAHDI_ALARM_BLUE | DAHDI_ALARM_NOTOPEN);

	if (wc->spantype == TYPE_E1) {
		if (c & 0x04) {
			/* No multiframe found, force RAI high after 400ms only if
			   we haven't found a multiframe since last loss
			   of frame */
			if (!wc->flags.nmf) {
				t1_setreg_full(wc, 0x20, 0x9f | 0x20, NOT_VPM);	/* LIM0: Force RAI High */
				wc->flags.nmf = 1;
				module_printk("NMF workaround on!\n");
			}
			t1_setreg_full(wc, 0x1e, 0xc3, NOT_VPM);	/* Reset to CRC4 mode */
			t1_setreg_full(wc, 0x1c, 0xf2, NOT_VPM);	/* Force Resync */
			t1_setreg_full(wc, 0x1c, 0xf0, NOT_VPM);	/* Force Resync */
		} else if (!(c & 0x02)) {
			if (wc->flags.nmf) {
				t1_setreg_full(wc, 0x20, 0x9f, NOT_VPM);	/* LIM0: Clear forced RAI */
				wc->flags.nmf = 0;
				module_printk("NMF workaround off!\n");
			}
		}
	} else {
		/* Detect loopup code if we're not sending one */
		if ((!wc->span.mainttimer) && (d & 0x08)) {
			/* Loop-up code detected */
			if ((wc->loopupcnt++ > 80)  && (wc->span.maintstat != DAHDI_MAINT_REMOTELOOP)) {
				t1_setreg_full(wc, 0x36, 0x08, NOT_VPM);	/* LIM0: Disable any local loop */
				t1_setreg_full(wc, 0x37, 0xf6, NOT_VPM);	/* LIM1: Enable remote loop */
				wc->span.maintstat = DAHDI_MAINT_REMOTELOOP;
			}
		} else
			wc->loopupcnt = 0;
		/* Same for loopdown code */
		if ((!wc->span.mainttimer) && (d & 0x10)) {
			/* Loop-down code detected */
			if ((wc->loopdowncnt++ > 80)  && (wc->span.maintstat == DAHDI_MAINT_REMOTELOOP)) {
				t1_setreg_full(wc, 0x36, 0x08, NOT_VPM);	/* LIM0: Disable any local loop */
				t1_setreg_full(wc, 0x37, 0xf0, NOT_VPM);	/* LIM1: Disable remote loop */
				wc->span.maintstat = DAHDI_MAINT_NONE;
			}
		} else
			wc->loopdowncnt = 0;
	}

	if (wc->span.lineconfig & DAHDI_CONFIG_NOTOPEN) {
		for (x=0,j=0;x < wc->span.channels;x++)
			if ((wc->span.chans[x]->flags & DAHDI_FLAG_OPEN) ||
			    (wc->span.chans[x]->flags & DAHDI_FLAG_NETDEV))
				j++;
		if (!j)
			alarms |= DAHDI_ALARM_NOTOPEN;
	}

	if (c & 0xa0) {
		if (wc->alarmcount >= alarmdebounce) {
			if (!unchannelized)
				alarms |= DAHDI_ALARM_RED;
		} else
			wc->alarmcount++;
	} else
		wc->alarmcount = 0;
	if (c & 0x4)
		alarms |= DAHDI_ALARM_BLUE;

	/* Keep track of recovering */
	if ((!alarms) && wc->span.alarms) 
		wc->alarmtimer = jiffies + 5*HZ;
	if (wc->alarmtimer)
		alarms |= DAHDI_ALARM_RECOVER;

	/* If receiving alarms, go into Yellow alarm state */
	if (alarms && !wc->flags.sendingyellow) {
		module_printk("Setting yellow alarm\n");

		/* We manually do yellow alarm to handle RECOVER and NOTOPEN, otherwise it's auto anyway */
		t1_setreg_full(wc, 0x20, fmr4 | 0x20, NOT_VPM);
		wc->flags.sendingyellow = 1;
	} else if (!alarms && wc->flags.sendingyellow) {
		module_printk("Clearing yellow alarm\n");
		/* We manually do yellow alarm to handle RECOVER  */
		t1_setreg_full(wc, 0x20, fmr4 & ~0x20, NOT_VPM);
		wc->flags.sendingyellow = 0;
	}
	
	if ((c & 0x10) && !unchannelized)
		alarms |= DAHDI_ALARM_YELLOW;
	if (wc->span.mainttimer || wc->span.maintstat) 
		alarms |= DAHDI_ALARM_LOOPBACK;
	wc->span.alarms = alarms;
	dahdi_alarm_notify(&wc->span);
}

static void handle_leds(struct t1 *wc)
{
	unsigned char led;
	unsigned long flags;

	led = wc->ledstate;

	if (wc->span.alarms & (DAHDI_ALARM_RED | DAHDI_ALARM_BLUE)) {
		/* When we're in red alarm, blink the led once a second. */
		if (time_after(jiffies, wc->blinktimer)) {
			led = (led & __LED_GREEN) ? SET_LED_RED(led) : UNSET_LED_REDGREEN(led);
		}
	} else if (wc->span.alarms & DAHDI_ALARM_YELLOW) {
		led = (led & __LED_RED) ? SET_LED_GREEN(led) : SET_LED_RED(led);
	} else {
		if (wc->span.maintstat != DAHDI_MAINT_NONE)
			led = SET_LED_ORANGE(led);
		else
			led = UNSET_LED_ORANGE(led);

		if (test_bit(DAHDI_FLAGBIT_RUNNING, &wc->span.flags))
			led = SET_LED_GREEN(led);
		else
			led = UNSET_LED_REDGREEN(led);
	}

	if (led != wc->ledstate) {
		struct command *cmd;
		cmd = get_free_cmd(wc);
		if (cmd) {
			wc->blinktimer = jiffies + HZ/2;
			cmd->flags |= __CMD_LEDS;
			cmd->address = ~led & 0x0E;
			submit_cmd(wc, cmd);
			spin_lock_irqsave(&wc->reglock, flags);
			wc->ledstate = led;
			spin_unlock_irqrestore(&wc->reglock, flags);
		}
	}
}


static void t1_do_counters(struct t1 *wc)
{
	if (wc->alarmtimer && time_after(jiffies, wc->alarmtimer)) {
		wc->span.alarms &= ~(DAHDI_ALARM_RECOVER);
		wc->alarmtimer = 0;
		dahdi_alarm_notify(&wc->span);
	}
}

static inline void t1_transmitprep(struct t1 *wc, unsigned char* writechunk)
{
	int x;
	int y;
	int chan;

	/* Calculate Transmission */
	if (likely(test_bit(INITIALIZED, &wc->bit_flags))) {
		dahdi_transmit(&wc->span);
	}

	for (x = 0; x < DAHDI_CHUNKSIZE; x++) {
		if (likely(test_bit(INITIALIZED, &wc->bit_flags))) {
			for (chan = 0; chan < wc->span.channels; chan++)
				writechunk[(chan+1)*2] = wc->chans[chan]->writechunk[x];	
		}

		/* process the command queue */
		for (y = 0; y < 7; y++) {
			cmd_dequeue(wc, writechunk, x, y);
		}
#ifdef VPM_SUPPORT
		if(likely(wc->vpmadt032)) {
			spin_lock(&wc->reglock);
			cmd_dequeue_vpmadt032(wc, writechunk, x);
			spin_unlock(&wc->reglock);
		}
#endif

		if (x < DAHDI_CHUNKSIZE - 1) {
			writechunk[EFRAME_SIZE] = wc->ctlreg;
			writechunk[EFRAME_SIZE + 1] = wc->txident++;
		}
		writechunk += (EFRAME_SIZE + EFRAME_GAP);
	}
}

static inline void t1_receiveprep(struct t1 *wc, unsigned char* readchunk)
{
	int x,chan;
	unsigned char expected;

	for (x = 0; x < DAHDI_CHUNKSIZE; x++) {
		if (likely(test_bit(INITIALIZED, &wc->bit_flags))) {
			for (chan = 0; chan < wc->span.channels; chan++) {
				wc->chans[chan]->readchunk[x]= readchunk[(chan+1)*2];
			}
		}
		if (x < DAHDI_CHUNKSIZE - 1) {
			expected = wc->rxident+1;
			wc->rxident = readchunk[EFRAME_SIZE + 1];
			wc->statreg = readchunk[EFRAME_SIZE + 2];
			if (wc->rxident != expected) {
				wc->span.irqmisses++;
				resend_cmds(wc);
				if (unlikely(debug && test_bit(INITIALIZED, &wc->bit_flags)))
					module_printk("oops: rxident=%d expected=%d x=%d\n", wc->rxident, expected, x);
			}
		}
		cmd_decipher(wc, readchunk);
#ifdef VPM_SUPPORT
		if (wc->vpmadt032) {
			spin_lock(&wc->reglock);
			cmd_decipher_vpmadt032(wc, readchunk);
			spin_unlock(&wc->reglock);
		}
#endif
		readchunk += (EFRAME_SIZE + EFRAME_GAP);
	}
	
	/* echo cancel */
	if (likely(test_bit(INITIALIZED, &wc->bit_flags))) {
		for (x = 0; x < wc->span.channels; x++) {
			dahdi_ec_chunk(wc->chans[x], wc->chans[x]->readchunk, wc->ec_chunk2[x]);
			memcpy(wc->ec_chunk2[x],wc->ec_chunk1[x],DAHDI_CHUNKSIZE);
			memcpy(wc->ec_chunk1[x],wc->chans[x]->writechunk,DAHDI_CHUNKSIZE);
		}
		dahdi_receive(&wc->span);
	}
}

static void
t1_handle_transmit(void* vbb, void* context)
{
	struct t1* wc = context;
	memset(vbb, 0, SFRAME_SIZE);
	atomic_inc(&wc->txints);
	t1_transmitprep(wc, vbb);
	voicebus_transmit(wc->vb, vbb);
	handle_leds(wc);
}

static void
t1_handle_receive(void* vbb, void* context)
{
	struct t1* wc = context;
	t1_receiveprep(wc, vbb);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
static void timer_work_func(void *param)
{
	struct t1 *wc = param;
#else
static void timer_work_func(struct work_struct *work)
{
	struct t1 *wc = container_of(work, struct t1, timer_work);
#endif
	/* Called once every 100ms */
	if (unlikely(!test_bit(INITIALIZED, &wc->bit_flags)))
		return;
	t1_do_counters(wc);
	t1_check_alarms(wc);
	t1_check_sigbits(wc);
	mod_timer(&wc->timer, jiffies + HZ/5);
}

static void
te12xp_timer(unsigned long data)
{
	struct t1 *wc = (struct t1 *)data;
	schedule_work(&wc->timer_work);
	return;
}

static int __devinit te12xp_init_one(struct pci_dev *pdev, const struct pci_device_id *ent)
{
	struct t1 *wc;
	struct t1_desc *d = (struct t1_desc *) ent->driver_data;
	unsigned int x;
	int res;
	int startinglatency;
	unsigned int index = -1;

	for (x = 0; x < sizeof(ifaces) / sizeof(ifaces[0]); x++) {
		if (!ifaces[x]) {
			index = x;
			break;
		}
	}

	if (-1 == index) {
		module_printk("Too many interfaces\n");
		return -EIO;
	}
	
retry:
	if (!(wc = kmalloc(sizeof(*wc), GFP_KERNEL))) {
		return -ENOMEM;
	}

	ifaces[index] = wc;
	memset(wc, 0, sizeof(*wc));
	wc->ledstate = -1;
	spin_lock_init(&wc->reglock);
	spin_lock_init(&wc->cmd_list_lock);
	INIT_LIST_HEAD(&wc->active_cmds);
	INIT_LIST_HEAD(&wc->pending_cmds);

	wc->variety = d->name;
	wc->txident = 1;

#	if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 18)
	wc->timer.function = te12xp_timer;
	wc->timer.data = (unsigned long)wc;
	init_timer(&wc->timer);
#	else
	setup_timer(&wc->timer, te12xp_timer, (unsigned long)wc);
#	endif

#	if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
	INIT_WORK(&wc->timer_work, timer_work_func, wc);
#	else
	INIT_WORK(&wc->timer_work, timer_work_func);
#	endif

	snprintf(wc->name, sizeof(wc->name)-1, "wcte12xp%d", index);
	if ((res = voicebus_init(pdev, SFRAME_SIZE, wc->name,
				 t1_handle_receive, t1_handle_transmit, wc,
				 debug, &wc->vb))) {
		WARN_ON(1);
		free_wc(wc);
		ifaces[index] = NULL;
		return res;
	}
	
	/* Keep track of which device we are */
	pci_set_drvdata(pdev, wc);
	if (VOICEBUS_DEFAULT_LATENCY != latency) {
		voicebus_set_minlatency(wc->vb, latency);
	}
	voicebus_start(wc->vb);
	startinglatency = voicebus_current_latency(wc->vb);
	t1_hardware_post_init(wc);

	for (x = 0; x < (wc->spantype == TYPE_E1 ? 31 : 24); x++) {
		if (!(wc->chans[x] = kmalloc(sizeof(*wc->chans[x]), GFP_KERNEL))) {
			free_wc(wc);
			ifaces[index] = NULL;
			return -ENOMEM;
		}
		memset(wc->chans[x], 0, sizeof(*wc->chans[x]));
		if (!(wc->ec[x] = kmalloc(sizeof(*wc->ec[x]), GFP_KERNEL))) {
			free_wc(wc);
			ifaces[index] = NULL;
			return -ENOMEM;
		}
		memset(wc->ec[x], 0, sizeof(*wc->ec[x]));
	}

	mod_timer(&wc->timer, jiffies + HZ/5);
	t1_software_init(wc);
	if (voicebus_current_latency(wc->vb) > startinglatency) {
		/* The voicebus library increased the latency during
		 * initialization because the host wasn't able to service the
		 * interrupts from the adapter quickly enough.  In this case,
		 * we'll increase our latency and restart the initialization.
		 */
		printk(KERN_NOTICE "%s: Restarting board initialization " \
		 "after increasing latency.\n", wc->name);
		latency = voicebus_current_latency(wc->vb);
		dahdi_unregister(&wc->span);
		voicebus_release(wc->vb);
		wc->vb = NULL;
		free_wc(wc);
		wc = NULL;
		goto retry;
	}

	module_printk("Found a %s\n", wc->variety);

	return 0;
}

static void __devexit te12xp_remove_one(struct pci_dev *pdev)
{
	struct t1 *wc = pci_get_drvdata(pdev);
#ifdef VPM_SUPPORT
	unsigned long flags;
	struct vpmadt032 *vpm = wc->vpmadt032;
#endif
	if (!wc)
		return;

#ifdef VPM_SUPPORT
	if(vpm) {
		wc->vpmadt032 = NULL;
		clear_bit(VPM150M_DTMFDETECT, &vpm->control);
		clear_bit(VPM150M_ACTIVE, &vpm->control);
	}
#endif
	clear_bit(INITIALIZED, &wc->bit_flags);
	del_timer_sync(&wc->timer);
	flush_scheduled_work();
	del_timer_sync(&wc->timer);

	BUG_ON(!wc->vb);
	voicebus_release(wc->vb);
	wc->vb = NULL;

#ifdef VPM_SUPPORT
	if(vpm) {
		spin_lock_irqsave(&wc->reglock, flags);
		spin_unlock_irqrestore(&wc->reglock, flags);
		vpmadt032_free(vpm);
	}
#endif
	t1_release(wc);
}

static struct pci_device_id te12xp_pci_tbl[] = {
	{ 0xd161, 0x0120, PCI_ANY_ID, PCI_ANY_ID, 0, 0, (unsigned long) &te120p},
	{ 0xd161, 0x8000, PCI_ANY_ID, PCI_ANY_ID, 0, 0, (unsigned long) &te121},
	{ 0xd161, 0x8001, PCI_ANY_ID, PCI_ANY_ID, 0, 0, (unsigned long) &te122},
	{ 0 }
};

MODULE_DEVICE_TABLE(pci, te12xp_pci_tbl);

struct pci_driver te12xp_driver = {
	.name = "wcte12xp",
	.probe = te12xp_init_one,
	.remove = __devexit_p(te12xp_remove_one),
	.id_table = te12xp_pci_tbl,
};

static int __init te12xp_init(void)
{
	int res;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 23)
	cmd_cache = kmem_cache_create(THIS_MODULE->name, sizeof(struct command), 0,
#if LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 22)
				SLAB_HWCACHE_ALIGN | SLAB_STORE_USER, NULL, NULL);
#else
				SLAB_HWCACHE_ALIGN, NULL, NULL);
#endif
#else
	cmd_cache = kmem_cache_create(THIS_MODULE->name, sizeof(struct command), 0,
				SLAB_HWCACHE_ALIGN, NULL);
#endif
	if (!cmd_cache)
		return -ENOMEM;

	res = dahdi_pci_module(&te12xp_driver);
	if (res) {
		kmem_cache_destroy(cmd_cache);
		return -ENODEV;
	}

	return 0;
}


static void __exit te12xp_cleanup(void)
{
	pci_unregister_driver(&te12xp_driver);
	kmem_cache_destroy(cmd_cache);
}

module_param(debug, int, S_IRUGO | S_IWUSR);
module_param(loopback, int, S_IRUGO | S_IWUSR);
module_param(t1e1override, int, S_IRUGO | S_IWUSR);
module_param(j1mode, int, S_IRUGO | S_IWUSR);
module_param(alarmdebounce, int, S_IRUGO | S_IWUSR);
module_param(latency, int, S_IRUGO | S_IWUSR);
#ifdef VPM_SUPPORT
module_param(vpmsupport, int, S_IRUGO | S_IWUSR);
module_param(vpmtsisupport, int, S_IRUGO | S_IWUSR);
module_param(vpmnlptype, int, S_IRUGO);
module_param(vpmnlpthresh, int, S_IRUGO);
module_param(vpmnlpmaxsupp, int, S_IRUGO);
#endif

MODULE_DESCRIPTION("Wildcard VoiceBus Digital Card Driver");
MODULE_AUTHOR("Digium Incorporated <support@digium.com>");
MODULE_LICENSE("GPL v2");

module_init(te12xp_init);
module_exit(te12xp_cleanup);
