/*
 * Rhino RXT1 DUAL - QUAD T1 E1 J1 Interface Driver
 *
 * Release version 10/10/10
 *
 * Written by
 *          Bob Conklin <helpdesk@rhinoequipment.com>
 *          Bryce Chidester <helpdesk@rhinoequipment.com>
 *          Matthew Gessner <helpdesk@rhinoequipment.com>
 *
 * Based on Digium's TE410P Quad-T1/E1 PCI Driver
 * Based on previous works, designs, and archetectures conceived and
 * written by Jim Dixon <jim@lambdatel.com>,
 * and Mark Spencer <markster@digium.com>
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
#ifndef _RXT1_H
#define _RXT1_H

#include <dahdi/version.h>

#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/sched.h>

#include <dahdi/kernel.h>
#include <dahdi/user.h>

#define addr_t (__u32)(dma_addr_t)

#define FRMR_TTR_BASE 0x10
#define FRMR_RTR_BASE 0x0c
#define FRMR_TSEO 0xa0
#define FRMR_TSBS1 0xa1
#define FRMR_CCR1 0x09
#define FRMR_CCR1_ITF 0x08
#define FRMR_CCR1_EITS 0x10
#define FRMR_CCR2 0x0a
#define FRMR_CCR2_RCRC 0x04
#define FRMR_CCR2_RADD 0x10
#define FRMR_MODE 0x03
#define FRMR_MODE_NO_ADDR_CMP 0x80
#define FRMR_MODE_SS7 0x20
#define FRMR_MODE_HRAC 0x08
#define FRMR_IMR0 0x14
#define FRMR_IMR0_RME 0x80
#define FRMR_IMR0_RPF 0x01
#define FRMR_IMR1 0x15
#define FRMR_IMR1_ALLS 0x20
#define FRMR_IMR1_XDU 0x10
#define FRMR_IMR1_XPR 0x01

#define FRMR_FMR2 0x1e
#define FRMR_XSP 0x21

#define FRMR_XC0 0x22
#define FRMR_XC1 0x23
#define FRMR_RC0 0x24
#define FRMR_RC1 0x25
#define FRMR_IDLE 0x2b

#define FRMR_ICB1 0x32
#define FRMR_ICB2 0x33
#define FRMR_ICB3 0x34
#define FRMR_ICB4 0x35
#define FRMR_LIM0 0x36
#define FRMR_LIM1 0x37

#define FRMR_SIC1 0x3e
#define FRMR_SIC2 0x3f
#define FRMR_SIC3 0x40
#define FRMR_CMR1 0x44
#define FRMR_CMR2 0x45
#define FRMR_GCR 0x46

#define FRMR_FRS0 0x4c

#define FRMR_FRS0_LOS 0x80		/* loss of signal */
#define FRMR_FRS0_AIS 0x40		/* Blue alarma pattern */
#define FRMR_FRS0_LFA 0x20		/* loss of frame */
#define FRMR_FRS0_RRA 0x10		/* recieve remote alarm */

#define FRMR_FRS0_NMF 0x04
#define FRMR_FRS0_LMFA 0x02		/* loss of multiframe */
#define FRMR_FRS0_FSRF 0x01		/* frame search restart */

#define FRMR_FRS1 0x4d

#define FRMR_FRS1_EXZD    0x80	/* excessive zeros */
#define FRMR_FRS1_PDEN    0x40	/* T1 ** pulse density violation */
#define FRMR_FRS1_LLBDD   0x10	/* loop down code recieved */
#define FRMR_FRS1_LLBAD   0x08	/* loop up code recieved */
#define FRMR_FRS1_XLS     0x02	/* Line shorted -- over current */
#define FRMR_FRS1_XLO     0x01	/* Line open  -- zero current */

#define FRMR_FRS1_TS16RA  0x40	/* E1 ** */
#define FRMR_FRS1_TS16LOS 0x20	/* E1 ** */
#define FRMR_FRS1_TS16AIS 0x10	/* E1 ** */
#define FRMR_FRS1_TS16LFA 0x08	/* E1 ** */

#define FRMR_ISR0 0x68
#define FRMR_ISR0_RME 0x80
#define FRMR_ISR0_RPF 0x01
#define FRMR_ISR1 0x69
#define FRMR_ISR1_ALLS 0x20
#define FRMR_ISR1_XDU 0x10
#define FRMR_ISR1_XPR 0x01
#define FRMR_ISR2 0x6a
#define FRMR_ISR3 0x6b
#define FRMR_ISR4 0x6c
#define FRMR_ISR5 0x6d
#define FRMR_ISR6 0xac
#define FRMR_ISR7 0xd8
#define FRMR_GIS  0x6e
#define FRMR_GIS_ISR0 0x01
#define FRMR_GIS_ISR1 0x02
#define FRMR_GIS_ISR2 0x04
#define FRMR_GIS_ISR3 0x08
#define FRMR_GIS_ISR4 0x10
#define FRMR_GIS_ISR5 0x20
#define FRMR_GIS_ISR6 0x40
#define FRMR_GIS_ISR7 0x80
#define FRMR_CIS 0x6f
#define FRMR_CIS_GIS1 0x01
#define FRMR_CIS_GIS2 0x02
#define FRMR_CIS_GIS3 0x04
#define FRMR_CIS_GIS4 0x08
#define FRMR_CMDR 0x02
#define FRMR_CMDR_SRES 0x01
#define FRMR_CMDR_XRES 0x10
#define FRMR_CMDR_RMC 0x80
#define FRMR_CMDR_XTF 0x04
#define FRMR_CMDR_XHF 0x08
#define FRMR_CMDR_XME 0x02
#define FRMR_RSIS 0x65
#define FRMR_RSIS_VFR 0x80
#define FRMR_RSIS_RDO 0x40
#define FRMR_RSIS_CRC16 0x20
#define FRMR_RSIS_RAB 0x10
#define FRMR_RBCL 0x66
#define FRMR_RBCL_MAX_SIZE 0x1f
#define FRMR_RBCH 0x67
#define FRMR_RXFIFO 0x00
#define FRMR_SIS 0x64
#define FRMR_SIS_XFW 0x40
#define FRMR_TXFIFO 0x00

#define FRMR_PC1 0x80
#define FRMR_PC2 0x81
#define FRMR_PC3 0x82
#define FRMR_PC4 0x83
#define FRMR_PC5 0x84

#define NUM_REGS 0xa9
#define NUM_PCI 5


struct rxt1_regs {
	unsigned int pci[NUM_PCI];
	unsigned char regs[NUM_REGS];
};

#define RXT1_GET_REGS   _IOW (DAHDI_CODE, 60, struct rxt1_regs)

#define RXT1_REGSTARTB  0x1000
#define RXT1_REGSTARTW  0x400
#define TARG_REGS       0x400
#define RXT1_VERSION    0x0		/* 0x1000 */
#define RH_VERMASK      0xffff << 16
#define RXT1_DMA        0x1		/* 0x1004 */
#define DMA_GO          1 << 0
#define DMA_ACK         1 << 1
#define DMA_IMSK        1 << 2
#define FRMR_IEN        1 << 3
#define FRMR_ISTAT      1 << 4
#define FRMR_ACK        1 << 5
#define FRMR_IMSK       1 << 6
#define BUFF_PTR        1 << 8
#define DMA_INT         1 << 9
#define FRAME_MA        1 << 10
#define DMA_OVFL        1 << 11
#define OVFL_FLGS       0xf << 12
#define DMA_LEN         0xffff << 16
#define RXT1_RXBUFSTART 0x2		/* 0x1008 */
#define RXT1_TXBUFSTART 0x3		/* 0x100C */
#define RXT1_STAT       0x4		/* 0x1010 */

#define RXT1_HPIC       0x7		/* 0x101c */

#define RXT1_ECA1       0x8		/* 0x1020 */
#define RXT1_ECB1       0x9		/* 0x1024 */
#define RXT1_ECA2       0xa		/* 0x1028 */
#define RXT1_ECB2       0xb		/* 0x102C */
#define RXT1_ECA3       0xc		/* 0x1030 */
#define RXT1_ECB3       0xd		/* 0x1034 */
#define RXT1_ECA4       0xe		/* 0x1038 */
#define RXT1_ECB4       0xf		/* 0x103c */

#define HPI_SEL         1 << 4
#define EC_ON           1 << 5
#define DSP_RST         1 << 6

#define RXT1_DSP_HPIC     0x10	/* 0x1040 */
#define RXT1_HPIDAI       0x11	/* 0x1044 */
#define RXT1_HPIA         0x12	/* 0x1048 */
#define RXT1_HPID         0x13	/* 0x104C */

#define RXT1_HPIRD        0x14	/* 0x1050 */
#define RXT1_HPIRD_STAT   0x14	/* 0x1052 */

#define RXT1_HRDY         1 << 0
#define RXT1_BUSY_N       2 << 1
#define RXT1_D_NEW        4 << 2

#define RXT1_HPIRDX       0x15	/* 0x1054 */
#define RXT1_HPIRDX_STAT  0x15	/* 0x1056 */

#define RXT1_HCS_REG      0x16	/* 0x1058 */

#define RXT1_XLATE1       0x17	/* 0x105c */
#define RXT1_XLATE2       0x18	/* 0x1060 */
#define RXT1_XLATE3       0x19	/* 0x1064 */
#define RXT1_XLATE4       0x1a	/* 0x1068 */

#define RXT1_BL_GO      1 << 0
#define RXT1_XADD       1 << 5

#define JTAG_TDI        1 << 0
#define JTAG_TCK        1 << 1
#define JTAG_TMS        1 << 2
#define JTAG_ENS        7 << 3	/* 3,4,5 */
#define JTAG_TDO        1 << 6
#define STATINIT        0x00777710
#define FRMR1_STAT      0xf << 8	/* << (8 + span * 4) */
#define FRMR2_STAT      0xf << 12
#define FRMR3_STAT      0xf << 16
#define FRMR4_STAT      0xf << 20

#define NORM_OP         0x9
#define YEL_ALM         0xa
#define NO_CARR         0x7
#define NO_SYNC         0x8
#define RECOVER         0xe
#define RLOOP           0xd
#define SPAN_OFF        0xc

#define RXT1_SIZE       0x14
#define DSP_5510        1

#define MAX_RXT1_CARDS 64

struct rxt1_card_t;

struct rxt1_span_t {
	struct rxt1_card_t *owner;
	unsigned int *writechunk;	/* Double-word aligned write memory */
	unsigned int *readchunk;	/* Double-word aligned read memory */
	int spantype;				/* card type, T1 or E1 or J1 */
	int sync;
	int psync;
	int alarmtimer;
	int redalarms;
	int notclear;
	int alarmcount;
	int dsp_up;
	int spanflags;
	int syncpos;
	int e1check;				/* E1 check */
	struct dahdi_span span;
	unsigned char txsigs[16];	/* Transmit sigs */
	int loopupcnt;
	int loopdowncnt;
	unsigned int ec_chan_id[31];
	unsigned char ec_chunk1[31][DAHDI_CHUNKSIZE];	/* first EC chunk buffer */
	unsigned char ec_chunk2[31][DAHDI_CHUNKSIZE];	/* second EC chunk buffer */
	int irqmisses;

	/* HDLC controller fields */
	struct dahdi_chan *sigchan;
	unsigned char sigmode;
	int sigactive;
	int frames_out;
	int frames_in;

#ifdef ENABLE_WORKQUEUES
	struct work_struct swork;
#endif
	struct dahdi_chan *chans[31];	/* Individual channels */
	struct dahdi_echocan_state *ec[31];	/* echocan state for each channel */
};

struct rxt1_card_t {
	/* This structure exists one per card */
	struct pci_dev *dev;		/* Pointer to PCI device */
	unsigned int intcount;
	int num;					/* Which card we are */
	int t1e1;					/* T1/E1 select pins */
	int globalconfig;			/* Whether global setup has been done */
	int syncsrc;				/* active sync source */
	struct rxt1_span_t *rxt1_spans[4];	/* Individual spans */
	int numspans;				/* Number of spans on the card */
	int hpi_fast;
	int hpi_xadd[4];
	int dsp_sel;
	int dsp_type;
	struct workqueue_struct *dspwq;
	struct work_struct dspwork;
	unsigned int nextec[4];
	unsigned int currec[4];
	int blinktimer;
	int irq;					/* IRQ used by device */
	int order;					/* Order */
	int flags;					/* Device flags */
	int master;					/* Are we master */
	int ledreg;					/* LED Register */
	int version;				/* LED Register */
	unsigned int gpio;
	unsigned int gpioctl;
	int stopdma;				/* Set to stop DMA */
	unsigned int dmactrl;
	int e1recover;				/* E1 recovery timer */
	dma_addr_t readdma;
	dma_addr_t writedma;
	unsigned long memaddr;		/* Base address of card */
	unsigned long memlen;
	volatile unsigned int *membase;	/* Base address of card */
	int spansstarted;			/* number of spans started */
	spinlock_t reglock;			/* lock register access */
	volatile unsigned int *writechunk;	/* Double-word aligned write memory */
	volatile unsigned int *readchunk;	/* Double-word aligned read memory */
	unsigned short canary;
#ifdef ENABLE_WORKQUEUES
	atomic_t worklist;
	struct workqueue_struct *workq;
#else
#ifdef ENABLE_TASKLETS
	int taskletrun;
	int taskletsched;
	int taskletpending;
	int taskletexec;
	int txerrors;
	struct tasklet_struct rxt1_tlet;
#endif
#endif
	unsigned int passno;		/* number of interrupt passes */
	char *variety;
	int nextbuf;
	int last_jiffie;
	int last0;					/* for detecting double-missed IRQ */
	int checktiming;			/* Set >0 to cause the timing source to be checked */
};

extern unsigned int __rxt1_card_dsp_in(struct rxt1_card_t *rxt1_card,
									   const unsigned int addr);
extern void __rxt1_card_dsp_out(struct rxt1_card_t *rxt1_card, const unsigned int addr,
								const unsigned int value);

extern unsigned int __rxt1_card_pci_in(struct rxt1_card_t *rxt1_card,
									   const unsigned int addr);
extern void __rxt1_card_pci_out(struct rxt1_card_t *rxt1_card, const unsigned int addr,
								const unsigned int value, const unsigned int mask);

#endif /* _RXT1_H */
