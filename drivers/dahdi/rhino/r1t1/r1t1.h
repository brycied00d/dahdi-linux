/*
 * Rhino Equipment Corp.  Rhino R1T1 Card Driver
 *
 * Release version 10/10/10
 *
 * Written by
 *          Lee Reeves <helpdesk@rhinoequipment.com>
 *          Bob Conklin <helpdesk@rhinoequipment.com>
 *          Bryce Chidester <helpdesk@rhinoequipment.com>
 *          Matthew Gessner <helpdesk@rhinoequipment.com>
 *
 * Based on Digium's wct1xxp module
 * and Zapata Telephony's Zaptel Telephony Interface
 *
 * Copyright (C) 2005-2010, Rhino Equipment Corp.
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
#ifndef _R1T1_H
#define _R1T1_H

#include <dahdi/version.h>

#include <dahdi/kernel.h>
#include <dahdi/user.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>

#define PCI_VENDOR_RHINO 0xb0b
#define PCI_DEVICE_R1T1 0x0105

#define RH_MAX_CARDS    32

#define R1T1_SIZE 0x820

#define E1_DRIVER  1
#define SUPPORT_E1 1

#define DELAY   0x0				/* 30 = 15 cycles, 10 = 8 cycles, 0 = 3 cycles */

#define RH_CNTL     (0x00 << 2)	/* loop back, led Al, led A0 */
#define RH_OPER     (0x01 << 2)	/* buff length, int ack, enable */
#define RH_AUXC     (0x02 << 2)
#define RH_AUXD     (0x03 << 2)
#define RH_MASK0    (0x04 << 2)
#define RH_MASK1    (0x05 << 2)
#define RH_INTSTAT  (0x06 << 2)

#define RH_DMAWS    (0x08 << 2)
#define RH_DMAWI    (0x0c << 2)
#define RH_DMAWE    (0x10 << 2)
#define RH_DMARS    (0x18 << 2)
#define RH_DMARI    (0x1c << 2)
#define RH_DMARE    (0x20 << 2)
#define RH_CURPOS   (0x24 << 2)

#define RH_SERC     (0x2d << 2)
#define RH_FSCDELAY (0x2f << 2)

#define RH_CLOCK    0x0
#define RH_LEDTEST  0x1
#define RH_VERSION  0x2

#define R1T1_STATE      0x800
#define R1T1_VERSION    0x802
#define R1T1_CONTROL    0x804
#define R1T1_BUFLEN     0x806
#define R1T1_RXBUFSTART 0x808
#define R1T1_TXBUFSTART 0x80C

/* Offset between transmit and receive - I'm not sure what this is actually used for */
#define RH_OFFSET   4

#define BIT_CS      (1 << 7)
#define BIT_ADDR    (0xf << 3)

#define BIT_LED0    (1 << 0)
#define BIT_LED1    (1 << 1)
#define BIT_TEST    (1 << 2)

#define NORM_OP 0x90
#define YEL_ALM 0xa0
#define NO_CARR 0x70
#define NO_SYNC 0x80
#define RECOVER 0xe0

#define DS2155_MSTRREG 0x00
#define DS2155_IOCR1 0x01
#define DS2155_IOCR2 0x02
#define DS2155_T1RCR1 0x03
#define DS2155_T1RCR2 0x04
#define DS2155_T1TCR1 0x05
#define DS2155_T1TCR2 0x06
#define DS2155_T1CCR1 0x07
#define DS2155_SSIE 0x08
#define DS2155_SR2 0x18
#define DS2155_SR3 0x1A
#define DS2155_E1RCR1 0x33
#define DS2155_E1RCR2 0x34
#define DS2155_E1TCR1 0x35
#define DS2155_E1TCR2 0x36
#define DS2155_SIGCR 0x40
#define DS2155_LBCR 0x4a
#define DS2155_ESCR 0x4f
#define DS2155_TSR1 0x50
#define DS2155_CCR1 0x70
#define DS2155_CCR2 0x71
#define DS2155_CCR3 0x72
#define DS2155_CCR4 0x73
#define DS2155_LIC1 0x78
#define DS2155_LIC2 0x79
#define DS2155_LIC4 0x7b
#define DS2155_TLBC 0x7d
#define DS2155_IBCC 0xb6
#define DS2155_TCD1 0xb7
#define DS2155_TCD2 0xb8
#define DS2155_RUPCD1 0xb9
#define DS2155_RUPCD2 0xba
#define DS2155_RDNCD1 0xbb
#define DS2155_RDNCD2 0xbc
#define DS2155_TAF 0xd0
#define DS2155_TNAF 0xd1
#define DS2155_TFDL 0xc1
#define DS2155_IBOC 0xc5

#define TARG_REGS       0x200

#define R1T1_HPIC       0x7		/* 0x81c */

#define R1T1_ECA1       0x8
#define R1T1_ECB1       0x9
#define R1T1_XLATE_EN   0xa		/* 0x828 */

#define HPI_SEL         1 << 4
#define EC_ON           1 << 5
#define DSP_RST         1 << 6
#define XLATE           1 << 7

#define R1T1_DSP_HPIC     0x10	/* 0x840 */
#define R1T1_HPIDAI       0x11	/* 0x844 */
#define R1T1_HPIA         0x12	/* 0x848 */
#define R1T1_HPID         0x13	/* 0x84C */

#define R1T1_HPIRD        0x14	/* 0x850 */
#define R1T1_HPIRD_STAT   0x14	/* 0x852 */

#define R1T1_HRDY         1 << 0
#define R1T1_BUSY_N       2 << 1
#define R1T1_D_NEW        4 << 2

#define R1T1_HPIRDX       0x15	/* 0x854 */
#define R1T1_HPIRDX_STAT  0x15	/* 0x856 */

#define R1T1_HCS_REG      0x16

#define R1T1_BL_GO      1 << 0
#define R1T1_XADD       1 << 5

#define DSP_5510 2


struct r1t1 {
	struct pci_dev *dev;
	spinlock_t lock;
	int ise1;
	int num;
	int version;
	/* Our offset for finding channel 1 */
	int offset;
	char *variety;
	unsigned int intcount;
	int usecount;
	int clocktimeout;
	int sync;
	int dead;
	int blinktimer;
	int alarmtimer;
	int loopupcnt;
	int loopdowncnt;
	int miss;
	int misslast;
	int dsp_up;
	int hpi_fast;
	int hpi_xadd;
	int dsp_sel;
	int dsp_type;
	int *chanmap;
	unsigned int nextec;
	unsigned int currec;

	wait_queue_head_t regq;

	struct workqueue_struct *wq;
	struct work_struct work;

	unsigned char ledtestreg;
	unsigned char outbyte;
	unsigned long pciaddr;
	void *ioaddr;
	volatile unsigned int *membase;	/* Base address of card */
	unsigned short canary;
	/* T1 signalling */
	unsigned char txsig[12];
	dma_addr_t readdma;
	dma_addr_t writedma;
	volatile unsigned char *writechunk;	/* Double-word aligned write memory */
	volatile unsigned char *readchunk;	/* Double-word aligned read memory */
	unsigned char ec_chunk1[31][DAHDI_CHUNKSIZE];
	unsigned char ec_chunk2[31][DAHDI_CHUNKSIZE];
	int nextbuf;
	struct dahdi_span span;		/* Span */
	struct dahdi_chan *chans[31];	/* Channels */
	struct dahdi_echocan_state *ec[31];	/* echocan state for each channel */
};

extern unsigned int __r1t1_card_dsp_in(struct r1t1 *rh, const unsigned int addr);
extern void __r1t1_card_dsp_out(struct r1t1 *rh, const unsigned int addr,
								const unsigned int value);

extern unsigned int __r1t1_card_pci_in(struct r1t1 *rh, const unsigned int addr);
extern void __r1t1_card_pci_out(struct r1t1 *rh, const unsigned int addr,
								const unsigned int value);

#endif /* _R1T1_H */
