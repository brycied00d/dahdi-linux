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

#ifndef _RCBFX_H
#define _RCBFX_H

#include <dahdi/version.h>

#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <asm/io.h>

#include <dahdi/kernel.h>
#include <dahdi/user.h>

#define NUM_FXO_REGS 60

#define RH_MAX_IFACES 128
#define PCI_VENDOR_RHINO     0xb0b
#define PCI_DEVICE_RCB8FXX   0x406
#define PCI_DEVICE_RCB24FXS  0x506
#define PCI_DEVICE_RCB24FXX  0x906
#define PCI_DEVICE_RCB24FXO  0x706
#define PCI_DEVICE_RCB4FXO   0x206

#define RCB_STATE        0x800
#define RCB_VERSION      0x802
#define RCB_CONTROL      0x804
#define RCB_INTSTAT      0x805
#define RCB_BUFLEN       0x806
#define RCB_RXBUFSTART   0x808
#define RCB_TXBUFSTART   0x80C
#define RCB_JTAG         0x810
#define RCB_TXSIG0       0x820
#define RCB_RXSIG0       0x830
#define RCB_RXSIGSTAT    0x83C
#define RCB_TXSIGSTAT    0x82C
#define RCB_CHANPRES     0x840
#define RCB_CHANTYPE     0x844
#define RH_SPI_DIV       0x811
#define RH_OLDSPI        0x814
#define RH_SPI           0x814
#define RH_CSR           0x815
#define SPI_WR           0x818
#define SPI_RD           0x81C
#define FW_DATA          0x854
#define FW_COMOUT        0x855
#define FW_VER           0x856
#define FW_COMIN         0x858
#define FW_BOOT          0x859
#define FW_BOOT_LOAD     2
#define FW_BOOT_RST      1
#define DSP_RST          4
#define FW_SUM           0x85A
#define EC_CNTL          0x85B
#define EC_ON            1
#define RCB_STATOUT      0x854
#define RCB_TDM_PTR      0x85C
#define RCB_HPIC         0x860
#define RCB_XADD         (1 << 5)
#define RCB_BL_GO        (1 << 0)
#define RCB_HPIDAI       0x864
#define RCB_HPIA         0x868
#define RCB_HPID         0x86C

#define RCB_HPIRD        0x870
#define RCB_HPIRD_STAT   0x872

#define RCB_HRDY         1
#define RCB_BUSY_N       2
#define RCB_D_NEW        4

#define RCB_HPIRDX       0x874
#define RCB_HPIRDX_STAT  0x876

#define RCB_EC_ENA       0x878
#define RCB_EC_ENB       0x87C

#define RCB_LVSSAMP      0x3ff	/* 1.024 sec */
#define RCB_LVSTAB       0x880
#define RCB_REGTAB       0x898
#define RCB_REGADDR      0x8b1
#define RCB_BATTTIME     0x8b0
#define RCB_REGTIME      1500	/* 1.5 sec */

#define RCB_SIZE         0x8b4

#define DSP_ENTRY_ADDR_LO 0x0061
#define DSP_ENTRY_ADDR_HI 0x0060

#define RH_SYNC     0x0
#define RH_TEST     0x1
#define RH_CS       0x2
#define RH_VER      0x3

#define BIT_CS      (1 << 2)
#define BIT_SCLK    (1 << 3)
#define BIT_SDI     (1 << 4)
#define BIT_SDO     (1 << 5)
#define BIT_RST     (1 << 6)
#define BIT_DMAGO   (1 << 0)
#define INT_ACK     (1 << 1)

#define DEBUG_MAIN  (1 << 0)
#define DEBUG_INTS  (1 << 1)
#define DEBUG_DSP   (1 << 2)
#define DEBUG_SIG   (1 << 3)

#define STOP_DMA    (1 << 0)
#define FREE_DMA    (1 << 1)
#define IOUNMAP     (1 << 2)
#define ZUNREG      (1 << 3)
#define FREE_INT    (1 << 4)
#define RH_KFREE    (1 << 5)
#define PCI_FREE    (1 << 6)
#define RST_SLAB    (1 << 7)

#define FLAG_EMPTY  0
#define FLAG_WRITE  1
#define FLAG_READ   2

/* the constants below control the 'debounce' periods enforced by the
   check_hook routines; these routines are called once every 4 interrupts
   (the interrupt cycles around the four modules), so the periods are
   specified in _4 millisecond_ increments
*/
#define RING_DEBOUNCE       4	/* Ringer Debounce (64 ms) */
#define DEFAULT_BATT_DEBOUNCE   4	/* 4 = Battery debounce (64 ms) */
#define POLARITY_DEBOUNCE   8	/* 4 = Polarity debounce (64 ms) */
#define DEFAULT_BATT_THRESH 3	/* Anything under this is "no battery" */

#define OHT_TIMER 6000			/* How long after RING to retain OHT */

#define MAX_CHANS 32

#define MAX_ALARMS 10

#define MOD_TYPE_UNK    0
#define MOD_TYPE_FXS    1
#define MOD_TYPE_FXO    2

#define DSP_NONE    0
#define DSP_5507    1
#define DSP_5510    2

#define PCMEN    (1 << 4)
#define ALAW     0x00
#define ULAW     0x01
#define LCR      (1 << 0)
#define CAL      (1 << 7)

#define RCB_CHAN_REG 72


struct rcb_card_t {
	struct pci_dev *dev;
	struct dahdi_span span;
	unsigned char ios;
	int usecount;
	unsigned int intcount;
	unsigned int tdm_ptr;
	unsigned int nextbuf;
	long unsigned int last_jiffie;
	long unsigned int buf_int_diff;
	int dead;
	int pos;
	int freeregion;
	int alt;
	int hpi_fast;
	int hpi_xadd;
	int dsp_up;
	int dsp_type;
	int curcard;
	int oldreg_addr;
	int read_on_int;
	int num_chans;
	int num_slots;
	int chanflag;				/* Bit-map of present cards */
	spinlock_t lock;

	/* Receive hook state and debouncing */
	int modtype[MAX_CHANS];
	int fxs_chanmap;
	int fxo_chanmap;
	int chans_configed;
	int param_upd_state;
	int comerr[MAX_CHANS];
	int retry[MAX_CHANS];
	int lastcomerr[MAX_CHANS];
	long unsigned int baseaddr;
	int currec;
	int nextec;
	wait_queue_head_t regq;
	struct workqueue_struct *wq;
	struct work_struct work;
	int memlen;
	int hw_ver_min;
	void *memaddr;
	dma_addr_t readdma;
	dma_addr_t writedma;
	volatile unsigned char *writechunk;	/* Double-word aligned write memory */
	volatile unsigned char *readchunk;	/* Double-word aligned read memory */
	struct dahdi_chan *chans[MAX_CHANS];
	char *variety;
	struct dahdi_echocan_state *ec[MAX_CHANS];	/* echocan state for each channel */
};

#endif /* _RCBFX_H */
