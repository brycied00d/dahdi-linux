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
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <asm/io.h>
#include <linux/workqueue.h>
#ifdef LINUX26
#include <linux/moduleparam.h>
#ifdef HOTPLUG_FIRMWARE
#ifndef CONFIG_FW_LOADER
#undef HOTPLUG_FIRMWARE
#else
#include <linux/firmware.h>
#endif
#endif
#endif
#include "rxt1.h"
#include "GpakCust.h"
#include "GpakApi.h"

/*
 * Tasklets provide better system interactive response at the cost of the
 * possibility of losing a frame of data at very infrequent intervals.  If
 * you are more concerned with the performance of your machine, enable the
 * tasklets.  If you are strict about absolutely no drops, then do not enable
 * tasklets.
 */

/* #define ENABLE_TASKLETS */


/* Work queues are a way to better distribute load on SMP systems */
#if defined(LINUX26) && (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20))
/*
 * Work queues can significantly improve performance and scalability
 * on multi-processor machines, but requires bypassing some kernel
 * API's, so it's not guaranteed to be compatible with all kernels.
 */
/* #define ENABLE_WORKQUEUES */
#endif

/* Enable prefetching may help performance */
#define ENABLE_PREFETCH

#define DEBUG_MAIN      (1 << 0)
#define DEBUG_DTMF      (1 << 1)
#define DEBUG_REGS      (1 << 2)
#define DEBUG_TSI       (1 << 3)
#define DEBUG_ECHOCAN   (1 << 4)
#define DEBUG_RBS       (1 << 5)
#define DEBUG_FRAMER    (1 << 6)
#define DEBUG_DSP       (1 << 7)

#define assert(expr) \
    if (unlikely(!(expr))) { \
    printk(KERN_ERR "Assertion failed! %s, %s, %s, line=%d\n",\
    #expr, __FILE__, __FUNCTION__, __LINE__); \
    }

#ifdef ENABLE_WORKQUEUES
#include <linux/cpu.h>

/* XXX UGLY!!!! XXX  We have to access the direct structures of the workqueue which
  are only defined within workqueue.c because they don't give us a routine to allow us
  to nail a work to a particular thread of the CPU.  Nailing to threads gives us 
  substantially higher scalability in multi-CPU environments though! */

/*
 * The per-CPU workqueue (if single thread, we always use cpu 0's).
 *
 * The sequence counters are for flush_scheduled_work().  It wants to wait
 * until until all currently-scheduled works are completed, but it doesn't
 * want to be livelocked by new, incoming ones.  So it waits until
 * remove_sequence is >= the insert_sequence which pertained when
 * flush_scheduled_work() was called.
 */

struct cpu_workqueue_struct {

	spinlock_t lock;

	long remove_sequence;		/* Least-recently added (next to run) */
	long insert_sequence;		/* Next to add */

	struct list_head worklist;
	wait_queue_head_t more_work;
	wait_queue_head_t work_done;

	struct workqueue_struct *wq;
	task_t *thread;

	int run_depth;				/* Detect run_workqueue() recursion depth */
} ____cacheline_aligned;

/*
 * The externally visible workqueue abstraction is an array of
 * per-CPU workqueues:
 */
struct workqueue_struct {
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,15)
	struct cpu_workqueue_struct *cpu_wq;
#else
	struct cpu_workqueue_struct cpu_wq[NR_CPUS];
#endif
	const char *name;
	struct list_head list;		/* Empty if single thread */
};

/* Preempt must be disabled. */
static void __rxt1_queue_work(struct cpu_workqueue_struct *cwq, struct work_struct *work)
{
	unsigned long flags;

	spin_lock_irqsave(&cwq->lock, flags);
	work->wq_data = cwq;
	list_add_tail(&work->entry, &cwq->worklist);
	cwq->insert_sequence++;
	wake_up(&cwq->more_work);
	spin_unlock_irqrestore(&cwq->lock, flags);
}

/*
 * Queue work on a workqueue. Return non-zero if it was successfully
 * added.
 *
 * We queue the work to the CPU it was submitted, but there is no
 * guarantee that it will be processed by that CPU.
 */
static inline int rxt1_queue_work(struct workqueue_struct *wq, struct work_struct *work,
								  int cpu)
{
	int ret = 0;
	get_cpu();

	if (!test_and_set_bit(0, &work->pending)) {
		BUG_ON(!list_empty(&work->entry));
		__rxt1_queue_work(wq->cpu_wq + cpu, work);
		ret = 1;
	}
	put_cpu();
	return ret;
}

#endif

static int debug = 0;
/* static int debug = (DEBUG_MAIN | DEBUG_DTMF | DEBUG_REGS | DEBUG_TSI | 
 *                      DEBUG_ECHOCAN | DEBUG_RBS | DEBUG_FRAMER); */
/* static int debug = (DEBUG_MAIN | DEBUG_DSP); */
/* static int debug = (DEBUG_MAIN | DEBUG_DTMF | DEBUG_REGS | DEBUG_TSI | 
 *                      DEBUG_ECHOCAN); */
static int monitor_mode = 0;
static int timingcable = 0;
/* static int recq_off = 5; */
static int recq_off = 1;
static int recd_off = 1;
static int xmit_off = 0;
static int sic3_set = 0x0c;
/* static int sic3_set = 0; */
static int ec_disable_1 = 0;
static int ec_disable_2 = 0;
static int ec_disable_3 = 0;
static int ec_disable_4 = 0;
static int no_ec = 0;
static int nlp_type = 3;
static int porboot = 0;
static int memloop = 0;
static int test_pat = 0;
static int regdump = 0;
static int regdumped = 0;
static int insert_idle = 0;
static int local_loop = 0;
static int double_buffer = 0;
static int show_pointers = 0;
static int highestorder;
static int t1e1override = 0x00;	/* -1 = jumper; 0xFF = E1 */
static int j1mode = 0;
static int sigmode = FRMR_MODE_NO_ADDR_CMP;
static int loopback = 0;
static int alarmdebounce = 0;
/* Enabling bursting can more efficiently utilize PCI bus bandwidth, but
   can also cause PCI bus starvation, especially in combination with other
   aggressive cards.  Please note that burst mode has no effect on CPU
   utilization / max number of calls / etc. */
static int noburst = 1;
static int debugslips = 0;
static int polling = 0;
static int gen_clk = 0;

#define MAX_SPANS 16

#define FLAG_STARTED (1 << 0)
#define FLAG_NMF (1 << 1)
#define FLAG_SENDINGYELLOW (1 << 2)

typedef struct {
	__u32 offsetw;
	__u32 offsetb;
	char *name;
	__u32 initial;
	__u32 iomask;
} pciregs;

static pciregs target_regs[] = {
/* offset     name       initial     mask */
	{0, 0x400, "VERSION", 0x00000000, 0x0000000c},	/* 0x1000 */
	{1, 0x404, "DMA_CONT", 0x00000000, 0xffff004d},	/* 0x1004 */
	{2, 0x408, "RX_ADDR", 0x00000000, 0xffffffff},	/* 0x1008 */
	{3, 0x412, "TX_ADDR", 0x00000000, 0xffffffff},	/* 0x100c */
	{4, 0x416, "STATUS", 0x00000000, 0x00ffff00},	/* 0x1010 */
	{5, 0x405, "HPI_W", 0x00000000, 0xffffffff},	/* 0x1014 */
	{6, 0x406, "HPI_R", 0x00000000, 0x00000000},	/* 0x1018 */
	{7, 0x407, "HPI_C", 0x00000000, 0x00000050},	/* 0x1018 */

};

static pciregs framer_regs[] = {
/*    offset             name            initial mask */
	/* 0x00,  Transmission FIFO */
	{0x00, (0x00 << 2), "FRMR_FIFOL", 0x00, 0x00},
	/* 0x01,  Transmission FIFO */
	{0x01, (0x01 << 2), "FRMR_FIFOH", 0x00, 0x00},
	/* 0x02,  Command Register */
	{0x02, (0x02 << 2), "FRMR_CMDR", 0x00, 0x00},
	/* 0x03,  Mode Register */
	{0x03, (0x03 << 2), "FRMR_MODE", 0x00, 0xff},
	/* 0x04,  Receive Address High 1 */
	{0x04, (0x04 << 2), "FRMR_RAH1", 0x00, 0xff},
	/* 0x05,  Receive Address High 2 */
	{0x05, (0x05 << 2), "FRMR_RAH2", 0x00, 0xff},
	/* 0x06,  Receive Address Low 1 */
	{0x06, (0x06 << 2), "FRMR_RAL1", 0x00, 0xff},
	/* 0x07,  Receive Address Low 2 */
	{0x07, (0x07 << 2), "FRMR_RAL2", 0x00, 0xff},
	/* 0x08,  Interrupt Port Configuration */
	{0x08, (0x08 << 2), "FRMR_IPC", 0x00, 0xff},
	/* 0x09,  Common Configuration Register 1 */
	{0x09, (0x09 << 2), "FRMR_CCR1", 0x00, 0xff},
	/* 0x0A,  Common Configuration Register 3 */
	{0x0A, (0x0A << 2), "FRMR_CCR2", 0x00, 0xff},
	/* 0x0B,  Preamble Register */
	{0x0B, (0x0B << 2), "FRMR_PRE", 0x00, 0xff},
	/* 0x0C,  Receive Timeslot Register 1 */
	{0x0C, (0x0C << 2), "FRMR_RTR1", 0x00, 0xff},
	/* 0x0D,  Receive Timeslot Register 2 */
	{0x0D, (0x0D << 2), "FRMR_RTR2", 0x00, 0xff},
	/* 0x0E,  Receive Timeslot Register 3 */
	{0x0E, (0x0E << 2), "FRMR_RTR3", 0x00, 0xff},
	/* 0x0F,  Receive Timeslot Register 4 */
	{0x0F, (0x0F << 2), "FRMR_RTR4", 0x00, 0xff},
	/* 0x10,  Transmit Timeslot Register 1 */
	{0x10, (0x10 << 2), "FRMR_TTR1", 0x00, 0xff},
	/* 0x11,  Transmit Timeslot Register 2 */
	{0x11, (0x11 << 2), "FRMR_TTR2", 0x00, 0xff},
	/* 0x12,  Transmit Timeslot Register 3 */
	{0x12, (0x12 << 2), "FRMR_TTR3", 0x00, 0xff},
	/* 0x13,  Transmit Timeslot Register 4 */
	{0x13, (0x13 << 2), "FRMR_TTR4", 0x00, 0xff},
	/* 0x14,  Interrupt Mask Register 0 */
	{0x14, (0x14 << 2), "FRMR_IMR0", 0x00, 0xff},
	/* 0x15,  Interrupt Mask Register 1 */
	{0x15, (0x15 << 2), "FRMR_IMR1", 0x00, 0xff},
	/* 0x16,  Interrupt Mask Register 2 */
	{0x16, (0x16 << 2), "FRMR_IMR2", 0x00, 0xff},
	/* 0x17,  Interrupt Mask Register 3 */
	{0x17, (0x17 << 2), "FRMR_IMR3", 0x00, 0xff},
	/* 0x18,  Interrupt Mask Register 4 */
	{0x18, (0x18 << 2), "FRMR_IMR4", 0x00, 0xff},
	/* 0x19,  Gap within address range no.1 */
	{0x19, (0x19 << 2), "FRMR_RESERVED_19", 0x00, 0x00},
	/* 0x1A,  Gap within address range no.1 */
	{0x1A, (0x1A << 2), "FRMR_RESERVED_2A", 0x00, 0x00},
	/* 0x1B,  Single Bit Insertion Register */
	{0x1B, (0x1B << 2), "FRMR_IERR", 0x00, 0xff},
	/* 0x1C,  Framer Mode Register 0 */
	{0x1C, (0x1C << 2), "FRMR_FMR0", 0x00, 0xff},
	/* 0x1D,  Framer Mode Register 1 */
	{0x1D, (0x1D << 2), "FRMR_FMR1", 0x00, 0xff},
	/* 0x1E,  Framer Mode Register 2 */
	{0x1E, (0x1E << 2), "FRMR_FMR2", 0x00, 0xff},
	/* 0x1F,  Channel Loop Back */
	{0x1F, (0x1F << 2), "FRMR_LOOP", 0x00, 0xff},
	/* 0x20,  Transmit Service Word Framer Mode Reigster 4 */
	{0x20, (0x20 << 2), "FRMR_XSW_FMR4", 0x00, 0xff},
	/* 0x21,  Transmit Spare Bits Framer Mode Reigster 5 */
	{0x21, (0x21 << 2), "FRMR_XSP_FMR5", 0x00, 0xff},
	/* 0x22,  Transmit Control 0 */
	{0x22, (0x22 << 2), "FRMR_XC0", 0x00, 0xff},
	/* 0x23,  Transmit Control 1 */
	{0x23, (0x23 << 2), "FRMR_XC1", 0x00, 0xff},
	/* 0x24,  Receive Control 0 */
	{0x24, (0x24 << 2), "FRMR_RC0", 0x00, 0xff},
	/* 0x25,  Receive Control 1 */
	{0x25, (0x25 << 2), "FRMR_RC1", 0x00, 0xff},
	/* 0x26,  Transmit Pulse Mask 0 */
	{0x26, (0x26 << 2), "FRMR_XPM0", 0x00, 0xff},
	/* 0x27,  Transmit Pulse Mask 1 */
	{0x27, (0x27 << 2), "FRMR_XPM1", 0x00, 0xff},
	/* 0x28,  Transmit Pulse Mask 2 */
	{0x28, (0x28 << 2), "FRMR_XPM2", 0x00, 0xff},
	/* 0x29,  Transparent Service Word Mask */
	{0x29, (0x29 << 2), "FRMR_TSWM", 0x00, 0xff},
	/* 0x2A,  Unused Byte No.2 */
	{0x2A, (0x2A << 2), "FRMR_RESERVED_2A", 0x00, 0x00},
	/* 0x2B,  Idle Channel Code */
	{0x2B, (0x2B << 2), "FRMR_IDLE", 0x00, 0xff},
	/* 0x2C,  Transmit SA4 Bit Register Fransmit DL-Bit Register 1 */
	{0x2C, (0x2C << 2), "FRMR_XSA4_XDL1", 0x00, 0xff},
	/* 0x2D,  Transmit SA5 Bit Register Fransmit DL-Bit Register 2 */
	{0x2D, (0x2D << 2), "FRMR_XSA5_XDL2", 0x00, 0xff},
	/* 0x2E,  Transmit SA6 Bit Register Fransmit DL-Bit Register 3 */
	{0x2E, (0x2E << 2), "FRMR_XSA6_XDL3", 0x00, 0xff},
	/* 0x2F,  Transmit SA7 Bit Register Clear Channel Register 1 */
	{0x2F, (0x2F << 2), "FRMR_XSA7_CCB1", 0x00, 0xff},
	/* 0x30,  Transmit SA8 Bit Register Clear Channel Register 2 */
	{0x30, (0x30 << 2), "FRMR_XSA8_CCB2", 0x00, 0xff},
	/* 0x31,  Framer Mode Reg. 3 Clear Channel Register 3 */
	{0x31, (0x31 << 2), "FRMR_FMR3_CCB3", 0x00, 0xff},
	/* 0x32,  Idle Channel Register 1 */
	{0x32, (0x32 << 2), "FRMR_ICB1", 0x00, 0xff},
	/* 0x33,  Idle Channel Register 2 */
	{0x33, (0x33 << 2), "FRMR_ICB2", 0x00, 0xff},
	/* 0x34,  Idle Channel Register 3 */
	{0x34, (0x34 << 2), "FRMR_ICB3", 0x00, 0xff},
	/* 0x35,  Idle Channel Register 4 */
	{0x35, (0x35 << 2), "FRMR_ICB4", 0x00, 0xff},
	/* 0x36,  Line Interface Mode 0 EQ bit is gone */
	{0x36, (0x36 << 2), "FRMR_LIM0", 0x00, 0xf7},
	/* 0x37,  Line Interface Mode 1 */
	{0x37, (0x37 << 2), "FRMR_LIM1", 0x00, 0xff},
	/* 0x38,  Pulse Count Detection */
	{0x38, (0x38 << 2), "FRMR_PCD", 0x00, 0xff},
	/* 0x39,  Pulse Count Recovery */
	{0x39, (0x39 << 2), "FRMR_PCR", 0x00, 0xff},
	/* 0x3A,  Line Interface Mode Register 2 */
	{0x3A, (0x3A << 2), "FRMR_LIM2", 0x00, 0xff},
	/* 0x3B,  Line Code Register 1 */
	{0x3B, (0x3B << 2), "FRMR_LCR1", 0x00, 0xff},
	/* 0x3C,  Line Code Register 2 */
	{0x3C, (0x3C << 2), "FRMR_LCR2", 0x00, 0xff},
	/* 0x3D,  Line Code Register 3 */
	{0x3D, (0x3D << 2), "FRMR_LCR3", 0x00, 0xff},
	/* 0x3E,  System Interface Control 1 */
	{0x3E, (0x3E << 2), "FRMR_SIC1", 0x00, 0xff},
	/* 0x3F,  System Interface Control 2 */
	{0x3F, (0x3F << 2), "FRMR_SIC2", 0x00, 0xff},
	/* 0x40,  System Interface Control 3 */
	{0x40, (0x40 << 2), "FRMR_SIC3", 0x00, 0xff},
	/* 0x41,  Gap within address range no.1 */
	{0x41, (0x41 << 2), "FRMR_RESERVED_41", 0x00, 0x00},
	/* 0x42,  Gap within address range no.1 */
	{0x42, (0x42 << 2), "FRMR_RESERVED_42", 0x00, 0x00},
	/* 0x43,  Gap within address range no.1 */
	{0x43, (0x43 << 2), "FRMR_RESERVED_43", 0x00, 0x00},
	/* 0x44,  Clock Mode Register 1 */
	{0x44, (0x44 << 2), "FRMR_CMR1", 0x00, 0xff},
	/* 0x45,  Clock Mode Register 2 */
	{0x45, (0x45 << 2), "FRMR_CMR2", 0x00, 0xff},
	/* 0x46,  Global Configuration Register */
	{0x46, (0x46 << 2), "FRMR_GCR", 0x00, 0xff},
	/* 0x47,  Errored Second Mask */
	{0x47, (0x47 << 2), "FRMR_ESM", 0x00, 0xff},
	/* 0x48,  Gap within address range no.1 */
	{0x48, (0x48 << 2), "FRMR_RESERVED_48", 0x00, 0x00},
	/* 0x49,  Receive Buffer Delay */
	{0x49, (0x49 << 2), "FRMR_RBD", 0x00, 0xff},
	/* 0x4A,  Version Status */
	{0x4A, (0x4A << 2), "FRMR_VSTR", 0x00, 0x00},
	/* 0x4B,  Receive Equilizer Status */
	{0x4B, (0x4B << 2), "FRMR_RES", 0x00, 0xff},
	/* 0x4C,  Framer Receive Status 0 */
	{0x4C, (0x4C << 2), "FRMR_FRS0", 0x00, 0xff},
	/* 0x4D,  Framer Receive Status 1 */
	{0x4D, (0x4D << 2), "FRMR_FRS1", 0x00, 0xff},
	/* 0x4E,  Receive Service Word Framer Receive Status 2 */
	{0x4E, (0x4E << 2), "FRMR_RSW_FRS2", 0x00, 0xff},
	/* 0x4F,  Receive Spare Bits Framer Receive Status 3 */
	{0x4F, (0x4F << 2), "FRMR_RSP_FRS3", 0x00, 0xff},
	/* 0x50,  Framing Error Counter */
	{0x50, (0x50 << 2), "FRMR_FECL", 0x00, 0xff},
	/* 0x51,  Framing Error Counter */
	{0x51, (0x51 << 2), "FRMR_FECH", 0x00, 0xff},
	/* 0x52,  Code Violation Counter */
	{0x52, (0x52 << 2), "FRMR_CVCL", 0x00, 0xff},
	/* 0x53,  Code Violation Counter */
	{0x53, (0x53 << 2), "FRMR_CVCH", 0x00, 0xff},
	/* 0x54,  CRC Error Counter 1 */
	{0x54, (0x54 << 2), "FRMR_CEC1L", 0x00, 0xff},
	/* 0x55,  CRC Error Counter 1 */
	{0x55, (0x55 << 2), "FRMR_CEC1H", 0x00, 0xff},
	/* 0x56,  E-Bit Error Counter */
	{0x56, (0x56 << 2), "FRMR_EBCL", 0x00, 0xff},
	/* 0x57,  E-Bit Error Counter */
	{0x57, (0x57 << 2), "FRMR_EBCH", 0x00, 0xff},
	/* 0x58,  CRC Error Counter 2 (E1) */
	{0x58, (0x58 << 2), "FRMR_CEC2_BECL", 0x00, 0xff},
	/* 0x59,  CRC Error Counter 2 (E1) */
	{0x59, (0x59 << 2), "FRMR_CEC2_BECH", 0x00, 0xff},
	/* 0x5A,  CRC Error Counter 3 (E1) */
	{0x5A, (0x5A << 2), "FRMR_CEC3_COFAL", 0x00, 0xff},
	/* 0x5B,  CRC Error Counter 3 (E1) */
	{0x5B, (0x5B << 2), "FRMR_CEC3_COFAH", 0x00, 0xff},
	/* 0x5C,  Receive SA4 Bit Register Receive DL-Bit Register 1 */
	{0x5C, (0x5C << 2), "FRMR_RSA4_RDL1", 0x00, 0xff},
	/* 0x5D,  Receive SA5 Bit Register Receive DL-Bit Register 2 */
	{0x5D, (0x5D << 2), "FRMR_RSA5_RDL2", 0x00, 0xff},
	/* 0x5E,  Receive SA6 Bit Register Receive DL-Bit Register 3 */
	{0x5E, (0x5E << 2), "FRMR_RSA6_RDL3", 0x00, 0xff},
	/* 0x5F,  Receive SA7 Bit Register */
	{0x5F, (0x5F << 2), "FRMR_RSA7", 0x00, 0xff},
	/* 0x60,  Receive SA8 Bit Register */
	{0x60, (0x60 << 2), "FRMR_RSA8_DEC", 0x00, 0x00},
	/* 0x61,  Receive Sa6 Bit Status Register */
	{0x61, (0x61 << 2), "FRMR_RSA6S", 0x00, 0xff},
	/* 0x62,  Manufacturer Test Register 0 */
	{0x62, (0x62 << 2), "FRMR_RSP1_TEST2", 0x00, 0x00},
	/* 0x63,  Manufacturer Test Register   1 */
	{0x63, (0x63 << 2), "FRMR_RSP2", 0x00, 0xff},
	/* 0x64,  Signaling Status Register */
	{0x64, (0x64 << 2), "FRMR_SIS", 0x00, 0xff},
	/* 0x65,  Receive Signaling Status Register */
	{0x65, (0x65 << 2), "FRMR_RSIS", 0x00, 0xff},
	/* 0x66,  Receive Byte Control */
	{0x66, (0x66 << 2), "FRMR_RBCL", 0x00, 0xff},
	/* 0x67,  Receive Byte Control */
	{0x67, (0x67 << 2), "FRMR_RBCH", 0x00, 0xff},
	/* 0x68,  Interrupt Status Register 0 */
	{0x68, (0x68 << 2), "FRMR_ISR0", 0x00, 0xff},
	/* 0x69,  Interrupt Status Register 1 */
	{0x69, (0x69 << 2), "FRMR_ISR1", 0x00, 0xff},
	/* 0x6A,  Interrupt Status Register 2 */
	{0x6A, (0x6A << 2), "FRMR_ISR2", 0x00, 0xff},
	/* 0x6B,  Interrupt Status Register 3 */
	{0x6B, (0x6B << 2), "FRMR_ISR3", 0x00, 0xff},
	/* 0x6C,  Interrupt Status Register 4 */
	{0x6C, (0x6C << 2), "FRMR_ISR4", 0x00, 0xff},
	/* 0x6D,  Unused Byte No.3 */
	{0x6D, (0x6D << 2), "FRMR_RESERVED_6D", 0x00, 0xff},
	/* 0x6E,  Global Interrupt Status */
	{0x6E, (0x6E << 2), "FRMR_GIS", 0x00, 0xff},
	/* 0x6F,  Channel Interrupt Status */
	{0x6F, (0x6F << 2), "FRMR_CIS", 0x00, 0xff},
	/* 0x70,  Receive CAS Register   1...16 */
	{0x70, (0x70 << 2), "FRMR_XS_RS_1", 0x00, 0x00},
	/* 0x71,  Receive CAS Register   1...16 */
	{0x71, (0x71 << 2), "FRMR_XS_RS_2", 0x00, 0x00},
	/* 0x72,  Receive CAS Register   1...16 */
	{0x72, (0x72 << 2), "FRMR_XS_RS_3", 0x00, 0x00},
	/* 0x73,  Receive CAS Register   1...16 */
	{0x73, (0x73 << 2), "FRMR_XS_RS_4", 0x00, 0x00},
	/* 0x74,  Receive CAS Register   1...16 */
	{0x74, (0x74 << 2), "FRMR_XS_RS_5", 0x00, 0x00},
	/* 0x75,  Receive CAS Register   1...16 */
	{0x75, (0x75 << 2), "FRMR_XS_RS_6", 0x00, 0x00},
	/* 0x76,  Receive CAS Register   1...16 */
	{0x76, (0x76 << 2), "FRMR_XS_RS_7", 0x00, 0x00},
	/* 0x77,  Receive CAS Register   1...16 */
	{0x77, (0x77 << 2), "FRMR_XS_RS_8", 0x00, 0x00},
	/* 0x78,  Receive CAS Register   1...16 */
	{0x78, (0x78 << 2), "FRMR_XS_RS_9", 0x00, 0x00},
	/* 0x79,  Receive CAS Register   1...16 */
	{0x79, (0x79 << 2), "FRMR_XS_RS_10", 0x00, 0x00},
	/* 0x7a,  Receive CAS Register   1...16 */
	{0x7a, (0x7a << 2), "FRMR_XS_RS_11", 0x00, 0x00},
	/* 0x7b,  Receive CAS Register   1...16 */
	{0x7b, (0x7b << 2), "FRMR_XS_RS_12", 0x00, 0x00},
	/* 0x7c,  Receive CAS Register   1...16 */
	{0x7c, (0x7c << 2), "FRMR_XS_RS_13", 0x00, 0x00},
	/* 0x7d,  Receive CAS Register   1...16 */
	{0x7d, (0x7d << 2), "FRMR_XS_RS_14", 0x00, 0x00},
	/* 0x7e,  Receive CAS Register   1...16 */
	{0x7e, (0x7e << 2), "FRMR_XS_RS_15", 0x00, 0x00},
	/* 0x7f,  Receive CAS Register   1...16 */
	{0x7f, (0x7f << 2), "FRMR_XS_RS_16", 0x00, 0x00},
	/* 0x80,  Port Configuration 1 */
	{0x80, (0x80 << 2), "FRMR_PC1", 0x00, 0xff},
	/* 0x81,  Port Configuration 2 */
	{0x81, (0x81 << 2), "FRMR_PC2", 0x00, 0xff},
	/* 0x82,  Port Configuration 3 */
	{0x82, (0x82 << 2), "FRMR_PC3", 0x00, 0xff},
	/* 0x83,  Port Configuration 4 */
	{0x83, (0x83 << 2), "FRMR_PC4", 0x00, 0xff},
	/* 0x84,  Port Configuration 5 */
	{0x84, (0x84 << 2), "FRMR_PC5", 0x00, 0xff},
	/* 0x85,  Global Port Configuration 1 */
	{0x85, (0x85 << 2), "FRMR_GPC1", 0x00, 0xff},
	/* 0x86,  Unused Byte 3 */
	{0x86, (0x86 << 2), "FRMR_RESERVED_86", 0x00, 0xff},
	/* 0x87,  Command Register no.2 */
	{0x87, (0x87 << 2), "FRMR_CMDR2", 0x00, 0x00},
	/* 0x88,  Gap within address range */
	{0x88, (0x88 << 2), "FRMR_RESERVED_88", 0x00, 0x00},
	/* 0x89,  Gap within address range */
	{0x89, (0x89 << 2), "FRMR_RESERVED_89", 0x00, 0x00},
	/* 0x8a,  Gap within address range */
	{0x8a, (0x8a << 2), "FRMR_RESERVED_8A", 0x00, 0x00},
	/* 0x8b,  Gap within address range */
	{0x8b, (0x8b << 2), "FRMR_RESERVED_8B", 0x00, 0x00},
	/* 0x8c,  Gap within address range */
	{0x8c, (0x8c << 2), "FRMR_RESERVED_8C", 0x00, 0x00},
	/* 0x8D,  Common Configuration Register 5 */
	{0x8D, (0x8D << 2), "FRMR_CCR5", 0x00, 0xff},
	/* 0x8E,  Gap within address range */
	{0x8E, (0x8E << 2), "FRMR_RESERVED_8E", 0x00, 0x00},
	/* 0x8f,  Gap within address range */
	{0x8f, (0x8f << 2), "FRMR_RESERVED_8F", 0x00, 0x00},
	/* 0x90,  Gap within address range */
	{0x90, (0x90 << 2), "FRMR_RESERVED_90", 0x00, 0x00},
	/* 0x91,  Gap within address range */
	{0x91, (0x91 << 2), "FRMR_RESERVED_91", 0x00, 0x00},
	/* 0x92,  Global Clocking Modes */
	{0x92, (0x92 << 2), "FRMR_GCM1", 0x00, 0xff},
	/* 0x93,  Channel Interrupt Status */
	{0x93, (0x93 << 2), "FRMR_GCM2", 0x00, 0xff},
	/* 0x94,  Global Clocking Modes */
	{0x94, (0x94 << 2), "FRMR_GCM3", 0x00, 0xff},
	/* 0x95,  Channel Interrupt Status */
	{0x95, (0x95 << 2), "FRMR_GCM4", 0x00, 0xff},
	/* 0x96,  Global Clocking Modes */
	{0x96, (0x96 << 2), "FRMR_GCM5", 0x00, 0xff},
	/* 0x97,  Global Clocking Modes */
	{0x97, (0x97 << 2), "FRMR_GCM6", 0x00, 0xff},
	/* 0x98,  Gap within address range */
	{0x98, (0x98 << 2), "FRMR_RESERVED_98", 0x00, 0x00},
	/* 0x99,  Gap within address range */
	{0x99, (0x99 << 2), "FRMR_RESERVED_99", 0x00, 0x00},
	/* 0x9a,  Gap within address range */
	{0x9a, (0x9a << 2), "FRMR_RESERVED_9A", 0x00, 0x00},
	/* 0x9b,  Gap within address range */
	{0x9b, (0x9b << 2), "FRMR_RESERVED_9B", 0x00, 0x00},
	/* 0x9c,  Gap within address range */
	{0x9c, (0x9c << 2), "FRMR_RESERVED_9C", 0x00, 0x00},
	/* 0x9d,  Gap within address range */
	{0x9d, (0x9d << 2), "FRMR_RESERVED_9D", 0x00, 0x00},
	/* 0x9f,  Gap within address range */
	{0x9f, (0x9f << 2), "FRMR_RESERVED_9F", 0x00, 0x00},
	/* 0xA0,  Time Slot Even/Odd Select */
	{0xA0, (0xA0 << 2), "FRMR_TSEO", 0x00, 0xff},
	/* 0xA1,  Time Slot Bit Select 1 */
	{0xA1, (0xA1 << 2), "FRMR_TSBS1", 0x00, 0xff},
	/* 0xA2,  Gap within address range */
	{0xA2, (0xA2 << 2), "FRMR_RESERVED_A2", 0x00, 0x00},
	/* 0xA3,  Gap within address range */
	{0xA3, (0xA3 << 2), "FRMR_RESERVED_A3", 0x00, 0x00},
	/* 0xA4,  Gap within address range */
	{0xA4, (0xA4 << 2), "FRMR_RESERVED_A4", 0x00, 0x00},
	/* 0xA5,  Gap within address range */
	{0xA5, (0xA5 << 2), "FRMR_RESERVED_A5", 0x00, 0x00},
	/* 0xA6,  Gap within address range */
	{0xA6, (0xA6 << 2), "FRMR_RESERVED_A6", 0x00, 0x00},
	/* 0xA7,  Gap within address range */
	{0xA7, (0xA7 << 2), "FRMR_RESERVED_A7", 0x00, 0x00},
	/* 0xA8,  Test Pattern Control 0 */
	{0xA8, (0xA8 << 2), "FRMR_TPC0", 0x00, 0xff},
	/* 0xA9,  Gap within address range */
	{0xA9, (0xA9 << 2), "FRMR_RESERVED_A9", 0x00, 0x00},
	/* 0xAA,  Gap within address range */
	{0xAA, (0xAA << 2), "FRMR_RESERVED_AA", 0x00, 0x00},
	/* 0xAB,  Gap within address range */
	{0xAB, (0xAB << 2), "FRMR_RESERVED_AB", 0x00, 0x00},
	/* 0xAC,  Gap within address range */
	{0xAC, (0xAC << 2), "FRMR_RESERVED_AC", 0x00, 0x00},
	/* 0xAD,  Gap within address range */
	{0xAD, (0xAD << 2), "FRMR_RESERVED_AD", 0x00, 0x00},
	/* 0xAE,  Gap within address range */
	{0xAE, (0xAE << 2), "FRMR_RESERVED_AE", 0x00, 0x00},
	/* 0xAF,  Gap within address range */
	{0xAF, (0xAF << 2), "FRMR_RESERVED_AF", 0x00, 0x00},
	/* 0xB0  Gap within address range */
	{0xB0, (0xB0 << 2), "FRMR_RESERVED_B0", 0x00, 0x00},
	/* 0xB1  Gap within address range */
	{0xB1, (0xB1 << 2), "FRMR_RESERVED_B1", 0x00, 0x00},
	/* 0xB2  Gap within address range */
	{0xB2, (0xB2 << 2), "FRMR_RESERVED_B2", 0x00, 0x00},
	/* 0xB3  Gap within address range */
	{0xB3, (0xB3 << 2), "FRMR_RESERVED_B3", 0x00, 0x00},
	/* 0xB4  Gap within address range */
	{0xB4, (0xB4 << 2), "FRMR_RESERVED_B4", 0x00, 0x00},
	/* 0xB5  Gap within address range */
	{0xB5, (0xB5 << 2), "FRMR_RESERVED_B5", 0x00, 0x00},
	/* 0xB6  Gap within address range */
	{0xB6, (0xB6 << 2), "FRMR_RESERVED_B6", 0x00, 0x00},
	/* 0xB7  Gap within address range */
	{0xB7, (0xB7 << 2), "FRMR_RESERVED_B7", 0x00, 0x00},
	/* 0xB8  Gap within address range */
	{0xB8, (0xB8 << 2), "FRMR_RESERVED_B8", 0x00, 0x00},
	/* 0xB9  Gap within address range */
	{0xB9, (0xB9 << 2), "FRMR_RESERVED_B9", 0x00, 0x00},
	/* 0xBA  Gap within address range */
	{0xBA, (0xBA << 2), "FRMR_RESERVED_BA", 0x00, 0x00},
	/* 0xBB */
	{0xBB, (0xBB << 2), "FRMR_REGFP", 0x00, 0xff},
	/* 0xBC */
	{0xBC, (0xBC << 2), "FRMR_REGFD", 0x00, 0x00},
};


#define TYPE_T1 1				/* is a T1 card */
#define TYPE_E1 2				/* is an E1 card */
#define TYPE_J1 3				/* is a running J1 */

#define FLAG_2PORT   (1 << 4)
#define FLAG_OCTOPT  (1 << 6)

#define CANARY 0xc0de

struct devtype {
	char *desc;
	unsigned int flags;
};

static struct devtype r4t1 = { "Rhino Equipment R4T1", 0 };
static struct devtype r2t1 = { "Rhino Equipment R2T1", FLAG_2PORT };

static int inirq = 0;

static void __rxt1_span_set_clear(struct rxt1_card_t *rxt1_card, int span);
static int rxt1_span_startup(struct dahdi_span *zap_span);
static int rxt1_span_shutdown(struct dahdi_span *zap_span);
static int rxt1_zap_chan_rbsbits(struct dahdi_chan *zap_chan, int bits);
static int rxt1_span_maint(struct dahdi_span *zap_span, int cmd);
#ifdef DAHDI_SIG_HARDHDLC
static void rxt1_zap_chan_hdlc_hard_xmit(struct dahdi_chan *zap_chan);
#endif
static int rxt1_zap_chan_ioctl(struct dahdi_chan *zap_chan, unsigned int cmd,
							   unsigned long data);
static void rxt1_card_tsi_assign(struct rxt1_card_t *rxt1_card, int fromspan,
								 int fromchan, int tospan, int tochan);
static void rxt1_card_tsi_unassign(struct rxt1_card_t *rxt1_card, int tospan, int tochan);
static void __rxt1_card_set_timing_source(struct rxt1_card_t *rxt1_card, int unit,
										  int master, int slave);
static void rxt1_span_check_alarms(struct rxt1_card_t *rxt1_card, int span);
static void rxt1_span_check_sigbits(struct rxt1_card_t *rxt1_card, int span);

static int rxt1_echocan_create(struct dahdi_chan *chan, struct dahdi_echocanparams *ecp,
							   struct dahdi_echocanparam *p,
							   struct dahdi_echocan_state **ec);
static void rxt1_echocan_free(struct dahdi_chan *chan, struct dahdi_echocan_state *ec);

static const struct dahdi_echocan_features my_ec_features = {
	.NLP_automatic = 1,
	.CED_tx_detect = 1,
	.CED_rx_detect = 1,
};

static const struct dahdi_echocan_ops my_ec_ops = {
	.name = "RXT1_HWEC",
	.echocan_free = rxt1_echocan_free,
};


#ifdef ENABLE_TASKLETS
static void rxt1_tasklet(unsigned long data);
#endif

static struct rxt1_card_t *rxt1_cards[MAX_RXT1_CARDS];

#define MAX_TDM_CHAN 32
#define MAX_DTMF_DET 16

#define HDLC_IMR0_MASK (FRMR_IMR0_RME | FRMR_IMR0_RPF)
#define HDLC_IMR1_MASK  (FRMR_IMR1_XDU | FRMR_IMR1_XPR)

/* read __u32 from BAR 0 indexed by DWORDS */
/* word 401 = bus address 1004 + BAR 0 = Framer address */
inline unsigned int __rxt1_card_pci_in(struct rxt1_card_t *rxt1_card,
									   const unsigned int addr)
{
	unsigned int res = readl(&rxt1_card->membase[addr]);
	return res;
}

/* write __u32 to BAR 0 indexed by DWORDS */
/* and test with mask word 401 = bus address 1004 */
inline void __rxt1_card_pci_out(struct rxt1_card_t *rxt1_card, const unsigned int addr,
								const unsigned int value, const unsigned int mask)
{
	/* &rxt1_case->membase[addr] CANNOT BE less than rxt1_card->membase 
	 *      but it was being tested before, so we'll leave this here */
	if (&rxt1_card->membase[addr] < rxt1_card->membase) {
		printk
			("PCI out writing %x to addr %p OUCH base is %p and Last Reg is %p OUCH (1)\n",
			 value, &rxt1_card->membase[addr], rxt1_card->membase,
			 &rxt1_card->membase[0x1068]);
		return;
	} else if (addr < 0x400 && value > 255) {
		printk("PCI out writing %x to addr %p OUCH too big for register OUCH\n",
			   value, &rxt1_card->membase[addr]);
	}

	writel(value, &rxt1_card->membase[addr]);
}

static inline void rxt1_card_pci_out(struct rxt1_card_t *rxt1_card,
									 const unsigned int addr, const unsigned int value,
									 const unsigned int mask)
{
	unsigned long flags;
	spin_lock_irqsave(&rxt1_card->reglock, flags);
	__rxt1_card_pci_out(rxt1_card, addr, value, mask);
	spin_unlock_irqrestore(&rxt1_card->reglock, flags);
}

static inline void __rxt1_card_set_led(struct rxt1_card_t *rxt1_card, int span, int state)
{
	int retry = 0;
	int oldreg = rxt1_card->ledreg;
	int mask = target_regs[RXT1_STAT].iomask;
	rxt1_card->ledreg &= ~(0xf << (8 + (span << 2)));
	rxt1_card->ledreg |= (state << (8 + (span << 2)));
	if (oldreg != rxt1_card->ledreg) {
		printk("Status Change on Span %d from %x to %x\n", span, oldreg,
			   rxt1_card->ledreg);
		__rxt1_card_pci_out(rxt1_card, RXT1_STAT + TARG_REGS, rxt1_card->ledreg, mask);
		while (((__rxt1_card_pci_in(rxt1_card, RXT1_STAT + TARG_REGS) & mask) !=
				(rxt1_card->ledreg & mask)) && ((retry++) < 10))
			__rxt1_card_pci_out(rxt1_card, RXT1_STAT + TARG_REGS, rxt1_card->ledreg,
								mask);
	}
}

static inline void rxt1_card_activate(struct rxt1_card_t *rxt1_card)
{
	rxt1_card->ledreg = STATINIT | 0x40;	/* keep JTAG locked and tri-state */
	rxt1_card_pci_out(rxt1_card, RXT1_STAT + TARG_REGS, rxt1_card->ledreg,
					  target_regs[RXT1_STAT].iomask);
}

static inline unsigned int rxt1_card_pci_in(struct rxt1_card_t *rxt1_card,
											const unsigned int addr)
{
	unsigned int ret;
	unsigned long flags;

	spin_lock_irqsave(&rxt1_card->reglock, flags);
	ret = __rxt1_card_pci_in(rxt1_card, addr);
	spin_unlock_irqrestore(&rxt1_card->reglock, flags);
	return ret;
}

unsigned long flags;

int try_select_framer(struct rxt1_card_t *rxt1_card)
{
	int hpi_lock;

	int sel;
	spin_lock_irqsave(&rxt1_card->reglock, flags);
	sel = __rxt1_card_pci_in(rxt1_card, RXT1_HCS_REG + TARG_REGS);
	if (sel == 0) {
		hpi_lock = __rxt1_card_pci_in(rxt1_card, TARG_REGS + RXT1_HPIC);
		__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_HPIC, hpi_lock & ~HPI_SEL,
							target_regs[RXT1_HPIC].iomask);
		__rxt1_card_pci_out(rxt1_card, RXT1_HCS_REG + TARG_REGS, (__u32) (0x10), 0);
		return (0);
	}
	spin_unlock_irqrestore(&rxt1_card->reglock, flags);
	return (1);
}

void rxt1_card_select_framer(struct rxt1_card_t *rxt1_card)
{
	int ridiculous = 0;

	while (try_select_framer(rxt1_card)) {
		mdelay(1);
		ridiculous++;
		if (ridiculous > 100000) {
			printk
				("I'm Broken. I waited 10 seconds .... and nothing happened. I quit. "
				 "I'm outta here.\n");
			BUG();
			return;
		}
	}
	return;
}

void rxt1_card_unselect_framer(struct rxt1_card_t *rxt1_card)
{
	int sel = __rxt1_card_pci_in(rxt1_card, RXT1_HCS_REG + TARG_REGS);
	if (sel != (0x10))
		printk("R%dT1: Framer un-select Wrong DSP Selected: Should be %x is %x\n",
			   rxt1_card->numspans, 0x10, sel);

	__rxt1_card_pci_out(rxt1_card, RXT1_HCS_REG + TARG_REGS, (__u32) (0), 0);
	spin_unlock_irqrestore(&rxt1_card->reglock, flags);

	return;
}

static inline unsigned int __rxt1_span_framer_in(struct rxt1_card_t *rxt1_card, int span,
												 const unsigned int addr)
{
	unsigned int ret, adj_addr;
	span &= 0x3;

	rxt1_card_select_framer(rxt1_card);

	/* Dual card span 1 is actually at span 2 */
	if ((rxt1_card->numspans == 2) && (span == 1))
		span = 2;
	else if ((rxt1_card->numspans == 2) && (span == 2))
		span = 1;

	adj_addr = (((span << 8) | (addr & 0xff)));
	ret = __rxt1_card_pci_in(rxt1_card, adj_addr);

	rxt1_card_unselect_framer(rxt1_card);

	return (ret & 0xff);
}

static inline unsigned int rxt1_span_framer_in(struct rxt1_card_t *rxt1_card, int span,
											   const unsigned int addr)
{
	return __rxt1_span_framer_in(rxt1_card, span, addr);
}

static inline void __rxt1_span_framer_out(struct rxt1_card_t *rxt1_card, int span,
										  const unsigned int addr,
										  const unsigned int value)
{
	unsigned int adj_addr;
	span &= 0x3;

	rxt1_card_select_framer(rxt1_card);

	/* Dual card span 2 is actually at span 3 */
	if ((rxt1_card->numspans == 2) && (span == 1))
		span = 2;
	else if ((rxt1_card->numspans == 2) && (span == 2))
		span = 1;

	adj_addr = (((span << 8) | (addr & 0xff)));
	if (debug & DEBUG_REGS)
		printk("Writing %02x to address %02x of span %d adj_addr %x\n", value, addr, span,
			   adj_addr);
	__rxt1_card_pci_out(rxt1_card, adj_addr, value, framer_regs[addr].iomask);

	rxt1_card_unselect_framer(rxt1_card);
}

static inline void rxt1_span_framer_out(struct rxt1_card_t *rxt1_card, int span,
										const unsigned int addr, const unsigned int value)
{
	__rxt1_span_framer_out(rxt1_card, span, addr, value);
}

static void __rxt1_span_hdlc_stop(struct rxt1_card_t *rxt1_card, unsigned int span)
{
	/* used in one place below */
	struct rxt1_span_t *rxt1_span = rxt1_card->rxt1_spans[span];
	unsigned char imr0, imr1, mode;
	int i = 0;

	if (debug & DEBUG_FRAMER)
		printk("Stopping HDLC controller on span %d\n", span + 1);

	/* Clear receive and transmit timeslots */
	for (i = 0; i < 4; i++) {
		__rxt1_span_framer_out(rxt1_card, span, FRMR_RTR_BASE + i, 0x00);
		__rxt1_span_framer_out(rxt1_card, span, FRMR_TTR_BASE + i, 0x00);
	}

	imr0 = __rxt1_span_framer_in(rxt1_card, span, FRMR_IMR0);
	imr1 = __rxt1_span_framer_in(rxt1_card, span, FRMR_IMR1);

	/* Disable HDLC interrupts */
	imr0 |= HDLC_IMR0_MASK;
	__rxt1_span_framer_out(rxt1_card, span, FRMR_IMR0, imr0);

	imr1 |= HDLC_IMR1_MASK;
	__rxt1_span_framer_out(rxt1_card, span, FRMR_IMR1, imr1);

	mode = __rxt1_span_framer_in(rxt1_card, span, FRMR_MODE);
	mode &= ~FRMR_MODE_HRAC;
	__rxt1_span_framer_out(rxt1_card, span, FRMR_MODE, mode);

	rxt1_span->sigactive = 0;
}

static inline void __rxt1_span_framer_cmd(struct rxt1_card_t *rxt1_card,
										  unsigned int span, int cmd)
{
	__rxt1_span_framer_out(rxt1_card, span, FRMR_CMDR, cmd);
}

static inline void __rxt1_span_framer_cmd_wait(struct rxt1_card_t *rxt1_card,
											   unsigned int span, int cmd)
{
	int sis;
	int loops = 0;

	/* XXX could be time consuming XXX */
	for (;;) {
		sis = __rxt1_span_framer_in(rxt1_card, span, FRMR_SIS);
		if (!(sis & 0x04))
			break;
		if (!loops++) {
			printk("!!!SIS Waiting before cmd %02x\n", cmd);
		}
	}
	if (loops)
		printk("!!!SIS waited %d loops\n", loops);

	__rxt1_span_framer_out(rxt1_card, span, FRMR_CMDR, cmd);
}

static int __rxt1_hdlc_start_chan(struct rxt1_card_t *rxt1_card, unsigned int span,
								  struct dahdi_chan *zap_chan, unsigned char mode)
{
	/* used in 2 places below */
	struct rxt1_span_t *rxt1_span = rxt1_card->rxt1_spans[span];

	unsigned char imr0, imr1;
	int offset = zap_chan->chanpos;

	if (debug & DEBUG_FRAMER)
		printk("Starting HDLC controller for channel %d span %d\n", offset, span + 1);

	if (mode != FRMR_MODE_NO_ADDR_CMP)
		return -1;

	mode |= FRMR_MODE_HRAC;

	/* Make sure we're in the right mode */
	__rxt1_span_framer_out(rxt1_card, span, FRMR_MODE, mode);
	__rxt1_span_framer_out(rxt1_card, span, FRMR_TSEO, 0x00);
	__rxt1_span_framer_out(rxt1_card, span, FRMR_TSBS1, 0xff);

	/* Set the interframe gaps, etc */
	__rxt1_span_framer_out(rxt1_card, span, FRMR_CCR1, FRMR_CCR1_ITF | FRMR_CCR1_EITS);

	__rxt1_span_framer_out(rxt1_card, span, FRMR_CCR2, FRMR_CCR2_RCRC);

	/* Set up the time slot that we want to tx/rx on */
	__rxt1_span_framer_out(rxt1_card, span, FRMR_TTR_BASE + (offset / 8),
						   (0x80 >> (offset % 8)));
	__rxt1_span_framer_out(rxt1_card, span, FRMR_RTR_BASE + (offset / 8),
						   (0x80 >> (offset % 8)));

	imr0 = __rxt1_span_framer_in(rxt1_card, span, FRMR_IMR0);
	imr1 = __rxt1_span_framer_in(rxt1_card, span, FRMR_IMR1);

	/* Enable our interrupts again */
	imr0 &= ~HDLC_IMR0_MASK;
	__rxt1_span_framer_out(rxt1_card, span, FRMR_IMR0, imr0);

	imr1 &= ~HDLC_IMR1_MASK;
	__rxt1_span_framer_out(rxt1_card, span, FRMR_IMR1, imr1);

	/* Reset the signaling controller */
	__rxt1_span_framer_cmd_wait(rxt1_card, span, FRMR_CMDR_SRES);

	rxt1_span->sigchan = zap_chan;
	rxt1_span->sigactive = 0;

	return 0;
}

static void __rxt1_span_set_clear(struct rxt1_card_t *rxt1_card, int span)
{
	int i, j;
	int oldnotclear;
	unsigned short val = 0;
	struct rxt1_span_t *rxt1_span = rxt1_card->rxt1_spans[span];

	oldnotclear = rxt1_span->notclear;
	if (rxt1_span->spantype == TYPE_T1) {
		for (i = 0; i < 24; i++) {
			j = (i / 8);
			if (rxt1_span->span.chans[i]->flags & DAHDI_FLAG_CLEAR) {
				val |= 1 << (7 - (i % 8));
				rxt1_span->notclear &= ~(1 << i);
			} else
				rxt1_span->notclear |= (1 << i);
			if ((i % 8) == 7) {
				if (debug & DEBUG_REGS)
					printk("SET CLEAR Putting %d in register %02x on span %d\n",
						   val, 0x2f + j, span + 1);
				__rxt1_span_framer_out(rxt1_card, span, 0x2f + j, val);
				val = 0;
			}
		}
	} else {
		for (i = 0; i < 31; i++) {
			if (rxt1_span->span.chans[i]->flags & DAHDI_FLAG_CLEAR)
				rxt1_span->notclear &= ~(1 << i);
			else
				rxt1_span->notclear |= (1 << i);
		}
	}
	if (rxt1_span->notclear != oldnotclear) {
		unsigned char reg;
		reg = __rxt1_span_framer_in(rxt1_card, span, FRMR_IMR0);
		if (rxt1_span->notclear)
			reg &= ~0x08;
		else
			reg |= 0x08;
		__rxt1_span_framer_out(rxt1_card, span, FRMR_IMR0, reg);
	}
}

static int rxt1_zap_chan_dacs(struct dahdi_chan *zap_chan_dst,
							  struct dahdi_chan *zap_chan_src)
{
	struct dahdi_span *span_dst = zap_chan_dst->span;
	struct rxt1_span_t *rxt1_span_dst = container_of(span_dst, struct rxt1_span_t, span);
	struct rxt1_card_t *rxt1_card_dst = rxt1_span_dst->owner;

	if (zap_chan_src) {
		struct dahdi_span *span_src = zap_chan_src->span;
		struct rxt1_span_t *rxt1_span_src =
			container_of(span_src, struct rxt1_span_t, span);
		struct rxt1_card_t *rxt1_card_src = rxt1_span_src->owner;

		if (rxt1_card_src != rxt1_card_dst) {
			rxt1_card_tsi_unassign(rxt1_card_dst,
								   zap_chan_dst->span->offset, zap_chan_dst->chanpos);
			rxt1_card_tsi_unassign(rxt1_card_src,
								   zap_chan_src->span->offset, zap_chan_src->chanpos);
			return -1;
		}
		/* channels reside on same card, this card. */
		rxt1_card_tsi_assign(rxt1_card_dst,
							 zap_chan_src->span->offset,
							 zap_chan_src->chanpos,
							 zap_chan_dst->span->offset, zap_chan_dst->chanpos);
		if (debug)
			printk("Assigning channel %d/%d -> %d/%d!\n",
				   zap_chan_src->span->offset, zap_chan_src->chanpos,
				   zap_chan_dst->span->offset, zap_chan_dst->chanpos);

	} else {					/* called with bad source channel */
		rxt1_card_tsi_unassign(rxt1_card_dst,
							   zap_chan_dst->span->offset, zap_chan_dst->chanpos);
	}

	return 0;
}

static int rxt1_zap_chan_ioctl(struct dahdi_chan *zap_chan, unsigned int cmd,
							   unsigned long data)
{
	struct rxt1_regs regs;
	int x;
	size_t regs_size;
	struct dahdi_span *span = zap_chan->span;
	struct rxt1_span_t *rxt1_span = container_of(span, struct rxt1_span_t, span);
	struct rxt1_card_t *rxt1_card = rxt1_span->owner;

	regs_size = sizeof(regs);
	switch (cmd) {
	case RXT1_GET_REGS:
		for (x = 0; x < NUM_PCI; x++)
			regs.pci[x] = rxt1_card_pci_in(rxt1_card, x);
		for (x = 0; x < NUM_REGS; x++)
			regs.regs[x] = rxt1_span_framer_in(rxt1_card, zap_chan->span->offset, x);
		if (copy_to_user((struct rxt1_regs *) data, &regs, regs_size))
			return -EFAULT;
		break;
	default:
		return -ENOTTY;
	}
	return 0;
}

static void inline __rxt1_span_hdlc_xmit_fifo(struct rxt1_card_t *rxt1_card,
											  unsigned int span,
											  struct rxt1_span_t *rxt1_span)
{
	int res = 0;
	int i, size = 32;
	unsigned char buf[32];

#ifdef DAHDI_SIG_HARDHDLC
	res = dahdi_hdlc_getbuf(rxt1_span->sigchan, buf, &size);
#endif
	if (debug & DEBUG_FRAMER)
		printk("Got buffer sized %d and res %d for %d\n", size, res, span);
	if (size > 0) {
		rxt1_span->sigactive = 1;

		if (debug & DEBUG_FRAMER) {
			printk("TX(");
			for (i = 0; i < size; i++)
				printk((i ? " %02x" : "%02x"), buf[i]);
			printk(")\n");
		}

		for (i = 0; i < size; i++)
			__rxt1_span_framer_out(rxt1_card, span, FRMR_TXFIFO, buf[i]);

		if (res) {				/* End of message */
			if (debug & DEBUG_FRAMER)
				printk("transmiting XHF|XME\n");
			__rxt1_span_framer_cmd_wait(rxt1_card, span, FRMR_CMDR_XHF | FRMR_CMDR_XME);
#if 0
			rxt1_span->sigactive =
				(__rxt1_span_framer_in(rxt1_card, span, FRMR_SIS) & FRMR_SIS_XFW) ? 0 : 1;
#endif
			++rxt1_span->frames_out;
			if ((debug & DEBUG_FRAMER) && !(rxt1_span->frames_out & 0x0f))
				printk("Transmitted %d frames on span %d\n", rxt1_span->frames_out, span);
		} else {				/* Still more to transmit */
			if (debug & DEBUG_FRAMER)
				printk("transmiting XHF\n");
			__rxt1_span_framer_cmd_wait(rxt1_card, span, FRMR_CMDR_XHF);
		}
	} else if (res < 0)
		rxt1_span->sigactive = 0;
}

#ifdef DAHDI_SIG_HARDHDLC
static void rxt1_zap_chan_hdlc_hard_xmit(struct dahdi_chan *zap_chan)
{
	struct dahdi_span *span = zap_chan->span;
	struct rxt1_span_t *rxt1_span = container_of(span, struct rxt1_span_t, span);
	struct rxt1_card_t *rxt1_card = rxt1_span->owner;

	unsigned long flags;

	spin_lock_irqsave(&rxt1_card->reglock, flags);
	if (!rxt1_span->sigchan) {
		spin_unlock_irqrestore(&rxt1_card->reglock, flags);
		return;
	}
	if ((rxt1_span->sigchan == zap_chan) && !rxt1_span->sigactive)
		__rxt1_span_hdlc_xmit_fifo(rxt1_card, zap_chan->span->offset, rxt1_span);
	spin_unlock_irqrestore(&rxt1_card->reglock, flags);
}
#endif

static int rxt1_span_maint(struct dahdi_span *span, int cmd)
{
	struct rxt1_span_t *rxt1_span = container_of(span, struct rxt1_span_t, span);
	struct rxt1_card_t *rxt1_card = rxt1_span->owner;

	if (rxt1_span->spantype == TYPE_E1) {
		switch (cmd) {
		case DAHDI_MAINT_NONE:
			printk("XXX Turn off local and remote loops E1 XXX\n");
			break;
		case DAHDI_MAINT_LOCALLOOP:
			printk("XXX Turn on local loopback E1 XXX\n");
			break;
		case DAHDI_MAINT_REMOTELOOP:
			printk("XXX Turn on remote loopback E1 XXX\n");
			break;
		case DAHDI_MAINT_LOOPUP:
			printk("XXX Send loopup code E1 XXX\n");
			break;
		case DAHDI_MAINT_LOOPDOWN:
			printk("XXX Send loopdown code E1 XXX\n");
			break;
		case DAHDI_MAINT_LOOPSTOP:
			printk("XXX Stop sending loop codes E1 XXX\n");
			break;
		default:
			printk("R%dT1: Unknown E1 maint command: %d\n", rxt1_card->numspans, cmd);
			break;
		}
	} else {
		switch (cmd) {
		case DAHDI_MAINT_NONE:
			printk("XXX Turn off local and remote loops T1 XXX\n");
			break;
		case DAHDI_MAINT_LOCALLOOP:
			printk("XXX Turn on local loop and no remote loop XXX\n");
			break;
		case DAHDI_MAINT_REMOTELOOP:
			printk("XXX Turn on remote loopup XXX\n");
			break;
		case DAHDI_MAINT_LOOPUP:
			/* FMR5: Nothing but RBS mode */
			rxt1_span_framer_out(rxt1_card, span->offset, 0x21, 0x50);
			break;
		case DAHDI_MAINT_LOOPDOWN:
			/* FMR5: Nothing but RBS mode */
			rxt1_span_framer_out(rxt1_card, span->offset, 0x21, 0x60);
			break;
		case DAHDI_MAINT_LOOPSTOP:
			rxt1_span_framer_out(rxt1_card, span->offset, 0x21, 0x40);	/* FMR5: Nothing but RBS mode */
			break;
		default:
			printk("R%dT1: Unknown T1 maint command: %d\n", rxt1_card->numspans, cmd);
			break;
		}
	}
	return 0;
}

static int rxt1_zap_chan_rbsbits(struct dahdi_chan *zap_chan, int bits)
{
	u_char m, c;
	int k, n, b;
	struct dahdi_span *span = zap_chan->span;
	struct rxt1_span_t *rxt1_span = container_of(span, struct rxt1_span_t, span);
	struct rxt1_card_t *rxt1_card = rxt1_span->owner;

	k = zap_chan->span->offset;
	if (rxt1_span->spantype == TYPE_E1) {	/* do it E1 way */
		if (zap_chan->chanpos == 16) {
			return 0;
		}
		n = zap_chan->chanpos - 1;
		if (zap_chan->chanpos > 15)
			n--;
		b = (n % 15);
		c = rxt1_span->txsigs[b];
		m = (n / 15) << 2;		/* nibble selector */
		c &= (0xf << m);		/* keep the other nibble */
		c |= (bits & 0xf) << (4 - m);	/* put our new nibble here */
		rxt1_span->txsigs[b] = c;

		/* output them to the chip */
		__rxt1_span_framer_out(rxt1_card, k, 0x71 + b, c);

	} else if (rxt1_span->span.lineconfig & DAHDI_CONFIG_D4) {
		n = zap_chan->chanpos - 1;
		b = (n / 4);
		c = rxt1_span->txsigs[b];
		m = ((3 - (n % 4)) << 1);	/* nibble selector */
		c &= ~(0x3 << m);		/* keep the other nibble */
		c |= ((bits >> 2) & 0x3) << m;	/* put our new nibble here */
		rxt1_span->txsigs[b] = c;

		/* output them to the chip */
		__rxt1_span_framer_out(rxt1_card, k, 0x70 + b, c);
		__rxt1_span_framer_out(rxt1_card, k, 0x70 + b + 6, c);

	} else if (rxt1_span->span.lineconfig & DAHDI_CONFIG_ESF) {
		n = zap_chan->chanpos - 1;
		b = (n / 2);
		c = rxt1_span->txsigs[b];
		m = ((n % 2) << 2);		/* nibble selector */
		c &= (0xf << m);		/* keep the other nibble */
		c |= (bits & 0xf) << (4 - m);	/* put our new nibble here */
		rxt1_span->txsigs[b] = c;
		/* output them to the chip */
		__rxt1_span_framer_out(rxt1_card, k, 0x70 + b, c);
	}

	return 0;
}

static int rxt1_span_shutdown(struct dahdi_span *span)
{
	int span_num;
	int wasrunning;
	unsigned long flags;
	struct rxt1_span_t *rxt1_span = container_of(span, struct rxt1_span_t, span);
	struct rxt1_card_t *rxt1_card = rxt1_span->owner;

	span_num = span->offset + 1;
	if (span_num < 0) {
		printk("R%dT1: Span '%d' isn't us?\n", rxt1_card->numspans, span->spanno);
		return -1;
	}

	if (debug & DEBUG_MAIN)
		printk("Shutting down span %d (%s)\n", span->spanno, span->name);

	/* Stop HDLC controller if runned */
	if (rxt1_span->sigchan)
		__rxt1_span_hdlc_stop(rxt1_card, span->offset);

	spin_lock_irqsave(&rxt1_card->reglock, flags);
	wasrunning = span->flags & DAHDI_FLAG_RUNNING;

	span->flags &= ~DAHDI_FLAG_RUNNING;

	__rxt1_card_set_led(rxt1_card, span->offset, SPAN_OFF);

	if (((rxt1_card->numspans == 4) &&
		 (!(rxt1_card->rxt1_spans[0]->span.flags & DAHDI_FLAG_RUNNING)) &&
		 (!(rxt1_card->rxt1_spans[1]->span.flags & DAHDI_FLAG_RUNNING)) &&
		 (!(rxt1_card->rxt1_spans[2]->span.flags & DAHDI_FLAG_RUNNING)) &&
		 (!(rxt1_card->rxt1_spans[3]->span.flags & DAHDI_FLAG_RUNNING)))
		||
		((rxt1_card->numspans == 2) &&
		 (!(rxt1_card->rxt1_spans[0]->span.flags & DAHDI_FLAG_RUNNING)) &&
		 (!(rxt1_card->rxt1_spans[1]->span.flags & DAHDI_FLAG_RUNNING)))) {
		/* No longer in use, disable interrupts */
		printk("R%dT1: Disabling interrupts since there are no active spans\n",
			   rxt1_card->numspans);
		rxt1_card->stopdma = 1;
	} else
		rxt1_card->checktiming = 1;
	spin_unlock_irqrestore(&rxt1_card->reglock, flags);

	/* Wait for interrupt routine to shut itself down */
	msleep(10);
	if (wasrunning)
		rxt1_card->spansstarted--;

	if (debug & DEBUG_MAIN)
		printk("DAHDI Span %d (%s) shutdown\n", span->spanno, span->name);
	return 0;
}

static int rxt1_span_startup(struct dahdi_span *span);

static int rxt1_spanconfig(struct dahdi_span *span, struct dahdi_lineconfig *spanconfig)
{
	int i;
	struct rxt1_span_t *rxt1_span = container_of(span, struct rxt1_span_t, span);
	struct rxt1_card_t *rxt1_card = rxt1_span->owner;

	printk("About to enter DAHDI spanconfig!\n");
	if (debug & DEBUG_MAIN) {
		printk("R%dT1: Configuring DAHDI span %d\n", rxt1_card->numspans, span->spanno);
		printk("lineconfig %x , lbo %x , sync %x\n", spanconfig->lineconfig,
			   spanconfig->lbo, spanconfig->sync);

	}

	/* XXX We assume lineconfig is okay and shouldn't XXX */
	span->lineconfig = spanconfig->lineconfig;
	span->txlevel = spanconfig->lbo;
	span->rxlevel = 0;
	if (spanconfig->sync < 0)
		spanconfig->sync = 0;
	if (spanconfig->sync > 4)
		spanconfig->sync = 0;

	/* remove this span number from the current sync sources, if there */
	for (i = 0; i < rxt1_card->numspans; i++) {
		if (rxt1_card->rxt1_spans[i]->sync == span->spanno) {
			rxt1_card->rxt1_spans[i]->sync = 0;
			rxt1_card->rxt1_spans[i]->psync = 0;
		}
	}
	rxt1_card->rxt1_spans[span->offset]->syncpos = spanconfig->sync;
	/* if a sync src, put it in proper place */
	if (spanconfig->sync) {
		rxt1_card->rxt1_spans[spanconfig->sync - 1]->sync = span->spanno;
		rxt1_card->rxt1_spans[spanconfig->sync - 1]->psync = span->offset + 1;
	}
	rxt1_card->checktiming = 1;

	/* If we're already running, then go ahead and apply the changes */
	if (span->flags & DAHDI_FLAG_RUNNING)
		return rxt1_span_startup(span);
	printk("Done with spanconfig!\n");
	return 0;
}

static int rxt1_zap_chanconfig(struct dahdi_chan *zap_chan, int sigtype)
{
	int alreadyrunning;

	struct dahdi_span *span = zap_chan->span;
	struct rxt1_span_t *rxt1_span = container_of(span, struct rxt1_span_t, span);
	struct rxt1_card_t *rxt1_card = rxt1_span->owner;

	alreadyrunning = rxt1_span->span.flags & DAHDI_FLAG_RUNNING;
	if (debug & DEBUG_MAIN) {
		if (alreadyrunning)
			printk("R%dT1: Reconfigured channel %d (%s) sigtype %d\n",
				   rxt1_card->numspans, zap_chan->channo, zap_chan->name, sigtype);
		else
			printk("R%dT1: Configured channel %d (%s) sigtype %d\n", rxt1_card->numspans,
				   zap_chan->channo, zap_chan->name, sigtype);
	}

	if (alreadyrunning)
		__rxt1_span_set_clear(rxt1_card, zap_chan->span->offset);

	/* (re)configure signalling channel */
#ifdef DAHDI_SIG_HARDHDLC
	if ((sigtype == DAHDI_SIG_HARDHDLC) || (rxt1_span->sigchan == zap_chan)) {
#else
	if (rxt1_span->sigchan == zap_chan) {
#endif
#ifdef DAHDI_SIG_HARDHDLC
		if (debug & DEBUG_FRAMER)
			printk("%sonfiguring hardware HDLC on %s\n",
				   ((sigtype == DAHDI_SIG_HARDHDLC) ? "C" : "Unc"), zap_chan->name);
#endif
		if (alreadyrunning) {
			if (rxt1_span->sigchan)
				__rxt1_span_hdlc_stop(rxt1_card, rxt1_span->sigchan->span->offset);
#ifdef DAHDI_SIG_HARDHDLC
			if (sigtype == DAHDI_SIG_HARDHDLC) {
				if (__rxt1_hdlc_start_chan
					(rxt1_card, zap_chan->span->offset, zap_chan, rxt1_span->sigmode)) {
					printk("Error initializing signalling controller\n");
					return -1;
				}
			} else
#endif
				rxt1_span->sigchan = NULL;
		} else {
#ifdef DAHDI_SIG_HARDHDLC
			rxt1_span->sigchan = (sigtype == DAHDI_SIG_HARDHDLC) ? zap_chan : NULL;
#else
			rxt1_span->sigchan = NULL;
#endif

			rxt1_span->sigactive = 0;
		}
	}
	return 0;
}

static int rxt1_zap_chan_open(struct dahdi_chan *zap_chan)
{
	try_module_get(THIS_MODULE);
	return 0;
}

static int rxt1_zap_chan_close(struct dahdi_chan *zap_chan)
{
	module_put(THIS_MODULE);
	return 0;
}

static struct dahdi_span_ops ops_with_echocan = {
	.spanconfig = rxt1_spanconfig,
	.chanconfig = rxt1_zap_chanconfig,
	.startup = rxt1_span_startup,
	.shutdown = rxt1_span_shutdown,
	.rbsbits = rxt1_zap_chan_rbsbits,
	.maint = rxt1_span_maint,
	.open = rxt1_zap_chan_open,
	.close = rxt1_zap_chan_close,
	.ioctl = rxt1_zap_chan_ioctl,
	.dacs = rxt1_zap_chan_dacs,
	.echocan_create = rxt1_echocan_create,
#ifdef DAHDI_SIG_HARDHDLC
	.hdlc_hard_xmit = rxt1_zap_chan_hdlc_hard_xmit,
#endif
	.owner = THIS_MODULE,
};

static struct dahdi_span_ops ops_without_echocan = {
	.spanconfig = rxt1_spanconfig,
	.chanconfig = rxt1_zap_chanconfig,
	.startup = rxt1_span_startup,
	.shutdown = rxt1_span_shutdown,
	.rbsbits = rxt1_zap_chan_rbsbits,
	.maint = rxt1_span_maint,
	.open = rxt1_zap_chan_open,
	.close = rxt1_zap_chan_close,
	.ioctl = rxt1_zap_chan_ioctl,
	.dacs = rxt1_zap_chan_dacs,
#ifdef DAHDI_SIG_HARDHDLC
	.hdlc_hard_xmit = rxt1_zap_chan_hdlc_hard_xmit,
#endif
	.owner = THIS_MODULE,
};

static void rxt1_card_init_spans(struct rxt1_card_t *rxt1_card)
{
	int span_num, chan_num /*,c */ ;
	int offset = 1;
	struct rxt1_span_t *rxt1_span;

	for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
		rxt1_span = rxt1_card->rxt1_spans[span_num];
		sprintf(rxt1_span->span.name, "R%dT1/%d/%d", rxt1_card->numspans,
				rxt1_card->num, span_num + 1);
		sprintf(rxt1_span->span.desc, "R%dT1 (PCI) Card %d Span %d", rxt1_card->numspans,
				rxt1_card->num, span_num + 1);
		rxt1_span->span.manufacturer = "Rhino Equipment";


		/* HDLC Specific init */
		rxt1_span->sigchan = NULL;
		rxt1_span->sigmode = sigmode;
		rxt1_span->sigactive = 0;

		if (rxt1_span->spantype == TYPE_T1 || rxt1_span->spantype == TYPE_J1) {
			rxt1_span->span.channels = 24;
			rxt1_span->span.deflaw = DAHDI_LAW_MULAW;
		} else {
			rxt1_span->span.channels = 31;
			rxt1_span->span.deflaw = DAHDI_LAW_ALAW;
		}

#ifdef DAHDI_GT_1471
		switch (rxt1_span->spantype) {
		case TYPE_T1:
			rxt1_span->span.spantype = "T1";

			break;
		case TYPE_E1:
			rxt1_span->span.spantype = "E1";

			break;
		case TYPE_J1:
			rxt1_span->span.spantype = "J1";

			break;
		}
#endif

		switch (rxt1_span->spantype) {
		case TYPE_T1:
			rxt1_span->span.linecompat =
				DAHDI_CONFIG_AMI | DAHDI_CONFIG_B8ZS | DAHDI_CONFIG_D4 | DAHDI_CONFIG_ESF;
			break;
		case TYPE_E1:
			rxt1_span->span.linecompat =
				DAHDI_CONFIG_AMI | DAHDI_CONFIG_HDB3 | DAHDI_CONFIG_CCS |
				DAHDI_CONFIG_CRC4;
			break;
		case TYPE_J1:
			rxt1_span->span.linecompat =
				DAHDI_CONFIG_AMI | DAHDI_CONFIG_B8ZS | DAHDI_CONFIG_D4 | DAHDI_CONFIG_ESF;
			break;
		}

		rxt1_span->span.chans = rxt1_span->chans;
		rxt1_span->span.flags = DAHDI_FLAG_RBS;


		rxt1_span->owner = rxt1_card;
		rxt1_span->span.offset = span_num;
		rxt1_span->writechunk = (void *) (rxt1_card->writechunk + span_num * 32 * 2);
		rxt1_span->readchunk = (void *) (rxt1_card->readchunk + span_num * 32 * 2);
		if (show_pointers == 1)
			printk("Span %d writechunk %x readchunk %x\n", span_num,
				   (__u32) rxt1_span->writechunk, (__u32) rxt1_span->readchunk);
		init_waitqueue_head(&rxt1_span->span.maintq);
		for (chan_num = 0; chan_num < rxt1_card->rxt1_spans[span_num]->span.channels;
			 chan_num++) {
			sprintf(rxt1_span->chans[chan_num]->name, "R%dT1/%d/%d/%d",
					rxt1_card->numspans, rxt1_card->num, span_num + 1, chan_num + 1);
			rxt1_span->chans[chan_num]->sigcap =
				DAHDI_SIG_EM | DAHDI_SIG_CLEAR | DAHDI_SIG_FXSLS | DAHDI_SIG_FXSGS |
				DAHDI_SIG_FXSKS |
#ifdef DAHDI_SIG_HARDHDLC
				DAHDI_SIG_HARDHDLC |
#endif
#ifdef DAHDI_SIG_FXONS
				DAHDI_SIG_FXONS |
#endif
				DAHDI_SIG_FXOLS | DAHDI_SIG_FXOGS | DAHDI_SIG_FXOKS | DAHDI_SIG_CAS |
				DAHDI_SIG_EM_E1 | DAHDI_SIG_DACS_RBS;

			rxt1_span->chans[chan_num]->pvt = rxt1_card;
			rxt1_span->chans[chan_num]->chanpos = chan_num + 1;
			rxt1_span->chans[chan_num]->writechunk =
				(void *) (rxt1_card->writechunk +
						  (span_num * 32 + chan_num + offset) * 2);
			rxt1_span->chans[chan_num]->readchunk =
				(void *) (rxt1_card->readchunk + (span_num * 32 + chan_num + offset) * 2);
			rxt1_span->chans[chan_num]->span = &rxt1_span->span;

			if (show_pointers)
				printk("R%dT1: Span %d Chan %d writechunk %p readchunk %p\n",
					   rxt1_card->numspans, span_num, chan_num,
					   rxt1_span->chans[chan_num]->writechunk,
					   rxt1_span->chans[chan_num]->readchunk);
		}
	}
}

static void rxt1_span_serial_setup(struct rxt1_card_t *rxt1_card, int span)
{
	unsigned char lim0 = 8;
	lim0 |= (__rxt1_span_framer_in(rxt1_card, span, 0x36) & 1);
	if (!rxt1_card->globalconfig) {	/* just do glabal section once */
		rxt1_card->globalconfig = 1;
		printk("R%dT1: Setting up global serial parameters on Span %d\n",
			   rxt1_card->numspans, span);
		/* GPC1: Multiplex mode enabled, FSC is output, active low, RCLK from channel 0 */
		rxt1_span_framer_out(rxt1_card, 0, 0x85, 0xe0);
		/* IPC: Interrupt push/pull active low */
		rxt1_span_framer_out(rxt1_card, 0, 0x08, 0x01);

		/* Global clocks (8.192 Mhz CLK) */
		rxt1_span_framer_out(rxt1_card, 0, 0x92, 0x00);
		rxt1_span_framer_out(rxt1_card, 0, 0x93, 0x18);
		rxt1_span_framer_out(rxt1_card, 0, 0x94, 0xfb);
		rxt1_span_framer_out(rxt1_card, 0, 0x95, 0x0b);
		rxt1_span_framer_out(rxt1_card, 0, 0x96, 0x00);
		rxt1_span_framer_out(rxt1_card, 0, 0x97, 0x0b);
		rxt1_span_framer_out(rxt1_card, 0, 0x98, 0xdb);
		rxt1_span_framer_out(rxt1_card, 0, 0x99, 0xdf);

		if (rxt1_card->numspans == 2) {
			rxt1_span_framer_out(rxt1_card, 2, FRMR_LIM1, 0x09);	/* 137 */
			rxt1_span_framer_out(rxt1_card, 2, FRMR_LIM0, 0x02);	/* 136 */
			rxt1_span_framer_out(rxt1_card, 2, FRMR_XSP, 0x20);	/* 121 */
			rxt1_span_framer_out(rxt1_card, 2, FRMR_FMR2, 0x10);	/* 11e */
			rxt1_span_framer_out(rxt1_card, 2, FRMR_IDLE, 0x00);	/* 12b */
			rxt1_span_framer_out(rxt1_card, 2, FRMR_ICB1, 0xff);	/* 132 */
			rxt1_span_framer_out(rxt1_card, 2, FRMR_ICB2, 0xff);	/* 133 */
			rxt1_span_framer_out(rxt1_card, 2, FRMR_ICB3, 0xff);	/* 134 */
			rxt1_span_framer_out(rxt1_card, 2, FRMR_ICB4, 0xff);	/* 135 */
			rxt1_span_framer_out(rxt1_card, 2, FRMR_CMR2, 0x0c);	/* 145 */

			rxt1_span_framer_out(rxt1_card, 2, FRMR_SIC2, 0x20 | (2 << 1));
			rxt1_span_framer_out(rxt1_card, 2, FRMR_XC0, 0x00);
			rxt1_span_framer_out(rxt1_card, 2, FRMR_XC1, xmit_off);
			rxt1_span_framer_out(rxt1_card, 2, FRMR_RC0, 0x00);
			rxt1_span_framer_out(rxt1_card, 2, FRMR_RC1, recd_off);

			rxt1_span_framer_out(rxt1_card, 3, FRMR_LIM1, 0x09);	/* 337 */
			rxt1_span_framer_out(rxt1_card, 3, FRMR_LIM0, 0x02);	/* 336 */
			rxt1_span_framer_out(rxt1_card, 3, FRMR_XSP, 0x20);	/* 321 */
			rxt1_span_framer_out(rxt1_card, 3, FRMR_FMR2, 0x10);	/* 31e */
			rxt1_span_framer_out(rxt1_card, 3, FRMR_IDLE, 0x00);	/* 32b */
			rxt1_span_framer_out(rxt1_card, 3, FRMR_ICB1, 0xff);	/* 332 */
			rxt1_span_framer_out(rxt1_card, 3, FRMR_ICB2, 0xff);	/* 333 */
			rxt1_span_framer_out(rxt1_card, 3, FRMR_ICB3, 0xff);	/* 334 */
			rxt1_span_framer_out(rxt1_card, 3, FRMR_ICB4, 0xff);	/* 335 */
			rxt1_span_framer_out(rxt1_card, 3, FRMR_CMR2, 0x0c);	/* 345 */

			rxt1_span_framer_out(rxt1_card, 3, FRMR_SIC2, 0x20 | (3 << 1));
			rxt1_span_framer_out(rxt1_card, 3, FRMR_XC0, 0x00);
			rxt1_span_framer_out(rxt1_card, 3, FRMR_XC1, xmit_off);
			rxt1_span_framer_out(rxt1_card, 3, FRMR_RC0, 0x00);
			rxt1_span_framer_out(rxt1_card, 3, FRMR_RC1, recd_off);
		}
	}

	/* Configure interrupts */
	/* GCR: Interrupt on Activation/Deactivation of each */
	rxt1_span_framer_out(rxt1_card, span, FRMR_GCR, 0x00);

	if (local_loop == 1)
		lim0 |= 2;
	if (monitor_mode & (1 << span))
		lim0 |= 4;
	if (gen_clk & (1 << span))
		lim0 |= 1;

	if (debug & DEBUG_FRAMER)
		printk("Span %x LIM0 %X\n", span, lim0);
	/* LIM0: Enable auto long haul mode, no local loop (must be after LIM1) */
	rxt1_span_framer_out(rxt1_card, span, 0x36, lim0);

	if (insert_idle == 1) {
		if (span == 0) {
			rxt1_span_framer_out(rxt1_card, span, FRMR_IDLE, 0x00);	/* IDLE code */
			rxt1_span_framer_out(rxt1_card, span, FRMR_ICB1, 0x00);	/* IDLE enable slot 7-0  */
			rxt1_span_framer_out(rxt1_card, span, FRMR_ICB2, 0xff);	/* ... 15-8 */
			rxt1_span_framer_out(rxt1_card, span, FRMR_ICB3, 0xff);	/* ... 23-16 */
			rxt1_span_framer_out(rxt1_card, span, FRMR_ICB4, 0xff);	/* ... 31-24 */
		}
		if (span == 1) {
			rxt1_span_framer_out(rxt1_card, span, FRMR_IDLE, 0xff);	/* IDLE code for test */
			rxt1_span_framer_out(rxt1_card, span, FRMR_ICB1, 0xff);	/* IDLE enable slot 7-0 */
			rxt1_span_framer_out(rxt1_card, span, FRMR_ICB2, 0xff);	/* ... 15-8 */
			rxt1_span_framer_out(rxt1_card, span, FRMR_ICB3, 0xff);	/* ... 23-16 */
			rxt1_span_framer_out(rxt1_card, span, FRMR_ICB4, 0xff);	/* ... 31-24 */
		}
		if (span == 2) {
			rxt1_span_framer_out(rxt1_card, span, FRMR_IDLE, 0xff);	/* IDLE code for test */
			rxt1_span_framer_out(rxt1_card, span, FRMR_ICB1, 0xff);	/* IDLE enable slot 7-0 */
			rxt1_span_framer_out(rxt1_card, span, FRMR_ICB2, 0xff);	/* ... 15-8 */
			rxt1_span_framer_out(rxt1_card, span, FRMR_ICB3, 0xff);	/* ... 23-16 */
			rxt1_span_framer_out(rxt1_card, span, FRMR_ICB4, 0xff);	/* ... 31-24 */
		}
		if (span == 3) {
			rxt1_span_framer_out(rxt1_card, span, FRMR_IDLE, 0xff);	/* IDLE code for test */
			rxt1_span_framer_out(rxt1_card, span, FRMR_ICB1, 0xff);	/* IDLE enable slot 7-0 */
			rxt1_span_framer_out(rxt1_card, span, FRMR_ICB2, 0xff);	/* ... 15-8 */
			rxt1_span_framer_out(rxt1_card, span, FRMR_ICB3, 0xff);	/* ... 23-16 */
			rxt1_span_framer_out(rxt1_card, span, FRMR_ICB4, 0xff);	/* ... 31-24 */
		}
	} else {
		rxt1_span_framer_out(rxt1_card, span, FRMR_IDLE, 0xff);	/* IDLE code */
		rxt1_span_framer_out(rxt1_card, span, FRMR_ICB1, 0x00);	/* IDLE enable slot 7-0 */
		rxt1_span_framer_out(rxt1_card, span, FRMR_ICB2, 0x00);	/* ... 15-8 */
		rxt1_span_framer_out(rxt1_card, span, FRMR_ICB3, 0x00);	/* ... 23-16 */
		rxt1_span_framer_out(rxt1_card, span, FRMR_ICB4, 0x00);	/* ... 31-24 */
	}

	/* Configure system interface */
	/* SIC1: 8.192 Mhz clock/bus, double buffer receive / transmit, byte interleaved */
	rxt1_span_framer_out(rxt1_card, span, FRMR_SIC1, 0xc2);
	/* SIC2: No FFS, no center receive eliastic buffer, phase */
	rxt1_span_framer_out(rxt1_card, span, FRMR_SIC2, 0x20 | ((span + 0) << 1));
	/* SIC3: Edges for capture */
	rxt1_span_framer_out(rxt1_card, span, FRMR_SIC3, sic3_set);
	/* CMR2: We provide sync and clock for tx and rx. */
	rxt1_span_framer_out(rxt1_card, span, FRMR_CMR2, 0x00);
	if (!rxt1_card->t1e1) {		/* T1 mode */
		/* XC0: Normal operation of Sa-bits */
		rxt1_span_framer_out(rxt1_card, span, FRMR_XC0, 0x00);
		/* XC1: 0 offset */
		rxt1_span_framer_out(rxt1_card, span, FRMR_XC1, xmit_off);
		if (rxt1_card->rxt1_spans[span]->spantype == TYPE_J1)
			/* RC0: Just shy of 1023 */
			rxt1_span_framer_out(rxt1_card, span, FRMR_RC0, 0x80);
		else
			/* RC0: Just shy of 1023 */
			rxt1_span_framer_out(rxt1_card, span, FRMR_RC0, 0x00);

		if (rxt1_card->numspans == 2)
			/* RC1: The rest of RC0 */
			rxt1_span_framer_out(rxt1_card, span, FRMR_RC1, recd_off);
		else
			/* RC1: The rest of RC0 */
			rxt1_span_framer_out(rxt1_card, span, FRMR_RC1, recq_off);
	} else {					/* E1 mode */
		/* XC0: Normal operation of Sa-bits */
		rxt1_span_framer_out(rxt1_card, span, FRMR_XC0, 0x00);
		/* XC1: 0 offset */
		rxt1_span_framer_out(rxt1_card, span, FRMR_XC1, xmit_off);
		/* RC0: Just shy of 1023 */
		rxt1_span_framer_out(rxt1_card, span, FRMR_RC0, 0x00);
		if (rxt1_card->numspans == 2)
			/* RC1: The rest of RC0 */
			rxt1_span_framer_out(rxt1_card, span, FRMR_RC1, recd_off);
		else
			/* RC1: The rest of RC0 */
			rxt1_span_framer_out(rxt1_card, span, FRMR_RC1, recq_off);
	}

	/* Configure ports */
	/* PC1: SPYR/SPYX input on RPA/XPA */
	rxt1_span_framer_out(rxt1_card, span, 0x80, 0x00);
	/* PC2: RMFB/XSIG output/input on RPB/XPB */
	rxt1_span_framer_out(rxt1_card, span, 0x81, 0xf7);
	/* PC3: Some unused stuff */
	rxt1_span_framer_out(rxt1_card, span, 0x82, 0xf7);
	/* PC4: Some more unused stuff */
	rxt1_span_framer_out(rxt1_card, span, 0x83, 0xf7);
	/* PC5: XMFS active low, SCLKR is input, RCLK is output */
	rxt1_span_framer_out(rxt1_card, span, 0x84, 0x01);
	if (debug & DEBUG_MAIN)
		printk("Successfully initialized serial bus for span %d\n", span);
}

static int syncsrc = 3;			/* Zaptel span number */
static int syncnum = 0 /* -1 */ ;	/* rxt1 card number */
static int syncspan = 0;		/* span on given rxt1 card */
#ifdef DEFINE_SPINLOCK
static DEFINE_SPINLOCK(synclock);
#else
static spinlock_t synclock = SPIN_LOCK_UNLOCKED;
#endif

static void __rxt1_card_set_timing_source(struct rxt1_card_t *rxt1_card, int src_span,
										  int master, int slave)
{
	unsigned int timing;
	int span_num;
	if (src_span != rxt1_card->syncsrc) {
		/* CMR1: RCLK src_span, 8.192 Mhz TCLK, RCLK is 8.192 Mhz */
		timing = 0x34;
		if ((src_span > -1) && (src_span < 4)) {
			if (rxt1_card->numspans == 4)
				timing |= (src_span << 6);
			else
				timing |= (src_span << 7);

			/* set all 4 receive reference clocks to src_span */
			for (span_num = 0; span_num < rxt1_card->numspans; span_num++)
				__rxt1_span_framer_out(rxt1_card, span_num, 0x44, timing);
		} else {
			/* set each receive reference clock to itself */
			for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
				if (rxt1_card->numspans == 4)
					__rxt1_span_framer_out(rxt1_card, span_num, 0x44,
										   timing | (span_num << 6));
				else
					__rxt1_span_framer_out(rxt1_card, span_num, 0x44,
										   timing | (span_num << 7));
			}
		}

		if (!master && !slave)
			rxt1_card->syncsrc = src_span;
		if ((src_span < 0) || (src_span > 3))
			src_span = 0;
		else
			src_span++;
		if (!master && !slave) {
			for (span_num = 0; span_num < rxt1_card->numspans; span_num++)
				rxt1_card->rxt1_spans[span_num]->span.syncsrc = src_span;
		}
	} else {
		/* already set */
		if (debug & DEBUG_MAIN)
			printk("r%d11: Set Timing source already set to %d\n", rxt1_card->numspans,
				   src_span);
	}
	if (debug && DEBUG_MAIN) {
		printk("R%dT1_card: Set Timing source set to %d master %d slave %d\n",
			   rxt1_card->numspans, src_span, master, slave);
		printk("R%dT1_card: Zap Span Timing Src %d on Card %d Span %d\n",
			   rxt1_card->numspans, syncsrc, syncnum, syncspan);
	}
}

static inline void __rxt1_card_update_timing(struct rxt1_card_t *rxt1_card)
{
	int span_num;
	/* update sync src info */
	if (debug && DEBUG_MAIN) {
		printk("R%dT1_card: Update Timing source set to %d\n", rxt1_card->numspans,
			   rxt1_card->syncsrc);
		printk("R%dT1_card: Zap Span Timing Src %d on Card %d Span %d\n",
			   rxt1_card->numspans, syncsrc, syncnum, syncspan);
	}

	if (rxt1_card->syncsrc != syncsrc) {
		printk("R%dT1: Swapping card %d from %d to %d\n", rxt1_card->numspans,
			   rxt1_card->num, rxt1_card->syncsrc, syncsrc);
		rxt1_card->syncsrc = syncsrc;
		/* Update sync sources */
		for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
			rxt1_card->rxt1_spans[span_num]->span.syncsrc = rxt1_card->syncsrc;
		}
		if (syncnum == rxt1_card->num) {	/*  M  S */
			__rxt1_card_set_timing_source(rxt1_card, syncspan - 1, 1, 0);
			if (debug)
				printk("R%dT1 card %d, using sync span %d, master\n", rxt1_card->numspans,
					   rxt1_card->num, syncspan);
		} else {				/*  M  S */
			__rxt1_card_set_timing_source(rxt1_card, syncspan - 1, 0, 1);
			if (debug)
				printk("R%dT1 card %d, using Timing Bus, NOT master\n",
					   rxt1_card->numspans, rxt1_card->num);
		}
	}
}

static int __rxt1_card_findsync(struct rxt1_card_t *rxt1_card)
{
	int i;
	int x;
	unsigned long flags;
	int p;
	int nonzero;
	int newsyncsrc = 0;			/* Zaptel span number */
	int newsyncnum = 0;			/* rxt1 card number */
	int newsyncspan = 0;		/* span on given rxt1 card */
	spin_lock_irqsave(&synclock, flags);

	if (!rxt1_card->num) {
		/* If we're the first card, go through all the motions, up to 8 levels
		   of sync source */
		p = 1;
		while (p < 8) {
			nonzero = 0;
			for (x = 0; rxt1_cards[x]; x++) {
				for (i = 0; i < rxt1_card->numspans; i++) {
					if (rxt1_cards[x]->rxt1_spans[i]->syncpos) {
						nonzero = 1;
						if ((rxt1_cards[x]->rxt1_spans[i]->syncpos == p) &&
							!(rxt1_cards[x]->rxt1_spans[i]->
							  span.alarms & (DAHDI_ALARM_RED | DAHDI_ALARM_BLUE |
											 DAHDI_ALARM_LOOPBACK)) &&
							(rxt1_cards[x]->rxt1_spans[i]->
							 span.flags & DAHDI_FLAG_RUNNING)) {
							/* This makes a good sync source */
							newsyncsrc = rxt1_cards[x]->rxt1_spans[i]->span.spanno;
							newsyncnum = x;
							newsyncspan = i + 1;
							/* Jump out */
							goto found;
						}
					}
				}
			}
			if (nonzero)
				p++;
			else
				break;
		}
	  found:
		if ((syncnum != newsyncnum) || (syncsrc != newsyncsrc) ||
			(newsyncspan != syncspan)) {
			syncnum = newsyncnum;
			syncsrc = newsyncsrc;
			syncspan = newsyncspan;
			for (x = 0; rxt1_cards[x]; x++) {
				__rxt1_card_update_timing(rxt1_cards[x]);
			}
		}
	} else
		rxt1_cards[0]->checktiming = 1;

	spin_unlock_irqrestore(&synclock, flags);

	if (debug && DEBUG_MAIN) {
		printk("R%dT1_card: Find Timing source set to %d\n", rxt1_card->numspans,
			   rxt1_card->syncsrc);
		printk("R%dT1_card: Zap Span Timing Src %d on Card %d Span %d\n",
			   rxt1_card->numspans, syncsrc, syncnum, syncspan);
	}

	return 0;
}

static void __rxt1_card_set_timing_source_auto(struct rxt1_card_t *rxt1_card)
{
	int span_num;
	if (debug && DEBUG_MAIN)
		printk("R%dT1: Timing source auto card %d!\n", rxt1_card->numspans,
			   rxt1_card->num);

	rxt1_card->checktiming = 0;
	if (timingcable) {
		__rxt1_card_findsync(rxt1_card);
	} else {
		for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
			if (rxt1_card->rxt1_spans[span_num]->sync) {
				if ((rxt1_card->rxt1_spans[rxt1_card->rxt1_spans[span_num]->psync -
										   1]->span.flags & DAHDI_FLAG_RUNNING) &&
					!(rxt1_card->rxt1_spans[rxt1_card->rxt1_spans[span_num]->psync -
											1]->span.
					  alarms & (DAHDI_ALARM_RED | DAHDI_ALARM_BLUE))) {
					/* Valid timing source *//*  M  S */
					__rxt1_card_set_timing_source(rxt1_card,
												  rxt1_card->rxt1_spans[span_num]->psync -
												  1, 0, 0);
					return;
				}
			}
		}						/*  X  M  S */
		__rxt1_card_set_timing_source(rxt1_card, 4, 0, 0);
	}
	if (debug && DEBUG_MAIN) {
		printk("R%dT1_card: Set auto Timing source set to %d\n", rxt1_card->numspans,
			   rxt1_card->syncsrc);
		printk("R%dT1_card: Zap Span Timing Src %d on Card %d Span %d\n",
			   rxt1_card->numspans, syncsrc, syncnum, syncspan);
	}
}

static void __rxt1_span_configure_t1(struct rxt1_card_t *rxt1_card, int span,
									 int lineconfig, int txlevel)
{
	unsigned int fmr4, fmr2, fmr1, fmr0, lim2, lim0;
	char *framing, *line;
	int mytxlevel;
	if ((txlevel > 7) || (txlevel < 4))
		mytxlevel = 0;
	else
		mytxlevel = txlevel - 4;
	/* FMR1: Mode 1, T1 mode, CRC on for ESF, 8.192 Mhz system data rate, no XAIS */
	fmr1 = 0x9c;
	/* FMR2: no payload loopback, auto send yellow alarm */
	fmr2 = 0x22;
	if (loopback)
		fmr2 |= 0x4;
	/* FMR4: Lose sync on 2 out of 5 framing bits, auto resync */
	fmr4 = 0x0c;
	/* LIM2: 50% peak is a "1", Advanced Loss recovery */
	lim2 = 0x21;
	/* LIM2: Add line buildout */
	lim2 |= (mytxlevel << 6);
	__rxt1_span_framer_out(rxt1_card, span, 0x1d, fmr1);
	__rxt1_span_framer_out(rxt1_card, span, 0x1e, fmr2);

	/* Configure line interface */
	if (lineconfig & DAHDI_CONFIG_AMI) {
		line = "AMI";
		fmr0 = 0xf0;
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
	lim0 = 8 | (__rxt1_span_framer_in(rxt1_card, span, 0x36) & 1);
	if (local_loop == 1)
		lim0 |= 2;
	if (monitor_mode & (1 << span))
		lim0 |= 4;
	__rxt1_span_framer_out(rxt1_card, span, 0x1c, fmr0);
	__rxt1_span_framer_out(rxt1_card, span, 0x20, fmr4);
	/* FMR5: Enable RBS mode */
	__rxt1_span_framer_out(rxt1_card, span, 0x21, 0x40);

	/* LIM1: Clear data in case of LOS, Set receiver threshold (0.5V), 
	 *      No remote loop, no DRS */
	__rxt1_span_framer_out(rxt1_card, span, 0x37, 0xf8);
	/* LIM0: Enable auto long haul mode, no local loop (must be after LIM1) */
	__rxt1_span_framer_out(rxt1_card, span, 0x36, lim0);

	/* CMDR: Reset the receiver and transmitter line interface */
	__rxt1_span_framer_out(rxt1_card, span, 0x02, 0x50);
	/* CMDR: Reset the receiver and transmitter line interface */
	__rxt1_span_framer_out(rxt1_card, span, 0x02, 0x00);

	/* LIM2: 50% peak amplitude is a "1" */
	__rxt1_span_framer_out(rxt1_card, span, 0x3a, lim2);
	/* PCD: LOS after 176 consecutive "zeros" */
	__rxt1_span_framer_out(rxt1_card, span, 0x38, 0x0a);
	/* PCR: 22 "ones" clear LOS */
	__rxt1_span_framer_out(rxt1_card, span, 0x39, 0x15);

	/* Generate pulse mask for T1 */
	switch (mytxlevel) {
	case 3:
		__rxt1_span_framer_out(rxt1_card, span, 0x26, 0x07);	/* XPM0 */
		__rxt1_span_framer_out(rxt1_card, span, 0x27, 0x01);	/* XPM1 */
		__rxt1_span_framer_out(rxt1_card, span, 0x28, 0x00);	/* XPM2 */
		break;
	case 2:
		__rxt1_span_framer_out(rxt1_card, span, 0x26, 0x8c);	/* XPM0 */
		__rxt1_span_framer_out(rxt1_card, span, 0x27, 0x11);	/* XPM1 */
		__rxt1_span_framer_out(rxt1_card, span, 0x28, 0x01);	/* XPM2 */
		break;
	case 1:
		__rxt1_span_framer_out(rxt1_card, span, 0x26, 0x8c);	/* XPM0 */
		__rxt1_span_framer_out(rxt1_card, span, 0x27, 0x01);	/* XPM1 */
		__rxt1_span_framer_out(rxt1_card, span, 0x28, 0x00);	/* XPM2 */
		break;
	case 0:
	default:
		__rxt1_span_framer_out(rxt1_card, span, 0x26, 0xd7);	/* XPM0 */
		__rxt1_span_framer_out(rxt1_card, span, 0x27, 0x22);	/* XPM1 */
		__rxt1_span_framer_out(rxt1_card, span, 0x28, 0x01);	/* XPM2 */
		break;
	}

	/* Don't mask framer interrupts if hardware HDLC is in use */
	/* IMR0: We care about CAS changes, etc */
	__rxt1_span_framer_out(rxt1_card, span, FRMR_IMR0,
						   0xff & ~((rxt1_card->rxt1_spans[span]->sigchan) ?
									HDLC_IMR0_MASK : 0));
	/* IMR1: We care about nothing */
	__rxt1_span_framer_out(rxt1_card, span, FRMR_IMR1,
						   0xff & ~((rxt1_card->rxt1_spans[span]->sigchan) ?
									HDLC_IMR1_MASK : 0));
	/* IMR2: We care about all the alarm stuff! */
	__rxt1_span_framer_out(rxt1_card, span, 0x16, 0x00);
	if (debugslips) {
		/* IMR3: We care about AIS and friends */
		__rxt1_span_framer_out(rxt1_card, span, 0x17, 0xf4);
		/* IMR4: We care about slips on transmit */
		__rxt1_span_framer_out(rxt1_card, span, 0x18, 0x3f);
	} else {
		/* IMR3: We care about AIS and friends */
		__rxt1_span_framer_out(rxt1_card, span, 0x17, 0xf7);
		/* IMR4: We don't care about slips on transmit */
		__rxt1_span_framer_out(rxt1_card, span, 0x18, 0xff);
	}

	if (!polling) {
		rxt1_span_check_alarms(rxt1_card, span);
		rxt1_span_check_sigbits(rxt1_card, span);
	}

	if (debug & DEBUG_MAIN)
		printk("R%dT1: Span %d configured for %s/%s\n", rxt1_card->numspans, span + 1,
			   framing, line);
}

static void __rxt1_span_configure_e1(struct rxt1_card_t *rxt1_card, int span,
									 int lineconfig)
{
	unsigned int fmr2, fmr1, fmr0;
	unsigned int cas = 0;
	unsigned int imr3extra = 0;
	char *crc4 = "";
	char *framing, *line;
	/* FMR1: E1 mode, Automatic force resync, PCM30 mode, 8.192 Mhz backplane, no XAIS */
	fmr1 = 0x44;
	/* FMR2: Auto transmit remote alarm, auto loss of multiframe recovery, 
	 *      no payload loopback */
	fmr2 = 0x03;
	if (loopback)
		fmr2 |= 0x4;
	if (lineconfig & DAHDI_CONFIG_CRC4) {
		/* CRC4 transmit */
		fmr1 |= 0x08;
		/* CRC4 receive */
		fmr2 |= 0xc0;
		crc4 = "/CRC4";
	}
	__rxt1_span_framer_out(rxt1_card, span, 0x1d, fmr1);
	__rxt1_span_framer_out(rxt1_card, span, 0x1e, fmr2);

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
		imr3extra = 0x28;
	} else {
		framing = "CAS";
		cas = 0x40;
	}
	__rxt1_span_framer_out(rxt1_card, span, 0x1c, fmr0);

	/* LIM1: Clear data in case of LOS, Set receiver threshold (0.5V), 
	 *    No remote loop, no DRS */
	__rxt1_span_framer_out(rxt1_card, span, 0x37, 0xf8);
	/* LIM0: Enable auto long haul mode, no local loop (must be after LIM1) */
	__rxt1_span_framer_out(rxt1_card, span, 0x36, 0x08);

	/* CMDR: Reset the receiver and transmitter line interface */
	__rxt1_span_framer_out(rxt1_card, span, 0x02, 0x50);
	/* CMDR: Reset the receiver and transmitter line interface */
	__rxt1_span_framer_out(rxt1_card, span, 0x02, 0x00);

	/* Condition receive line interface for E1 after reset */
	__rxt1_span_framer_out(rxt1_card, span, 0xbb, 0x17);
	__rxt1_span_framer_out(rxt1_card, span, 0xbc, 0x55);
	__rxt1_span_framer_out(rxt1_card, span, 0xbb, 0x97);
	__rxt1_span_framer_out(rxt1_card, span, 0xbb, 0x11);
	__rxt1_span_framer_out(rxt1_card, span, 0xbc, 0xaa);
	__rxt1_span_framer_out(rxt1_card, span, 0xbb, 0x91);
	__rxt1_span_framer_out(rxt1_card, span, 0xbb, 0x12);
	__rxt1_span_framer_out(rxt1_card, span, 0xbc, 0x55);
	__rxt1_span_framer_out(rxt1_card, span, 0xbb, 0x92);
	__rxt1_span_framer_out(rxt1_card, span, 0xbb, 0x0c);
	__rxt1_span_framer_out(rxt1_card, span, 0xbb, 0x00);
	__rxt1_span_framer_out(rxt1_card, span, 0xbb, 0x8c);

	/* LIM2: 50% peak amplitude is a "1" */
	__rxt1_span_framer_out(rxt1_card, span, 0x3a, 0x20);
	/* PCD: LOS after 176 consecutive "zeros" */
	__rxt1_span_framer_out(rxt1_card, span, 0x38, 0x0a);
	/* PCR: 22 "ones" clear LOS */
	__rxt1_span_framer_out(rxt1_card, span, 0x39, 0x15);

	/* XSW: Spare bits all to 1 */
	__rxt1_span_framer_out(rxt1_card, span, 0x20, 0x9f);
	/* XSP: E-bit set when async. AXS auto, XSIF to 1 */
	__rxt1_span_framer_out(rxt1_card, span, 0x21, 0x1c | cas);


	/* Generate pulse mask for E1 */
	__rxt1_span_framer_out(rxt1_card, span, 0x26, 0x54);	/* XPM0 */
	__rxt1_span_framer_out(rxt1_card, span, 0x27, 0x02);	/* XPM1 */
	__rxt1_span_framer_out(rxt1_card, span, 0x28, 0x00);	/* XPM2 */

	/* Don't mask framer interrupts if hardware HDLC is in use */
	/* IMR0: We care about CRC errors, CAS changes, etc */
	__rxt1_span_framer_out(rxt1_card, span, FRMR_IMR0,
						   0xff & ~((rxt1_card->rxt1_spans[span]->sigchan) ?
									HDLC_IMR0_MASK : 0));
	/* IMR1: We care about loopup / loopdown */
	__rxt1_span_framer_out(rxt1_card, span, FRMR_IMR1,
						   0x3f & ~((rxt1_card->rxt1_spans[span]->sigchan) ?
									HDLC_IMR1_MASK : 0));
	/* IMR2: We care about all the alarm stuff! */
	__rxt1_span_framer_out(rxt1_card, span, 0x16, 0x00);
	if (debugslips) {
		/* IMR3: We care about AIS and friends */
		__rxt1_span_framer_out(rxt1_card, span, 0x17, 0xc4 | imr3extra);
		/* IMR4: We care about slips on transmit */
		__rxt1_span_framer_out(rxt1_card, span, 0x18, 0x3f);
	} else {
		/* IMR3: We care about AIS and friends */
		__rxt1_span_framer_out(rxt1_card, span, 0x17, 0xc7 | imr3extra);
		/* IMR4: We don't care about slips on transmit */
		__rxt1_span_framer_out(rxt1_card, span, 0x18, 0xff);
	}
	if (!polling) {
		rxt1_span_check_alarms(rxt1_card, span);
		rxt1_span_check_sigbits(rxt1_card, span);
	}

	if (debug & DEBUG_MAIN)
		printk("R%dT1: Span %d configured for %s/%s%s\n", rxt1_card->numspans, span + 1,
			   framing, line, crc4);
}

static int rxt1_span_startup(struct dahdi_span *span)
{
	int chan_num;
	int tspan;
	int alreadyrunning;

	struct rxt1_span_t *rxt1_span = container_of(span, struct rxt1_span_t, span);

	struct rxt1_card_t *rxt1_card = rxt1_span->owner;

	if (debug & DEBUG_MAIN)
		printk("R%dT1: About to enter startup!\n", rxt1_card->numspans);
	tspan = span->offset + 1;
	if (tspan < 0) {
		printk("R%dT1: Span '%d' isn't us?\n", rxt1_card->numspans, span->spanno);
		return -1;
	}

	alreadyrunning = span->flags & DAHDI_FLAG_RUNNING;

	/* initialize the start value for the entire chunk of last ec buffer */
	for (chan_num = 0; chan_num < span->channels; chan_num++) {
		memset(rxt1_span->ec_chunk1[chan_num],
			   DAHDI_LIN2X(0, span->chans[chan_num]), DAHDI_CHUNKSIZE);
		memset(rxt1_span->ec_chunk2[chan_num],
			   DAHDI_LIN2X(0, span->chans[chan_num]), DAHDI_CHUNKSIZE);
	}

	/* Force re-evaluation fo timing source */
	if (timingcable)
		rxt1_card->syncsrc = -1;

	if (rxt1_span->spantype == TYPE_E1) {	/* if this is an E1 card */
		__rxt1_span_configure_e1(rxt1_card, span->offset, span->lineconfig);
	} else {					/* is a T1 card */
		__rxt1_span_configure_t1(rxt1_card, span->offset, span->lineconfig,
								 span->txlevel);
	}

	/* Note clear channel status */
	rxt1_card->rxt1_spans[span->offset]->notclear = 0;
	__rxt1_span_set_clear(rxt1_card, span->offset);

	if (!alreadyrunning) {
		span->flags |= DAHDI_FLAG_RUNNING;
		rxt1_card->spansstarted++;

		/* enable interrupts */
		/* Start DMA, enabling DMA interrupts on read only */
		rxt1_card->nextbuf = 0;

		/* this is for test -- remove */
		rxt1_card->dmactrl |= FRMR_IMSK;
		rxt1_card->dmactrl &= ~(DMA_IMSK);
		rxt1_card->dmactrl |= (DMA_GO | FRMR_IEN);
		__rxt1_card_pci_out(rxt1_card, RXT1_DMA + TARG_REGS, rxt1_card->dmactrl,
							target_regs[RXT1_DMA].iomask);

		/* Startup HDLC controller too */
		if (rxt1_span->sigchan) {
			if (__rxt1_hdlc_start_chan
				(rxt1_card, span->offset, rxt1_span->sigchan, rxt1_span->sigmode)) {
				printk("R%dT1: Error initializing signalling controller\n",
					   rxt1_card->numspans);
				return -1;
			}
		}

		if (!polling) {
			rxt1_span_check_alarms(rxt1_card, span->offset);
			rxt1_span_check_sigbits(rxt1_card, span->offset);
		}
	}

	if (rxt1_card->rxt1_spans[0]->sync == span->spanno)
		printk("R%dT1: SPAN %d: Primary Sync Source\n", rxt1_card->numspans,
			   span->spanno);
	if (rxt1_card->rxt1_spans[1]->sync == span->spanno)
		printk("R%dT1: SPAN %d: Secondary Sync Source\n", rxt1_card->numspans,
			   span->spanno);
	if (rxt1_card->numspans == 4) {
		if (rxt1_card->rxt1_spans[2]->sync == span->spanno)
			printk("R%dT1: SPAN %d: Tertiary Sync Source\n", rxt1_card->numspans,
				   span->spanno);
		if (rxt1_card->rxt1_spans[3]->sync == span->spanno)
			printk("R%dT1: SPAN %d: Quaternary Sync Source\n", rxt1_card->numspans,
				   span->spanno);
	}

	if (debug & DEBUG_MAIN)
		printk("R%dT1: Completed Span %d startup!\n", rxt1_card->numspans, tspan);

	return 0;
}


#if (DAHDI_CHUNKSIZE != 8)
#error Sorry, rxt1 does not support chunksize != 8
#endif

static inline void __rxt1_receive_span(struct rxt1_span_t *rxt1_span)
{
	int chan_num, samp_num;
	struct rxt1_card_t *rxt1_card = rxt1_span->owner;

	if (show_pointers == 1) {

		if ((rxt1_card->intcount > 100) && (rxt1_card->intcount < 104)) {

			printk("R%dT1: Span %d writechunk %p readchunk %p int %d\n",
				   rxt1_card->numspans, rxt1_span->span.offset, rxt1_span->writechunk,
				   rxt1_span->readchunk, rxt1_card->intcount);

			for (chan_num = 0; chan_num < rxt1_span->span.channels; chan_num++) {
				struct dahdi_chan *mychans = rxt1_span->chans[chan_num];
				printk("R%dT1: Span %d Chan %d writechunk %p readchunk %p\n",
					   rxt1_card->numspans, rxt1_span->span.offset, chan_num,
					   mychans->writechunk, mychans->readchunk);
			}
		}
	}
#ifdef ENABLE_PREFETCH
	prefetch((void *) (rxt1_span->readchunk));
	prefetch((void *) (rxt1_span->writechunk));
	prefetch((void *) (rxt1_span->readchunk + 8));
	prefetch((void *) (rxt1_span->writechunk + 8));
	prefetch((void *) (rxt1_span->readchunk + 16));
	prefetch((void *) (rxt1_span->writechunk + 16));
	prefetch((void *) (rxt1_span->readchunk + 24));
	prefetch((void *) (rxt1_span->writechunk + 24));
	prefetch((void *) (rxt1_span->readchunk + 32));
	prefetch((void *) (rxt1_span->writechunk + 32));
	prefetch((void *) (rxt1_span->readchunk + 40));
	prefetch((void *) (rxt1_span->writechunk + 40));
	prefetch((void *) (rxt1_span->readchunk + 48));
	prefetch((void *) (rxt1_span->writechunk + 48));
	prefetch((void *) (rxt1_span->readchunk + 56));
	prefetch((void *) (rxt1_span->writechunk + 56));
#endif

	if ((test_pat == 1) &&
		((rxt1_card->intcount > 10000) && (rxt1_card->intcount < 10003))) {
		for (chan_num = 0; chan_num < rxt1_span->span.channels; chan_num++) {
			for (samp_num = 0; samp_num < DAHDI_CHUNKSIZE; samp_num++) {

				printk("Sp %02x Sa %02x Ch %02d = %x ",
					   rxt1_span->span.offset, samp_num,
					   chan_num, rxt1_span->span.chans[chan_num]->readchunk[samp_num]
					);
				printk("Sa %02x Ch %02d Addr %p\n",
					   rxt1_span->span.chans[chan_num]->readchunk[samp_num] & 0x7,
					   (rxt1_span->span.chans[chan_num]->readchunk[samp_num] & 0xf8) >> 3,
					   &rxt1_span->span.chans[chan_num]->readchunk[samp_num]);
			}
		}
	}

	dahdi_ec_span(&rxt1_span->span);
	dahdi_receive(&rxt1_span->span);
}

static inline void __rxt1_transmit_span(struct rxt1_span_t *rxt1_span)
{
	int chan_num, samp_num;

	dahdi_transmit(&rxt1_span->span);

	if (test_pat == 1) {
		for (chan_num = 0; chan_num < rxt1_span->span.channels; chan_num++) {
			for (samp_num = 0; samp_num < DAHDI_CHUNKSIZE; samp_num++) {
				if ((rxt1_span->span.offset == 0) && (chan_num == 1)) {
					rxt1_span->span.chans[chan_num]->writechunk[samp_num] = 0x81;
				} else {
					rxt1_span->span.chans[chan_num]->writechunk[samp_num] = 0x00;
				}
			}
		}
	}
}

#ifdef ENABLE_WORKQUEUES
static void workq_handlespan(void *data)
{
	struct rxt1_span_t *rxt1_span = data;
	struct rxt1_card_t *rxt1_card = rxt1_span->owner;

	__rxt1_receive_span(rxt1_span);
	__rxt1_transmit_span(rxt1_span);
	atomic_dec(&rxt1_card->worklist);
}
#else
static void rxt1_card_prep_gen2(struct rxt1_card_t *rxt1_card)
{
	int offset = 1;
	int span_num;
	int chan_num;

	for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
		struct rxt1_span_t *rxt1_span = rxt1_card->rxt1_spans[span_num];

		if (rxt1_span->span.flags & DAHDI_FLAG_RUNNING) {

			if (double_buffer == 1) {
				rxt1_span->writechunk =
					(void *) (rxt1_card->writechunk + span_num * 32 * 2 +
							  (rxt1_card->nextbuf * 8 * 32));
				rxt1_span->readchunk =
					(void *) (rxt1_card->readchunk + span_num * 32 * 2 +
							  (rxt1_card->nextbuf * 8 * 32));

				for (chan_num = 0; chan_num < rxt1_span->span.channels; chan_num++) {
					struct dahdi_chan *mychans = rxt1_span->chans[chan_num];

					mychans->writechunk =
						(void *) (rxt1_card->writechunk +
								  (span_num * 32 + chan_num + offset) * 2 +
								  (rxt1_card->nextbuf * 8 * 32));
					mychans->readchunk =
						(void *) (rxt1_card->readchunk +
								  (span_num * 32 + chan_num + offset) * 2 +
								  (rxt1_card->nextbuf * 8 * 32));
				}
			}

			__rxt1_receive_span(rxt1_span);
			__rxt1_transmit_span(rxt1_span);
		}
	}
}

#endif


static void rxt1_span_check_sigbits(struct rxt1_card_t *rxt1_card, int span)
{
	int a, i, rxs;
	struct rxt1_span_t *rxt1_span = rxt1_card->rxt1_spans[span];

	if (debug & DEBUG_RBS)
		printk("Checking sigbits on span %d\n", span + 1);

	if (!(rxt1_span->span.flags & DAHDI_FLAG_RUNNING))
		return;
	if (rxt1_span->spantype == TYPE_E1) {
		for (i = 0; i < 15; i++) {
			a = __rxt1_span_framer_in(rxt1_card, span, 0x71 + i);
			/* Get high channel in low bits */
			rxs = (a & 0xf);
			if (!(rxt1_span->chans[i + 16]->sig & DAHDI_SIG_CLEAR)) {
				if (rxt1_span->chans[i + 16]->rxsig != rxs)
					dahdi_rbsbits(rxt1_span->chans[i + 16], rxs);
			}
			rxs = (a >> 4) & 0xf;
			if (!(rxt1_span->chans[i]->sig & DAHDI_SIG_CLEAR)) {
				if (rxt1_span->chans[i]->rxsig != rxs)
					dahdi_rbsbits(rxt1_span->chans[i], rxs);
			}
		}
	} else if (rxt1_span->span.lineconfig & DAHDI_CONFIG_D4) {
		for (i = 0; i < 24; i += 4) {
			a = __rxt1_span_framer_in(rxt1_card, span, 0x70 + (i >> 2));
			/* Get high channel in low bits */
			rxs = (a & 0x3) << 2;
			if (!(rxt1_span->chans[i + 3]->sig & DAHDI_SIG_CLEAR)) {
				if (rxt1_span->chans[i + 3]->rxsig != rxs)
					dahdi_rbsbits(rxt1_span->chans[i + 3], rxs);
			}
			rxs = (a & 0xc);
			if (!(rxt1_span->chans[i + 2]->sig & DAHDI_SIG_CLEAR)) {
				if (rxt1_span->chans[i + 2]->rxsig != rxs)
					dahdi_rbsbits(rxt1_span->chans[i + 2], rxs);
			}
			rxs = (a >> 2) & 0xc;
			if (!(rxt1_span->chans[i + 1]->sig & DAHDI_SIG_CLEAR)) {
				if (rxt1_span->chans[i + 1]->rxsig != rxs)
					dahdi_rbsbits(rxt1_span->chans[i + 1], rxs);
			}
			rxs = (a >> 4) & 0xc;
			if (!(rxt1_span->chans[i]->sig & DAHDI_SIG_CLEAR)) {
				if (rxt1_span->chans[i]->rxsig != rxs)
					dahdi_rbsbits(rxt1_span->chans[i], rxs);
			}
		}
	} else {
		for (i = 0; i < 24; i += 2) {
			a = __rxt1_span_framer_in(rxt1_card, span, 0x70 + (i >> 1));
			/* Get high channel in low bits */
			rxs = (a & 0xf);
			if (!(rxt1_span->chans[i + 1]->sig & DAHDI_SIG_CLEAR)) {
				/* XXX Not really reset on every trans! XXX */
				if (rxt1_span->chans[i + 1]->rxsig != rxs) {
					dahdi_rbsbits(rxt1_span->chans[i + 1], rxs);
				}
			}
			rxs = (a >> 4) & 0xf;
			if (!(rxt1_span->chans[i]->sig & DAHDI_SIG_CLEAR)) {
				/* XXX Not really reset on every trans! XXX */
				if (rxt1_span->chans[i]->rxsig != rxs) {
					dahdi_rbsbits(rxt1_span->chans[i], rxs);
				}
			}
		}
	}
}

static void rxt1_span_check_alarms(struct rxt1_card_t *rxt1_card, int span)
{
	unsigned char c, d, led_state;
	int alarms;
	int x, j;
	struct rxt1_span_t *rxt1_span = rxt1_card->rxt1_spans[span];

	if (!(rxt1_span->span.flags & DAHDI_FLAG_RUNNING))
		return;

	c = __rxt1_span_framer_in(rxt1_card, span, 0x4c);
	d = __rxt1_span_framer_in(rxt1_card, span, 0x4d);

	if (debug && DEBUG_FRAMER) {
		printk("check alarms: intcount %x\n", rxt1_card->intcount);
		if (c)
			printk("SPAN %d FRMR_FRS0 = %x ", span, c);
		if (c & FRMR_FRS0_FSRF)
			printk("FRMR_FRS0_FSRF ");	/* 0 0x01 */
		if (c & FRMR_FRS0_LMFA)
			printk("FRMR_FRS0_LMFA ");	/* 1 0x02 */
		if (c & FRMR_FRS0_NMF)
			printk("FRMR_FRS0_WTFNMF ");	/* 2 0x04 */
		if (c & FRMR_FRS0_RRA)
			printk("FRMR_FRS0_RRA ");	/* 4 0x10 */
		if (c & FRMR_FRS0_LFA)
			printk("FRMR_FRS0_LFA ");	/* 5 0x20 */
		if (c & FRMR_FRS0_AIS)
			printk("FRMR_FRS0_AIS ");	/* 6 0x40 */
		if (c & FRMR_FRS0_LOS)
			printk("FRMR_FRS0_LOS");	/* 7 0x80 */
		if (c)
			printk("\n");

		if (d)
			printk("SPAN %d FRMR_FRS1 = %x ", span, d);
		if (d & FRMR_FRS1_EXZD)
			printk("FRMR_FRS1_EXZD ");	/* 7 0x80 */
		if (d & FRMR_FRS1_PDEN)
			printk("FRMR_FRS1_PDEN ");	/* 6 0x40 */
		if (d & FRMR_FRS1_LLBDD)
			printk("FRMR_FRS1_LLBDD ");	/* 4 0x10 */
		if (d & FRMR_FRS1_LLBAD)
			printk("FRMR_FRS1_LLBAD ");	/* 3 0x08 */
		if (d & FRMR_FRS1_XLS)
			printk("FRMR_FRS1_XLS ");	/* 1 0x02 */
		if (d & FRMR_FRS1_XLO)
			printk("FRMR_FRS1_XLO");	/* 0 0x01 */
		if (d)
			printk("\n");
	}

	/* Assume no alarms */
	alarms = 0;
	led_state = NORM_OP;

	/* And consider only carrier alarms */
	rxt1_span->span.alarms &= (DAHDI_ALARM_RED | DAHDI_ALARM_BLUE | DAHDI_ALARM_NOTOPEN);

	if (rxt1_span->spantype == TYPE_E1) {
		if (c & 0x04) {
			/* No multiframe found, force RAI high after 400ms only if
			   we haven't found a multiframe since last loss of frame */
			if (!(rxt1_span->spanflags & FLAG_NMF)) {
				__rxt1_span_framer_out(rxt1_card, span, 0x20, 0x9f | 0x20);	/* LIM0: Force RAI High */
				rxt1_span->spanflags |= FLAG_NMF;
				led_state = YEL_ALM;
				printk("NMF workaround on!\n");
			}
			__rxt1_span_framer_out(rxt1_card, span, 0x1e, 0xc3);	/* Reset to CRC4 mode */
			__rxt1_span_framer_out(rxt1_card, span, 0x1c, 0xf2);	/* Force Resync */
			__rxt1_span_framer_out(rxt1_card, span, 0x1c, 0xf0);	/* Force Resync */
		} else if (!(c & 0x02)) {
			if ((rxt1_span->spanflags & FLAG_NMF)) {
				/* LIM0: Clear forced RAI */
				__rxt1_span_framer_out(rxt1_card, span, 0x20, 0x9f);
				rxt1_span->spanflags &= ~FLAG_NMF;
				led_state = NORM_OP;
				printk("NMF workaround off!\n");
			}
		}
	} else {					/* T1 */
		/* Detect loopup code if we're not sending one */
		/* Line Loop Back Activate Detected */
		if ((!rxt1_span->span.mainttimer) && (d & FRMR_FRS1_LLBAD)) {
			/* Loop-up code detected */
			if ((rxt1_span->loopupcnt++ > 80) &&
				(rxt1_span->span.maintstat != DAHDI_MAINT_REMOTELOOP)) {
				/* LIM0: Disable any local loop */
				__rxt1_span_framer_out(rxt1_card, span, 0x36, 0x08);
				/* LIM1: Enable remote loop */
				__rxt1_span_framer_out(rxt1_card, span, 0x37, 0xf6);
				rxt1_span->span.maintstat = DAHDI_MAINT_REMOTELOOP;
				led_state = RLOOP;
			}
		} else
			rxt1_span->loopupcnt = 0;
		/* Same for loopdown code */
		/* Line Loop Back De-activate Detected */
		if ((!rxt1_span->span.mainttimer) && (d & FRMR_FRS1_LLBDD)) {
			/* Loop-down code detected */
			if ((rxt1_span->loopdowncnt++ > 80) &&
				(rxt1_span->span.maintstat == DAHDI_MAINT_REMOTELOOP)) {
				/* LIM0: Disable any local loop */
				__rxt1_span_framer_out(rxt1_card, span, 0x36, 0x08);
				/* LIM1: Disable remote loop */
				__rxt1_span_framer_out(rxt1_card, span, 0x37, 0xf0);
				rxt1_span->span.maintstat = DAHDI_MAINT_NONE;
				led_state = NORM_OP;
			}
		} else
			rxt1_span->loopdowncnt = 0;
	}

	if (rxt1_span->span.lineconfig & DAHDI_CONFIG_NOTOPEN) {
		for (x = 0, j = 0; x < rxt1_span->span.channels; x++)
		/*
			if ((rxt1_span->span.chans[x]->flags & DAHDI_FLAG_OPEN) ||
				(rxt1_span->span.chans[x]->flags & DAHDI_FLAG_NETDEV))
			*/
			if ((rxt1_span->span.chans[x]->flags & DAHDI_FLAG_OPEN))
				j++;
		if (!j)
			alarms |= DAHDI_ALARM_NOTOPEN;
	}

	if (c & (FRMR_FRS0_LFA | FRMR_FRS0_LOS)) {
		if (rxt1_span->alarmcount >= alarmdebounce) {
			alarms |= DAHDI_ALARM_RED;
			led_state = NO_SYNC;
		} else
			rxt1_span->alarmcount++;
	} else
		rxt1_span->alarmcount = 0;
	if (c & FRMR_FRS0_NMF) {
		alarms |= DAHDI_ALARM_BLUE;
		led_state = NO_SYNC;
	}

	if (((!rxt1_span->span.alarms) && alarms) || (rxt1_span->span.alarms && (!alarms)))
		rxt1_card->checktiming = 1;

	/* Keep track of recovering */
	if ((!alarms) && rxt1_span->span.alarms)
		rxt1_span->alarmtimer = DAHDI_ALARMSETTLE_TIME;
	if (rxt1_span->alarmtimer) {
		alarms |= DAHDI_ALARM_RECOVER;
		led_state = RECOVER;
	}
	/* If receiving alarms, go into Yellow alarm state */
	if (alarms && !(rxt1_span->spanflags & FLAG_SENDINGYELLOW)) {
		unsigned char fmr4;
		printk("R%dT1: Setting yellow alarm on span %d\n", rxt1_card->numspans, span + 1);
		/* We manually do yellow alarm to handle RECOVER and NOTOPEN, 
		 *      otherwise it's auto anyway */
		fmr4 = __rxt1_span_framer_in(rxt1_card, span, 0x20);
		__rxt1_span_framer_out(rxt1_card, span, 0x20, fmr4 | 0x20);
		rxt1_span->spanflags |= FLAG_SENDINGYELLOW;
		led_state = YEL_ALM;
	} else if ((!alarms) && (rxt1_span->spanflags & FLAG_SENDINGYELLOW)) {
		unsigned char fmr4;
		printk("R%dT1: Clearing yellow alarm on span %d\n", rxt1_card->numspans,
			   span + 1);
		/* We manually do yellow alarm to handle RECOVER  */
		fmr4 = __rxt1_span_framer_in(rxt1_card, span, 0x20);
		__rxt1_span_framer_out(rxt1_card, span, 0x20, fmr4 & ~0x20);
		rxt1_span->spanflags &= ~FLAG_SENDINGYELLOW;
	}

	/* Re-check the timing source when we enter/leave alarm, 
	 *    not withstanding yellow alarm */
	if (c & FRMR_FRS0_RRA)
		alarms |= DAHDI_ALARM_YELLOW;
	if (rxt1_span->span.mainttimer || rxt1_span->span.maintstat)
		alarms |= DAHDI_ALARM_LOOPBACK;
	rxt1_span->span.alarms = alarms;

	__rxt1_card_set_led(rxt1_card, span, led_state);

	dahdi_alarm_notify(&rxt1_span->span);
}

static void rxt1_card_do_counters(struct rxt1_card_t *rxt1_card)
{
	int span_num;
	for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
		struct rxt1_span_t *rxt1_span = rxt1_card->rxt1_spans[span_num];
		int docheck = 0;

		spin_lock(&rxt1_card->reglock);
		if (rxt1_span->loopupcnt || rxt1_span->loopdowncnt)
			docheck++;
		if (rxt1_span->alarmtimer) {
			if (!--rxt1_span->alarmtimer) {
				docheck++;
				rxt1_span->span.alarms &= ~(DAHDI_ALARM_RECOVER);

				__rxt1_card_set_led(rxt1_card, span_num, NORM_OP);
			}
		}
		spin_unlock(&rxt1_card->reglock);
		if (docheck) {
			if (!polling)
				rxt1_span_check_alarms(rxt1_card, span_num);
			dahdi_alarm_notify(&rxt1_span->span);
		}
	}
}


static inline void rxt1_span_framer_interrupt(struct rxt1_card_t *rxt1_card, int span)
{
	/* Check interrupts for a given span */
	unsigned char cis, gis, isr0, isr1, isr2, isr3, isr4, isr5, isr6, isr7;
	int readsize = -1;
	struct rxt1_span_t *rxt1_span = rxt1_card->rxt1_spans[span];
	struct dahdi_chan *sigchan;
	unsigned long flags;

	if (debug & DEBUG_FRAMER)
		printk("Framer interrupt span %d:%d!\n", rxt1_card->num, span + 1);

	/* 1st gen cards isn't used interrupts */
	cis = __rxt1_span_framer_in(rxt1_card, 0, FRMR_CIS);
	gis = __rxt1_span_framer_in(rxt1_card, span, FRMR_GIS);
	isr0 = (gis & FRMR_GIS_ISR0) ? __rxt1_span_framer_in(rxt1_card, span, FRMR_ISR0) : 0;
	isr1 = (gis & FRMR_GIS_ISR1) ? __rxt1_span_framer_in(rxt1_card, span, FRMR_ISR1) : 0;
	isr2 = (gis & FRMR_GIS_ISR2) ? __rxt1_span_framer_in(rxt1_card, span, FRMR_ISR2) : 0;
	isr3 = (gis & FRMR_GIS_ISR3) ? __rxt1_span_framer_in(rxt1_card, span, FRMR_ISR3) : 0;
	isr4 = (gis & FRMR_GIS_ISR4) ? __rxt1_span_framer_in(rxt1_card, span, FRMR_ISR4) : 0;
	isr5 = (gis & FRMR_GIS_ISR5) ? __rxt1_span_framer_in(rxt1_card, span, FRMR_ISR5) : 0;
	isr6 = (gis & FRMR_GIS_ISR6) ? __rxt1_span_framer_in(rxt1_card, span, FRMR_ISR6) : 0;
	isr7 = (gis & FRMR_GIS_ISR7) ? __rxt1_span_framer_in(rxt1_card, span, FRMR_ISR7) : 0;

	if (debug & DEBUG_FRAMER)
		printk
			("cis: %02x, gis: %02x, cnt: %d\nisr0: %02x, isr1: %02x, isr2: %02x,isr3: %02x, \nisr4: %02x, isr5: %02x, isr6: %02x, isr7: %02x\n",
			 cis, gis, rxt1_card->intcount, isr0, isr1, isr2, isr3, isr4, isr5, isr6,
			 isr7);

	if (isr0)
		rxt1_span_check_sigbits(rxt1_card, span);

	if (rxt1_span->spantype == TYPE_E1) {
		/* E1 checks */
		if ((isr3 & 0x38) || isr2 || isr1)
			rxt1_span_check_alarms(rxt1_card, span);
	} else {
		/* T1 checks */
		if (isr2 || (isr3 & 0x08))
			rxt1_span_check_alarms(rxt1_card, span);
	}

	if (!rxt1_span->span.alarms) {
		if ((isr3 & 0x3) || (isr4 & 0xc0))
			if (debug & DEBUG_MAIN) {
				if (isr3 & 0x02)
					printk("R%dT1: RECEIVE slip NEGATIVE on span %d\n",
						   rxt1_card->numspans, span + 1);
				if (isr3 & 0x01)
					printk("R%dT1: RECEIVE slip POSITIVE on span %d\n",
						   rxt1_card->numspans, span + 1);
				if (isr4 & 0x80)
					printk("R%dT1: TRANSMIT slip POSITIVE on span %d\n",
						   rxt1_card->numspans, span + 1);
				if (isr4 & 0x40)
					printk("R%dT1: TRANSMIT slip NEGATIVE on span %d\n",
						   rxt1_card->numspans, span + 1);
			}
	}

	spin_lock_irqsave(&rxt1_card->reglock, flags);

	/* HDLC controller checks - receive side */
	if (!rxt1_span->sigchan) {
		spin_unlock_irqrestore(&rxt1_card->reglock, flags);
		return;
	}

	sigchan = rxt1_span->sigchan;
	spin_unlock_irqrestore(&rxt1_card->reglock, flags);

	if (isr0 & FRMR_ISR0_RME) {
		readsize =
			(__rxt1_span_framer_in(rxt1_card, span, FRMR_RBCH) << 8) |
			__rxt1_span_framer_in(rxt1_card, span, FRMR_RBCL);
		if (debug & DEBUG_FRAMER)
			printk("Received data length is %d (%d)\n", readsize,
				   readsize & FRMR_RBCL_MAX_SIZE);
		/* RPF isn't set on last part of frame */
		if ((readsize > 0) && ((readsize &= FRMR_RBCL_MAX_SIZE) == 0))
			readsize = 32;
	} else if (isr0 & FRMR_ISR0_RPF)
		readsize = 32;

	if (readsize > 0) {
		struct dahdi_chan *sigchan = rxt1_span->sigchan;
		int i;
		unsigned char readbuf[FRMR_RBCL_MAX_SIZE];

		if (debug & DEBUG_FRAMER)
			printk("Framer %d: Got RPF/RME! readsize is %d\n", sigchan->span->offset,
				   readsize);

		for (i = 0; i < readsize; i++)
			readbuf[i] = __rxt1_span_framer_in(rxt1_card, span, FRMR_RXFIFO);

		/* Tell the framer to clear the RFIFO */
		__rxt1_span_framer_cmd_wait(rxt1_card, span, FRMR_CMDR_RMC);

		if (debug & DEBUG_FRAMER) {
			printk("RX(");
			for (i = 0; i < readsize; i++)
				printk((i ? " %02x" : "%02x"), readbuf[i]);
			printk(")\n");
		}
#ifdef DAHDI_SIG_HARDHDLC
		if (isr0 & FRMR_ISR0_RME) {
			/* Do checks for HDLC problems */
			unsigned char rsis = readbuf[readsize - 1];
			unsigned int olddebug = debug;
			unsigned char rsis_reg = __rxt1_span_framer_in(rxt1_card, span, FRMR_RSIS);

			++rxt1_span->frames_in;
			if ((debug & DEBUG_FRAMER) && !(rxt1_span->frames_in & 0x0f))
				printk("Received %d frames on span %d\n", rxt1_span->frames_in, span);
			if (debug & DEBUG_FRAMER)
				printk("Received HDLC frame %d.  RSIS = 0x%x (%x)\n",
					   rxt1_span->frames_in, rsis, rsis_reg);
			if (!(rsis & FRMR_RSIS_CRC16)) {
				if (debug & DEBUG_FRAMER)
					printk("CRC check failed %d\n", span);
				dahdi_hdlc_abort(sigchan, DAHDI_EVENT_BADFCS);
			} else if (rsis & FRMR_RSIS_RAB) {
				if (debug & DEBUG_FRAMER)
					printk("ABORT of current frame due to overflow %d\n", span);
				dahdi_hdlc_abort(sigchan, DAHDI_EVENT_ABORT);
			} else if (rsis & FRMR_RSIS_RDO) {
				if (debug & DEBUG_FRAMER)
					printk("HDLC overflow occured %d\n", span);
				dahdi_hdlc_abort(sigchan, DAHDI_EVENT_OVERRUN);
			} else if (!(rsis & FRMR_RSIS_VFR)) {
				if (debug & DEBUG_FRAMER)
					printk("Valid Frame check failed on span %d\n", span);
				dahdi_hdlc_abort(sigchan, DAHDI_EVENT_ABORT);
			} else {
				dahdi_hdlc_putbuf(sigchan, readbuf, readsize - 1);
				dahdi_hdlc_finish(sigchan);
				if (debug & DEBUG_FRAMER)
					printk("Received valid HDLC frame on span %d\n", span);
			}
			debug = olddebug;
		} else if (isr0 & FRMR_ISR0_RPF)
			dahdi_hdlc_putbuf(sigchan, readbuf, readsize);
#endif /* HARDHDLC */
	}

	/* Transmit side */
	if (isr1 & FRMR_ISR1_XDU) {
		if (debug & DEBUG_FRAMER)
			printk("XDU: Resetting signal controler!\n");
		__rxt1_span_framer_cmd_wait(rxt1_card, span, FRMR_CMDR_SRES);
	} else if (isr1 & FRMR_ISR1_XPR) {
		struct dahdi_chan *sigchan = rxt1_span->sigchan;

		if (debug & DEBUG_FRAMER)
			printk("Sigchan %d is %p\n", sigchan->chanpos, sigchan);

		if (debug & DEBUG_FRAMER)
			printk("Framer %d: Got XPR!\n", sigchan->span->offset);
		__rxt1_span_hdlc_xmit_fifo(rxt1_card, span, rxt1_span);
	}

	if (isr1 & FRMR_ISR1_ALLS) {
		if (debug & DEBUG_FRAMER)
			printk("ALLS received\n");
	}

}

DAHDI_IRQ_HANDLER(rxt1_card_interrupt_gen2)
{
	struct rxt1_card_t *rxt1_card = dev_id;
	unsigned char cis;
	int x, span_num, reg_num;

	unsigned int status;
	inirq = 1;

	/* Make sure it's really for us */
	status = rxt1_card_pci_in(rxt1_card, RXT1_DMA + TARG_REGS);

	/* Ignore if it's not for us */
	if (!(status & (FRMR_ISTAT | DMA_INT))) {
		if (unlikely(debug))
			printk("Int called with no INT status high!\n");
		return IRQ_NONE;
	}

	if (unlikely(!rxt1_card->spansstarted)) {
		if (status & DMA_INT)
			__rxt1_card_pci_out(rxt1_card, RXT1_DMA + TARG_REGS,
								rxt1_card->dmactrl | DMA_ACK,
								target_regs[RXT1_DMA].iomask);
		if (debug)
			printk("Not prepped yet!\n");
		return IRQ_NONE;
	}

	if (unlikely((rxt1_card->intcount < 20) && debug))
		printk("2G: Got interrupt, status = %08x, CIS = %04x\n", status,
			   __rxt1_span_framer_in(rxt1_card, 0, FRMR_CIS));

	if (status & DMA_INT) {
		rxt1_card->intcount++;

		__rxt1_card_pci_out(rxt1_card, RXT1_DMA + TARG_REGS, rxt1_card->dmactrl | DMA_ACK,
							target_regs[RXT1_DMA].iomask);

		if (status & BUFF_PTR) {
			if (unlikely((rxt1_card->nextbuf == 1) && debug))
				printk("Miss %d PTR was 1 twice\n", rxt1_card->intcount);
			rxt1_card->nextbuf = 1;
		} else {
			if (unlikely((rxt1_card->nextbuf == 0) && debug))
				printk("Miss %d PTR was 0 twice\n", rxt1_card->intcount);
			rxt1_card->nextbuf = 0;
		}

		if (unlikely((rxt1_card->intcount % 10000) == 0)) {
			for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
				struct rxt1_span_t *rxt1_span = rxt1_card->rxt1_spans[span_num];
				if (rxt1_span->span.flags & DAHDI_FLAG_RUNNING) {
					if (__rxt1_span_framer_in(rxt1_card, span_num, 0x4c) &
						(FRMR_FRS0_LFA | FRMR_FRS0_LOS)) {
						printk("RESYNC Card %d Span %d\n", rxt1_card->num + 1,
							   span_num + 1);
						rxt1_span_startup(&rxt1_span->span);
					}
				}
			}
		}

		if (unlikely((rxt1_card->intcount > 8500) && (regdump == 1) && (regdumped == 0))) {
			for (span_num = 0; span_num < 4; span_num++) {
				for (reg_num = 0; reg_num < 0xba; reg_num++)
					printk("Span %d Reg 0x%x: %s 0x%02x\n", span_num, reg_num,
						   framer_regs[reg_num].name,
						   __rxt1_span_framer_in(rxt1_card, span_num, reg_num));
			}
			regdumped = 1;
		}
#ifdef ENABLE_WORKQUEUES
		int cpus = num_online_cpus();
		atomic_set(&rxt1_card->worklist, rxt1_card->numspans);
		if (rxt1_card->rxt1_spans[0]->span.flags & DAHDI_FLAG_RUNNING)
			t4_queue_work(rxt1_card->workq, &rxt1_card->rxt1_spans[0]->swork, 0);
		else
			atomic_dec(&rxt1_card->worklist);
		if (rxt1_card->rxt1_spans[1]->span.flags & DAHDI_FLAG_RUNNING)
			t4_queue_work(rxt1_card->workq, &rxt1_card->rxt1_spans[1]->swork, 1 % cpus);
		else
			atomic_dec(&rxt1_card->worklist);
		if (rxt1_card->numspans == 4) {
			if (rxt1_card->rxt1_spans[2]->span.flags & DAHDI_FLAG_RUNNING)
				t4_queue_work(rxt1_card->workq, &rxt1_card->rxt1_spans[2]->swork,
							  2 % cpus);
			else
				atomic_dec(&rxt1_card->worklist);
			if (rxt1_card->rxt1_spans[3]->span.flags & DAHDI_FLAG_RUNNING)
				t4_queue_work(rxt1_card->workq, &rxt1_card->rxt1_spans[3]->swork,
							  3 % cpus);
			else
				atomic_dec(&rxt1_card->worklist);
		}
#else
		rxt1_card_prep_gen2(rxt1_card);
#endif
	}


	if (status & DMA_INT)
		rxt1_card_do_counters(rxt1_card);

	/* This should be something like :
	 * x = (intcount & (7 << shift)) >> shift
	 * not 8 polling and then 8 idle ints
	 * then case even : sigbits(x >> 1)
	 *      case odd  : alarms(x >> 1)
	 *
	 * Look up the required response time for shift
	 */
	if (polling && (status & DMA_INT)) {
		x = rxt1_card->intcount & 15 /* 63 */ ;
		switch (x) {
		case 0:
		case 1:
		case 2:
		case 3:
			rxt1_span_check_sigbits(rxt1_card, x);
			break;
		case 4:
		case 5:
		case 6:
		case 7:
			rxt1_span_check_alarms(rxt1_card, x - 4);
			break;
		}
	} else if (status & FRMR_ISTAT) {
		cis = __rxt1_span_framer_in(rxt1_card, 0, FRMR_CIS);
		/* all cards have span 0 */
		if (cis & FRMR_CIS_GIS1)
			rxt1_span_framer_interrupt(rxt1_card, 0);
		/* dual card GIS2 is in GIS3 bit position */
		if ((rxt1_card->numspans == 2) && (cis & FRMR_CIS_GIS3)) {
			rxt1_span_framer_interrupt(rxt1_card, 1);
		}
		/* other 3 on quad only */
		if (rxt1_card->numspans == 4) {
			if (cis & FRMR_CIS_GIS2)
				rxt1_span_framer_interrupt(rxt1_card, 1);
			if (cis & FRMR_CIS_GIS3)
				rxt1_span_framer_interrupt(rxt1_card, 2);
			if (cis & FRMR_CIS_GIS4)
				rxt1_span_framer_interrupt(rxt1_card, 3);
		}
	}

	if (rxt1_card->checktiming > 0)
		__rxt1_card_set_timing_source_auto(rxt1_card);
	if (rxt1_card->stopdma) {

		rxt1_card->dmactrl &= ~(DMA_GO | FRMR_IEN);
		__rxt1_card_pci_out(rxt1_card, RXT1_DMA + TARG_REGS, rxt1_card->dmactrl,
							target_regs[RXT1_DMA].iomask);
		__rxt1_card_set_timing_source(rxt1_card, 4, 0, 0);
		rxt1_card->stopdma = 0x0;

	}

	return IRQ_RETVAL(1);
}

static void rxt1_card_tsi_reset(struct rxt1_card_t *rxt1_card)
{
	int x;
	for (x = 0; x < 128; x++) {
	}
}

/* Note that channels here start from 1 */
static void rxt1_card_tsi_assign(struct rxt1_card_t *rxt1_card, int fromspan,
								 int fromchan, int tospan, int tochan)
{
	int fromts, tots;

	fromts = (fromspan << 5) | (fromchan);
	tots = (tospan << 5) | (tochan);

	if (!rxt1_card->t1e1) {
		fromts += 4;
		tots += 4;
	}
}

static void rxt1_card_tsi_unassign(struct rxt1_card_t *rxt1_card, int tospan, int tochan)
{
	int tots;

	tots = (tospan << 5) | (tochan);

	if (!rxt1_card->t1e1)
		tots += 4;
}

static int rxt1_card_hardware_init_1(struct rxt1_card_t *rxt1_card, int gen2)
{
	rxt1_card->version = (rxt1_card_pci_in(rxt1_card, RXT1_VERSION + TARG_REGS) >> 16);

	printk("R%dT1 version %08x\n", rxt1_card->numspans, rxt1_card->version);

	if (debug & DEBUG_MAIN) {
		printk("burst %s, slip debug: %s\n", noburst ? "OFF" : "ON",
			   debugslips ? "ON" : "OFF");
		printk("test pattern: %s, register dump: %s, idle codes: %s\n",
			   test_pat ? "ON" : "OFF", regdump ? "ON" : "OFF",
			   insert_idle ? "ON" : "OFF");
		printk("receive quad offset: %d, receive dual offset: %d, transmit offset: %d,\n",
			   recq_off, recd_off, xmit_off);
	}
#ifdef ENABLE_WORKQUEUES
	printk("R%dT1 running with work queues.\n", rxt1_card->numspans);
#endif
#ifdef ENABLE_PREFETCH
	printk("R%dT1 running with prefetch enabled.\n", rxt1_card->numspans);
#endif
	/* Make sure DMA engine is not running and interrupts are acknowledged */
	rxt1_card->dmactrl = 0x0;
	__rxt1_card_pci_out(rxt1_card, RXT1_DMA + TARG_REGS, rxt1_card->dmactrl,
						target_regs[RXT1_DMA].iomask);

	/* Set DMA addresses */
	__rxt1_card_pci_out(rxt1_card, RXT1_RXBUFSTART + TARG_REGS, rxt1_card->readdma,
						target_regs[RXT1_RXBUFSTART].iomask);
	__rxt1_card_pci_out(rxt1_card, RXT1_TXBUFSTART + TARG_REGS, rxt1_card->writedma,
						target_regs[RXT1_TXBUFSTART].iomask);

	rxt1_card_tsi_reset(rxt1_card);

	/* Setup counter */
	rxt1_card->dmactrl =
		((DAHDI_MAX_CHUNKSIZE * 2 * 32) << 16) | (rxt1_card->dmactrl & ~DMA_LEN);
	if (double_buffer != 1)
		rxt1_card->dmactrl |= 0x80;
	__rxt1_card_pci_out(rxt1_card, RXT1_DMA + TARG_REGS, rxt1_card->dmactrl,
						target_regs[RXT1_DMA].iomask);

	rxt1_card->order = 0x0;

	return 0;
}

static int rxt1_card_hardware_init_2(struct rxt1_card_t *rxt1_card)
{
	int x;
	unsigned int falcver;

	rxt1_span_framer_out(rxt1_card, 0, 0x4a, 0xaa);
	falcver = rxt1_span_framer_in(rxt1_card, 0, 0x4a);
	printk("R%dT1: FALC version: %08x, Board ID: %02x\n", rxt1_card->numspans, falcver,
		   rxt1_card->order);

	if (debug & DEBUG_MAIN) {
		for (x = 0; x < 5; x++)
			printk("R%dT1: Reg %d: %s 0x%08x\n", rxt1_card->numspans, x,
				   target_regs[x].name, rxt1_card_pci_in(rxt1_card, x + TARG_REGS));
	}

	return 0;
}

static inline void rxt1_card_reset_dsp(struct rxt1_card_t *rxt1_card)
{
	unsigned long flags;
	unsigned int hpi_c;

	spin_lock_irqsave(&rxt1_card->reglock, flags);
	hpi_c = __rxt1_card_pci_in(rxt1_card, TARG_REGS + RXT1_HPIC);

	__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_HPIC, (hpi_c & ~DSP_RST),
						target_regs[RXT1_HPIC].iomask);
	__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_HPIC, (hpi_c & ~DSP_RST),
						target_regs[RXT1_HPIC].iomask);

	rxt1_card->hpi_fast = 0;
	rxt1_card->hpi_xadd[0] = 0;
	rxt1_card->hpi_xadd[1] = 0;
	rxt1_card->hpi_xadd[2] = 0;
	rxt1_card->hpi_xadd[3] = 0;
	rxt1_card->dsp_sel = 0;

	__rxt1_card_pci_out(rxt1_card, RXT1_HCS_REG + TARG_REGS, 0, 0);
	__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_HPIC, (hpi_c | DSP_RST),
						target_regs[RXT1_HPIC].iomask);

	spin_unlock_irqrestore(&rxt1_card->reglock, flags);
}


int try_select_dsp(struct rxt1_card_t *rxt1_card, int span_num, int bc)
{
	int hcs;
	int sel;

	sel = __rxt1_card_pci_in(rxt1_card, RXT1_HCS_REG + TARG_REGS);

	if (sel == 0) {
		if (bc == 0) {
			hcs = (1 << span_num);
			__rxt1_card_pci_out(rxt1_card, RXT1_HCS_REG + TARG_REGS, (__u32) (hcs), 0);
		} else
			__rxt1_card_pci_out(rxt1_card, RXT1_HCS_REG + TARG_REGS, (__u32) 0xf, 0);

		return 0;
	}
	return 1;
}

void rxt1_card_select_dsp(struct rxt1_card_t *rxt1_card, int span_num, int bc)
{
	rxt1_card->dsp_sel = span_num;
}

void rxt1_card_unselect_dsp(struct rxt1_card_t *rxt1_card, int span_num)
{
	rxt1_card->dsp_sel = 0;
}

static unsigned short int rxt1_card_dsp_ping(struct rxt1_card_t *rxt1_card, int span_num)
{
	gpakPingDspStat_t ping_stat;
	unsigned short int dsp_ver;
	unsigned short int DspId;

	DspId = (rxt1_card->num * 4) + span_num;

	rxt1_card_select_dsp(rxt1_card, span_num, 0);

	ping_stat = gpakPingDsp(rxt1_card, DspId, &dsp_ver);

	if (debug & DEBUG_DSP) {
		if (ping_stat == PngSuccess)
			printk("R%dT1 %d %d: G168 DSP Ping DSP Version %x\n", rxt1_card->numspans,
				   rxt1_card->num + 1, DspId + 1, dsp_ver);
		else
			printk("R%dT1 %d %d: G168 DSP Ping Error %d\n", rxt1_card->numspans,
				   rxt1_card->num + 1, DspId + 1, ping_stat);
	}

	rxt1_card_unselect_dsp(rxt1_card, span_num);

	if (ping_stat == PngSuccess)
		return dsp_ver;
	else
		return 0;
}

static int __devinit rxt1_span_download_dsp(struct rxt1_card_t *rxt1_card, int span_num)
{
	unsigned short int DspId;
	gpakDownloadStatus_t dl_res = 0;

	rxt1_card_select_dsp(rxt1_card, span_num, 0);
	DspId = (rxt1_card->num * 4) + span_num;
	if ((dl_res = gpakDownloadDsp_5510(rxt1_card, DspId, app_file)))
		printk("R%dT1 %d DSP %d: G168 DSP App Loader Failed %d\n", rxt1_card->numspans,
			   rxt1_card->num + 1, DspId + 1, dl_res);
	else
		printk("R%dT1 %d DSP %d: G168 DSP App Loader Success %d\n", rxt1_card->numspans,
			   rxt1_card->num + 1, DspId + 1, dl_res);

	rxt1_card_dsp_set(rxt1_card, DSP_IFBLK_ADDRESS, 0);
	rxt1_card_dsp_set(rxt1_card, DSP_IFBLK_ADDRESS + 1, 0);

	rxt1_card_unselect_dsp(rxt1_card, span_num);

	if (dl_res)
		return -1;
	else
		return 0;
}

static void __devinit rxt1_span_run_dsp(struct rxt1_card_t *rxt1_card, int span_num)
{
	unsigned long flags;
	unsigned long hcs;

	spin_lock_irqsave(&rxt1_card->reglock, flags);

	rxt1_card_select_dsp(rxt1_card, span_num, 0);

	hcs = 1 << rxt1_card->dsp_sel;

	__rxt1_card_pci_out(rxt1_card, RXT1_HCS_REG + TARG_REGS, hcs, 0);

	rxt1_card_hpic_set(rxt1_card, RXT1_BL_GO);

	__rxt1_card_pci_out(rxt1_card, RXT1_HCS_REG + TARG_REGS, 0, 0);

	rxt1_card_unselect_dsp(rxt1_card, span_num);

	spin_unlock_irqrestore(&rxt1_card->reglock, flags);

	return;
}

static GpakPortConfig_t Gpak_32_chan_port_config = {

	/* GpakSlotCfg_t         SlotsSelect1          port 1 Slot selection */
	SlotCfgNone,
	/* unsigned short int    FirstBlockNum1        port 1 first group Block Number */
	0x0000,
	/* unsigned short int    FirstSlotMask1        port 1 first group Slot Mask */
	0x0000,
	/* unsigned short int    SecBlockNum1          port 1 second group Block Number */
	0x0000,
	/* unsigned short int    SecSlotMask1          port 1 second group Slot Mask */
	0x0000,
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
	/* GpakSerClockPol_t     TxClockPolarity1      port 1 Rx Clock Polarity */
	SerClockActHigh,
	/* GpakSerDataDelay_t    TxDataDelay1          port 1 Tx data delay */
	DataDelay1,
	/* GpakSerDataDelay_t    RxDataDelay1          port 1 Rx data delay */
	DataDelay1,
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
	SlotCfg8Groups,
	/* unsigned short int    FirstBlockNum2        port 2 first group Block Number */
	0,
	/* unsigned short int    FirstSlotMask2        port 2 first group Slot Mask */
	0x1110,
	/* unsigned short int    SecBlockNum2          port 2 second group Block Number */
	1,
	/* unsigned short int    SecSlotMask2          port 2 second group Slot Mask */
	0x1111,
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
	SerClockActHigh,
	/* GpakSerDataDelay_t    TxDataDelay2          port 2 Tx data delay */
	DataDelay1,
	/* GpakSerDataDelay_t    RxDataDelay2          port 2 Rx data delay */
	DataDelay1,
	/* GpakActivation        DxDelay2              port 2 DX Delay */
	Disabled,
	/* unsigned short int    ThirdSlotMask2        port 2 3rd group Slot Mask */
	0x1111,
	/* unsigned short int    FouthSlotMask2        port 2 4th group Slot Mask */
	0x1111,
	/* unsigned short int    FifthSlotMask2        port 2 5th group Slot Mask */
	0x1111,
	/* unsigned short int    SixthSlotMask2        port 2 6th group Slot Mask */
	0x1111,
	/* unsigned short int    SevenSlotMask2        port 2 7th group Slot Mask */
	0x1111,
	/* unsigned short int    EightSlotMask2        port 2 8th group Slot Mask */
	0x1111,

	/* GpakSlotCfg_t         SlotsSelect3          port 3 Slot selection */
	SlotCfg8Groups,
	/* unsigned short int    FirstBlockNum3        port 3 first group Block Number */
	0,
	/* unsigned short int    FirstSlotMask3        port 3 first group Slot Mask */
	0x1110,
	/* unsigned short int    SecBlockNum3          port 3 second group Block Number */
	1,
	/* unsigned short int    SecSlotMask3          port 3 second group Slot Mask */
	0x1111,
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
	DataDelay1,
	/* GpakSerDataDelay_t    RxDataDelay3          port 3 Rx data delay */
	DataDelay1,
	/* GpakActivation        DxDelay3              port 3 DX Delay */
	Disabled,
	/* unsigned short int    ThirdSlotMask3        port 3 3rd group Slot Mask */
	0x1111,
	/* unsigned short int    FouthSlotMask3        port 3 4th group Slot Mask */
	0x1111,
	/* unsigned short int    FifthSlotMask3        port 3 5th group Slot Mask */
	0x1111,
	/* unsigned short int    SixthSlotMask3        port 3 6th group Slot Mask */
	0x1111,
	/* unsigned short int    SevenSlotMask3        port 3 7th group Slot Mask */
	0x1111,
	/* unsigned short int    EightSlotMask3        port 3 8th group Slot Mask */
	0x1111,
};

static void rxt1_card_dsp_show_portconfig(GpakPortConfig_t PortConfig)
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

static int __devinit rxt1_span_dsp_configureports(struct rxt1_card_t *rxt1_card,
												  GpakPortConfig_t PortConfig,
												  int span_num)
{
	gpakConfigPortStatus_t cp_res;
	GPAK_PortConfigStat_t cp_error;
	unsigned short int DspId;

	rxt1_card_dsp_show_portconfig(PortConfig);

	rxt1_card_select_dsp(rxt1_card, span_num, 0);
	DspId = (rxt1_card->num * 4) + span_num;

	if ((cp_res = gpakConfigurePorts(rxt1_card, DspId, &PortConfig, &cp_error)))
		printk("R%dT1 %d DSP %d: G168 DSP Port Config failed res = %d error = %d\n",
			   rxt1_card->numspans, rxt1_card->num + 1, DspId + 1, cp_res, cp_error);
	else if (debug & DEBUG_DSP) {
		printk("R%dT1 %d DSP %d: G168 DSP Port Config success %d\n", rxt1_card->numspans,
			   rxt1_card->num + 1, DspId + 1, cp_res);
	}

	rxt1_card_unselect_dsp(rxt1_card, span_num);

	if (cp_res)
		return -1;
	else
		return 0;
}

static GpakChannelConfig_t Gpak_chan_config = {

	/* GpakSerialPort_t    PCM Input Serial Port A Id */
	SerialPort2,
	/* unsigned short int  PCM Input Time Slot */
	0,
	/* GpakSerialPort_t    PCM Output Serial Port A Id */
	SerialPort3,
	/* unsigned short int  PCM Output Time Slot */
	0,
	/* GpakSerialPort_t    PCM Input Serial Port B Id */
	SerialPort3,
	/* unsigned short int  PCM Input Time Slot */
	0,
	/* GpakSerialPort_t    PCM Output Serial Port B Id */
	SerialPortNull,
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
	 17,
	 /* short int  Dynamic NLP control, NLP limit when EC not converged yet */
	 12,
	 /* short int  suppression level for NLP_SUPP mode */
	 0,
	 /* short int  Echo Can CNG Noise threshold */
	 50,
	 /* short int  Echo Can Max Adapts per frame */
	 40,
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
	 17,
	 /* short int  Dynamic NLP control, NLP limit when EC not converged yet */
	 12,
	 /* short int  suppression level for NLP_SUPP mode */
	 0,
	 /* short int  Echo Can CNG Noise threshold */
	 50,
	 /* short int  Echo Can Max Adapts per frame */
	 40,
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
	Disabled,
	Disabled,
	Disabled,
	Disabled
};

static void rxt1_card_dsp_show_chanconfig(GpakChannelConfig_t ChanConfig)
{
	if (debug & DEBUG_DSP) {
		printk("%d = %s\n", ChanConfig.PcmInPortA, "PcmInPortA");
		printk("%d = %s\n", ChanConfig.PcmInSlotA, "PcmInSlotA");
		printk("%d = %s\n", ChanConfig.PcmOutPortA, "PcmOutPortA");
		printk("%d = %s\n", ChanConfig.PcmOutSlotA, "PcmOutSlotA");
		printk("%d = %s\n", ChanConfig.PcmInPortB, "PcmInPortB");
		printk("%d = %s\n", ChanConfig.PcmInSlotB, "PcmInSlotB");
		printk("%d = %s\n", ChanConfig.PcmOutPortB, "PcmOutPortB");
		printk("%d = %s\n", ChanConfig.PcmOutSlotB, "PcmOutSlotB");

		printk("%d = %s\n", ChanConfig.ToneTypesA, "ToneTypesA");
		printk("%d = %s\n", ChanConfig.ToneTypesB, "ToneTypesB");

		printk("%d = %s\n", ChanConfig.EcanEnableA, "EcanEnableA");
		printk("%d = %s\n", ChanConfig.EcanEnableB, "EcanEnableB");

		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanTapLength,
			   "EcanParametersA.EcanTapLength");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanNlpType,
			   "EcanParametersA.EcanNlpType");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanAdaptEnable,
			   "EcanParametersA.EcanAdaptEnable");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanG165DetEnable,
			   "EcanParametersA.EcanG165DetEnable");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanDblTalkThresh,
			   "EcanParametersA.EcanDblTalkThresh");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanNlpThreshold,
			   "EcanParametersA.EcanNlpThreshold");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanNlpConv,
			   "EcanParametersA.EcanNlpConv");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanNlpUnConv,
			   "EcanParametersA.EcanNlpUnConv");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanNlpMaxSuppress,
			   "EcanParametersA.EcanNlpMaxSuppress");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanCngThreshold,
			   "EcanParametersA.EcanCngThreshold");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanAdaptLimit,
			   "EcanParametersA.EcanAdaptLimit");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanCrossCorrLimit,
			   "EcanParametersA.EcanCrossCorrLimit");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanNumFirSegments,
			   "EcanParametersA.EcanNumFirSegments");
		printk("%d = %s\n", ChanConfig.EcanParametersA.EcanFirSegmentLen,
			   "EcanParametersA.EcanFirSegmentLen");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanTapLength,
			   "EcanParametersB.EcanTapLength");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanNlpType,
			   "EcanParametersB.EcanNlpType");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanAdaptEnable,
			   "EcanParametersB.EcanAdaptEnable");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanG165DetEnable,
			   "EcanParametersB.EcanG165DetEnable");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanDblTalkThresh,
			   "EcanParametersB.EcanDblTalkThresh");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanNlpThreshold,
			   "EcanParametersB.EcanNlpThreshold");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanNlpConv,
			   "EcanParametersB.EcanNlpConv");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanNlpUnConv,
			   "EcanParametersB.EcanNlpUnConv");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanNlpMaxSuppress,
			   "EcanParametersB.EcanNlpMaxSuppress");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanCngThreshold,
			   "EcanParametersB.EcanCngThreshold");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanAdaptLimit,
			   "EcanParametersB.EcanAdaptLimit");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanCrossCorrLimit,
			   "EcanParametersB.EcanCrossCorrLimit");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanNumFirSegments,
			   "EcanParametersB.EcanNumFirSegments");
		printk("%d = %s\n", ChanConfig.EcanParametersB.EcanFirSegmentLen,
			   "EcanParametersB.EcanFirSegmentLen");

		printk("%d = %s\n", ChanConfig.SoftwareCompand, "SoftwareCompand");

		printk("%d = %s\n", ChanConfig.FrameRate, "FrameRate");

		printk("%d = %s\n", ChanConfig.MuteToneA, "MuteToneA");
		printk("%d = %s\n", ChanConfig.MuteToneB, "MuteToneB");
		printk("%d = %s\n", ChanConfig.FaxCngDetA, "FaxCngDetA");
		printk("%d = %s\n", ChanConfig.FaxCngDetB, "FaxCngDetB");

	}
	return;
}

static int __devinit rxt1_span_dsp_configurechannel(struct rxt1_card_t *rxt1_card,
													GpakChannelConfig_t ChanConfig,
													int chan_num, int span_num)
{
	GPAK_ChannelConfigStat_t chan_config_err;
	gpakConfigChanStatus_t chan_conf_stat;
	unsigned short int DspId;

	rxt1_card_dsp_show_chanconfig(ChanConfig);

	rxt1_card_select_dsp(rxt1_card, span_num, 0);
	DspId = (rxt1_card->num * 4) + span_num;

	if ((chan_conf_stat =
		 gpakConfigureChannel(rxt1_card, DspId, chan_num, tdmToTdm, &Gpak_chan_config,
							  &chan_config_err)))
		printk("R%dT1 %d DSP %d: Chan %d G168 DSP Chan Config failed error = %d  %d\n",
			   rxt1_card->numspans, rxt1_card->num + 1, DspId + 1, chan_num,
			   chan_config_err, chan_conf_stat);
	else if (debug & DEBUG_DSP) {
		printk("R%dT1 %d DSP %d: G168 DSP Chan %d Config success %d\n",
			   rxt1_card->numspans, rxt1_card->num + 1, DspId + 1, chan_num,
			   chan_conf_stat);
	}

	rxt1_card_unselect_dsp(rxt1_card, span_num);

	if (chan_conf_stat)
		return -1;
	else
		return 0;
}

static void rxt1_card_dsp_framestats(struct rxt1_card_t *rxt1_card, int span_num)
{
	gpakReadFramingStatsStatus_t framing_status_status;
	unsigned short int ec1, ec2, ec3, dmaec, slips[6];
	unsigned short int DspId;

	rxt1_card_select_dsp(rxt1_card, span_num, 0);
	DspId = (rxt1_card->num * 4) + span_num;

	if (debug & DEBUG_DSP) {
		framing_status_status =
			gpakReadFramingStats(rxt1_card, DspId, &ec1, &ec2, &ec3, &dmaec, &slips[0]);
		if (framing_status_status == RfsSuccess) {
			printk("R%dT1 %d DSP %d: G168 DSP Framing Status Success %d\n",
				   rxt1_card->numspans, rxt1_card->num + 1, DspId + 1,
				   framing_status_status);
			if (ec1 + ec2 + ec3 + dmaec + slips[0] + slips[1] + slips[2] + slips[3] +
				slips[4] + slips[5]) {
				printk
					("R%dT1 %d DSP %d: G168 DSP Framing Status p1 %2d p2 %2d p3 %2d stop %2d\n",
					 rxt1_card->numspans, rxt1_card->num + 1, DspId + 1, ec1, ec2, ec3,
					 dmaec);
				printk
					("R%dT1 %d DSP %d: G168 DSP Framing Status slip0 %2d slip1 %2d slip2 %2d "
					 "slip3 %2d slip4 %2d slip5 %2d\n",
					 rxt1_card->numspans, rxt1_card->num + 1, DspId + 1, slips[0],
					 slips[1], slips[2], slips[3], slips[4], slips[5]);
			} else
				printk("R%dT1 %d DSP %d: G168 DSP Framing Status GOOD!!\n",
					   rxt1_card->numspans, rxt1_card->num + 1, DspId + 1);
		} else {
			printk("R%dT1 %d DSP %d: G168 DSP Framing Status Failed %d\n",
				   rxt1_card->numspans, rxt1_card->num + 1, DspId + 1,
				   framing_status_status);
			printk("R%dT1 %d DSP %d: G168 DSP Framing Status %d %d %d %d %d\n",
				   rxt1_card->numspans, rxt1_card->num + 1, DspId + 1, ec1, ec2, ec3,
				   dmaec, slips[0]);
		}
	}

	rxt1_card_unselect_dsp(rxt1_card, span_num);
	return;
}

static void rxt1_card_dsp_reetframestats(struct rxt1_card_t *rxt1_card, int span_num)
{
	gpakResetFramingStatsStatus_t framing_reset_status;
	unsigned short int DspId;

	rxt1_card_select_dsp(rxt1_card, span_num, 0);
	DspId = (rxt1_card->num * 4) + span_num;

	if ((framing_reset_status = gpakResetFramingStats(rxt1_card, DspId)))
		printk("R%dT1 %d DSP %d: G168 DSP Reset Framing Stats Failed %d\n",
			   rxt1_card->numspans, rxt1_card->num + 1, DspId + 1, framing_reset_status);
	else
		printk("R%dT1 %d DSP %d: G168 DSP Reset Framing Stats Success %d\n",
			   rxt1_card->numspans, rxt1_card->num + 1, DspId + 1, framing_reset_status);

	rxt1_card_unselect_dsp(rxt1_card, span_num);
	return;
}

static void rxt1_card_dsp_cpustats(struct rxt1_card_t *rxt1_card, int span_num)
{
	gpakReadCpuUsageStat_t cpu_status_status;
	unsigned short int pPeakUsage, pPrev1SecPeakUsage;
	unsigned short int DspId;

	rxt1_card_select_dsp(rxt1_card, span_num, 0);
	DspId = (rxt1_card->num * 4) + span_num;

	cpu_status_status =
		gpakReadCpuUsage(rxt1_card, DspId, &pPeakUsage, &pPrev1SecPeakUsage);
	if (cpu_status_status)
		printk("R%dT1 %d DSP %d: G168 DSP CPU Status Failed %d\n", rxt1_card->numspans,
			   rxt1_card->num + 1, DspId + 1, cpu_status_status);
	else
		printk("R%dT1 %d DSP %d: G168 DSP CPU Status peek %2d  1 S %2d\n",
			   rxt1_card->numspans, rxt1_card->num + 1, DspId + 1, pPeakUsage,
			   pPrev1SecPeakUsage);

	rxt1_card_unselect_dsp(rxt1_card, span_num);
	return;
}

static void rxt1_chan_ec_enable(struct rxt1_card_t *rxt1_card, int span_num, int chan_num)
{
	gpakAlgControlStat_t a_c_stat;
	GPAK_AlgControlStat_t a_c_err;
	unsigned short int DspId;
	unsigned int en_mask = ec_disable_1;
	unsigned int mask;

	if (span_num == 1)
		en_mask = ec_disable_2;
	if (span_num == 2)
		en_mask = ec_disable_3;
	if (span_num == 3)
		en_mask = ec_disable_4;

	if (en_mask & (1 << chan_num)) {
		if (debug & DEBUG_DSP)
			printk("rxt1 %d: Echo Can NOT enable DSP %d EC Chan %d\n", rxt1_card->num + 1,
				   span_num, chan_num);
		return;
	}

	DspId = (rxt1_card->num * 4) + span_num;

	if (debug & DEBUG_DSP)
		printk("rxt1 %d: Echo Can enable DSP %d EC Chan %d\n", rxt1_card->num + 1,
			   span_num, chan_num);

	rxt1_card_select_dsp(rxt1_card, span_num, 0);

	if ((a_c_stat = gpakAlgControl(rxt1_card, DspId, chan_num, EnableEcanB, &a_c_err))) {
		if ((a_c_stat =
			 gpakAlgControl(rxt1_card, DspId, chan_num, EnableEcanB, &a_c_err))) {
			if ((a_c_stat =
				 gpakAlgControl(rxt1_card, DspId, chan_num, EnableEcanB, &a_c_err))) {
				printk
					("R%dT1 %d: G168 DSP Enable Alg Control failed res = %d error = %d\n",
					 rxt1_card->numspans, rxt1_card->num + 1, a_c_stat, a_c_err);
			}
		}
	}

	msleep(1);

	mask = __rxt1_card_pci_in(rxt1_card, TARG_REGS + RXT1_ECA1 + (span_num * 2));
	mask |= (1 << chan_num);

	if (debug & DEBUG_DSP)
		printk("rxt1 %d: Echo Mask %x\n", rxt1_card->num + 1, mask);

	rxt1_card_unselect_dsp(rxt1_card, span_num);

	return;
}

static void rxt1_chan_ec_disable(struct rxt1_card_t *rxt1_card, int span_num,
								 int chan_num)
{
	gpakAlgControlStat_t a_c_stat;
	GPAK_AlgControlStat_t a_c_err;
	unsigned short int DspId;
	unsigned int mask;

	DspId = (rxt1_card->num * 4) + span_num;

	if (debug & DEBUG_DSP)
		printk("rxt1 %d: Echo Can disable DSP %d EC Chan %d\n", rxt1_card->num + 1,
			   span_num, chan_num);

	rxt1_card_select_dsp(rxt1_card, span_num, 0);

	if ((a_c_stat = gpakAlgControl(rxt1_card, DspId, chan_num, BypassEcanB, &a_c_err))) {
		if ((a_c_stat =
			 gpakAlgControl(rxt1_card, DspId, chan_num, BypassEcanB, &a_c_err))) {
			if ((a_c_stat =
				 gpakAlgControl(rxt1_card, DspId, chan_num, BypassEcanB, &a_c_err))) {
				printk
					("R%dT1 %d: G168 DSP Disable Alg Control failed res = %d error = %d\n",
					 rxt1_card->numspans, rxt1_card->num + 1, a_c_stat, a_c_err);
			}
		}
	}

	msleep(1);

	mask = __rxt1_card_pci_in(rxt1_card, TARG_REGS + RXT1_ECA1 + (span_num * 2));
	mask &= ~(1 << chan_num);

	if (debug & DEBUG_DSP)
		printk("rxt1 %d: Echo Mask %x\n", rxt1_card->num + 1, mask);

	rxt1_card_unselect_dsp(rxt1_card, span_num);

	return;
}

static int rxt1_echocan_create(struct dahdi_chan *chan, struct dahdi_echocanparams *ecp,
							   struct dahdi_echocanparam *p,
							   struct dahdi_echocan_state **ec)
{
	struct dahdi_span *span = chan->span;
	struct rxt1_span_t *rxt1_span = container_of(span, struct rxt1_span_t, span);
	struct rxt1_card_t *rxt1_card = rxt1_span->owner;

	int span_num, chan_num;
	const struct dahdi_echocan_ops *ops;
	const struct dahdi_echocan_features *features;
	ops = &my_ec_ops;
	features = &my_ec_features;

	if (ecp->param_count > 0) {
		printk(KERN_WARNING
			   "RXT1 echo canceller does not support parameters; failing request\n");
		return -EINVAL;
	}

	span_num = chan->span->offset;
	chan_num = chan->chanpos - 1;
	if (debug & DEBUG_DSP)
		printk("rxt1 %d: Echo Can control Span %d Chan %d dahdi_chan %d\n",
			   rxt1_card->num + 1, span_num + 1, chan_num, chan->channo);

	if (rxt1_span->dsp_up == 1) {
		*ec = rxt1_span->ec[chan_num];
		printk("ec %x\n", (__u32) (ec));

		(*ec)->ops = ops;
		(*ec)->features = *features;
		rxt1_card->nextec[span_num] |= (1 << chan_num);
		printk("rxt1 echo can create nextec %x\n", rxt1_card->nextec[span_num]);
		queue_work(rxt1_card->dspwq, &rxt1_card->dspwork);
	}

	return 0;
}

static void rxt1_echocan_free(struct dahdi_chan *chan, struct dahdi_echocan_state *ec)
{
	struct dahdi_span *span = chan->span;
	struct rxt1_span_t *rxt1_span = container_of(span, struct rxt1_span_t, span);
	struct rxt1_card_t *rxt1_card = rxt1_span->owner;
	int span_num, chan_num;

	memset(ec, 0, sizeof(*ec));
	chan_num = chan->chanpos - 1;

	span_num = chan->span->offset;
	chan_num = chan->chanpos - 1;
	if (debug & DEBUG_DSP)
		printk("rxt1 %d: Echo Can control Span %d Chan %d dahdi_chan %d\n",
			   rxt1_card->num + 1, span_num + 1, chan_num, chan->channo);

	if (rxt1_span->dsp_up == 1) {
		printk("rxt1 echo can free nextec %x\n", rxt1_card->nextec[span_num]);
		rxt1_card->nextec[span_num] &= ~(1 << chan_num);
		queue_work(rxt1_card->dspwq, &rxt1_card->dspwork);
	}
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
static void echocan_bh(void *data)
{
	struct rxt1_card_t *rxt1_card = data;
#else
static void echocan_bh(struct work_struct *data)
{
	struct rxt1_card_t *rxt1_card = container_of(data, struct rxt1_card_t, dspwork);
#endif
	unsigned int todo[4], chan_num, span_num;

	for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
		todo[span_num] = rxt1_card->nextec[span_num] ^ rxt1_card->currec[span_num];
		if (debug & DEBUG_DSP) {
			printk("rxt1: %d Span %d Echo Can control bh change %x to %x\n",
				   rxt1_card->num + 1, span_num, todo[span_num],
				   (rxt1_card->nextec[span_num] & todo[span_num]));
			printk("nextec %x currec %x\n", rxt1_card->nextec[span_num],
				   rxt1_card->currec[span_num]);
		}

		for (chan_num = 0; chan_num < rxt1_card->rxt1_spans[span_num]->span.channels;
			 chan_num++) {
			if (todo[span_num] & (1 << chan_num)) {
				if (rxt1_card->nextec[span_num] & (1 << chan_num)) {
					rxt1_chan_ec_enable(rxt1_card, span_num, chan_num);
					rxt1_card->currec[span_num] |= (1 << chan_num);
				} else {
					rxt1_chan_ec_disable(rxt1_card, span_num, chan_num);
					rxt1_card->currec[span_num] &= ~(1 << chan_num);
				}
			}
		}
	}
}

static int __devinit rxt1_card_init_dsp(struct rxt1_card_t *rxt1_card)
{
	int loops = 0;
	__u16 high, low;
	int span_num, chan_num, chan_count;
	int ifb_z = 4;
	int strt = 4;
	int skip = 4;

	if (debug & DEBUG_DSP)
		printk("Reset DSP\n");
	rxt1_card_reset_dsp(rxt1_card);
	if (debug & DEBUG_DSP)
		printk("Un-Reset DSP\n");

	__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_ECB1, 0, 0);
	__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_ECB2, 0, 0);
	__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_ECB3, 0, 0);
	__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_ECB4, 0, 0);

	__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_ECA1, 0, 0);
	__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_ECA2, 0, 0);
	__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_ECA3, 0, 0);
	__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_ECA4, 0, 0);

	if (no_ec)
		return 0;

	__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_HPIC,
						(~EC_ON & __rxt1_card_pci_in(rxt1_card, TARG_REGS + RXT1_HPIC)),
						target_regs[RXT1_HPIC].iomask);

	for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
		if (rxt1_span_download_dsp(rxt1_card, span_num)) {
			return -1;
		}
	}

	for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
		rxt1_span_run_dsp(rxt1_card, span_num);
		if (debug & DEBUG_DSP)
			printk("R%dT1 %d DSP %d: GO!!\n", rxt1_card->numspans, rxt1_card->num + 1,
				   (rxt1_card->num * 4) + span_num + 1);
	}

	while (ifb_z != 0) {
		ifb_z = 0;
		msleep(100);
		for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
			rxt1_card_select_dsp(rxt1_card, span_num, 0);
			high = rxt1_card_dsp_get(rxt1_card, DSP_IFBLK_ADDRESS);
			low = rxt1_card_dsp_get(rxt1_card, DSP_IFBLK_ADDRESS + 1);
			if (debug & DEBUG_DSP)
				printk("R%dT1 %d DSP %d: IfBlockPntr %x\n", rxt1_card->numspans,
					   rxt1_card->num + 1, (rxt1_card->num * 4) + span_num + 1,
					   ((high << 16) + low));
			if ((high == 0) && (low == 0))
				ifb_z++;
			rxt1_card_unselect_dsp(rxt1_card, span_num);
		}
		schedule();
		if ((loops++) > 2) {
			printk("R%dT1 %d at least one DSP did not respond No EC configured\n",
				   rxt1_card->numspans, rxt1_card->num + 1);
			return -1;
		}
	}

	rxt1_card->hpi_fast = 0;

	for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
		rxt1_card_dsp_ping(rxt1_card, span_num);
	}

	Gpak_32_chan_port_config.FirstSlotMask2 = 0x1111;
	Gpak_32_chan_port_config.SecSlotMask2 = 0x1111;
	Gpak_32_chan_port_config.ThirdSlotMask2 = 0x1111;
	Gpak_32_chan_port_config.FouthSlotMask2 = 0x1111;
	Gpak_32_chan_port_config.FifthSlotMask2 = 0x1111;
	Gpak_32_chan_port_config.SixthSlotMask2 = 0x1111;
	Gpak_32_chan_port_config.SevenSlotMask2 = 0x1111;
	Gpak_32_chan_port_config.EightSlotMask2 = 0x1111;
	Gpak_32_chan_port_config.FirstSlotMask3 = 0x1111;
	Gpak_32_chan_port_config.SecSlotMask3 = 0x1111;
	Gpak_32_chan_port_config.ThirdSlotMask3 = 0x1111;
	Gpak_32_chan_port_config.FouthSlotMask3 = 0x1111;
	Gpak_32_chan_port_config.FifthSlotMask3 = 0x1111;
	Gpak_32_chan_port_config.SixthSlotMask3 = 0x1111;
	Gpak_32_chan_port_config.SevenSlotMask3 = 0x1111;
	Gpak_32_chan_port_config.EightSlotMask3 = 0x1111;

	for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {

		if (rxt1_span_dsp_configureports(rxt1_card, Gpak_32_chan_port_config, span_num))
			return -1;

		Gpak_32_chan_port_config.FirstSlotMask2 <<= 1;
		Gpak_32_chan_port_config.SecSlotMask2 <<= 1;
		Gpak_32_chan_port_config.ThirdSlotMask2 <<= 1;
		Gpak_32_chan_port_config.FouthSlotMask2 <<= 1;
		Gpak_32_chan_port_config.FifthSlotMask2 <<= 1;
		Gpak_32_chan_port_config.SixthSlotMask2 <<= 1;
		Gpak_32_chan_port_config.SevenSlotMask2 <<= 1;
		Gpak_32_chan_port_config.EightSlotMask2 <<= 1;
		Gpak_32_chan_port_config.FirstSlotMask3 <<= 1;
		Gpak_32_chan_port_config.SecSlotMask3 <<= 1;
		Gpak_32_chan_port_config.ThirdSlotMask3 <<= 1;
		Gpak_32_chan_port_config.FouthSlotMask3 <<= 1;
		Gpak_32_chan_port_config.FifthSlotMask3 <<= 1;
		Gpak_32_chan_port_config.SixthSlotMask3 <<= 1;
		Gpak_32_chan_port_config.SevenSlotMask3 <<= 1;
		Gpak_32_chan_port_config.EightSlotMask3 <<= 1;
	}

	Gpak_chan_config.EcanParametersA.EcanNlpType = nlp_type;
	Gpak_chan_config.EcanParametersB.EcanNlpType = nlp_type;

	Gpak_chan_config.EcanEnableB = Enabled;

	Gpak_chan_config.EcanEnableA = Disabled;
	for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
		rxt1_card_dsp_ping(rxt1_card, span_num);
	}

	for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
		chan_count = 0;

		for (chan_num = 0; chan_num < rxt1_card->rxt1_spans[span_num]->span.channels;
			 chan_num++) {

			if (rxt1_card->rxt1_spans[span_num]->spantype == TYPE_E1) {
				__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_XLATE1 + span_num,
									0xffff7fff, 0);
				if (chan_num != 15)
					Gpak_chan_config.SoftwareCompand = cmpPCMU;
				else
					Gpak_chan_config.SoftwareCompand = cmpNone;
			} else
				Gpak_chan_config.SoftwareCompand = cmpPCMU;


			msleep(1);
			chan_count++;
			Gpak_chan_config.PcmInSlotA = (span_num + strt) + (chan_num * skip);
			Gpak_chan_config.PcmOutSlotA = (span_num + strt) + (chan_num * skip);
			Gpak_chan_config.PcmInSlotB = (span_num + strt) + (chan_num * skip);
			Gpak_chan_config.PcmOutSlotB = (span_num + strt) + (chan_num * skip);

			if (rxt1_span_dsp_configurechannel
				(rxt1_card, Gpak_chan_config, chan_num, span_num))
				return -1;

			rxt1_chan_ec_disable(rxt1_card, span_num, chan_num);

		}
		printk("R%dT1 %d DSP %d: %d channels configured\n", rxt1_card->numspans,
			   rxt1_card->num + 1, (rxt1_card->num * 4) + span_num + 1, chan_count);

	}

	for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
		rxt1_card->rxt1_spans[span_num]->dsp_up = 1;
		/* we have to coerce the ops pointer to remove the const, because we 
		 *  don't necessarily know what this pointer was supposed to be until now. */
		((struct dahdi_span_ops *)
		 rxt1_card->rxt1_spans[span_num]->span.ops)->echocan_create = rxt1_echocan_create;
	}

	if (debug & DEBUG_DSP) {
		for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
			rxt1_card_dsp_cpustats(rxt1_card, span_num);
			rxt1_card_dsp_framestats(rxt1_card, span_num);
			rxt1_card_dsp_reetframestats(rxt1_card, span_num);

		}

		msleep(100);

		for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
			rxt1_card_dsp_cpustats(rxt1_card, span_num);
			rxt1_card_dsp_framestats(rxt1_card, span_num);
		}

		msleep(100);

		for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
			rxt1_card_dsp_cpustats(rxt1_card, span_num);
			rxt1_card_dsp_framestats(rxt1_card, span_num);
		}

		msleep(100);

		for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
			rxt1_card_dsp_framestats(rxt1_card, span_num);
		}

		msleep(100);

		for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
			rxt1_card_dsp_framestats(rxt1_card, span_num);
		}

		msleep(100);

		for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
			rxt1_card_dsp_framestats(rxt1_card, span_num);
		}

		msleep(100);

		for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
			rxt1_card_dsp_framestats(rxt1_card, span_num);
		}

		msleep(100);

		for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
			rxt1_card_dsp_framestats(rxt1_card, span_num);
		}

		msleep(100);

		for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
			rxt1_card_dsp_framestats(rxt1_card, span_num);
		}

		msleep(100);

		for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
			rxt1_card_dsp_framestats(rxt1_card, span_num);
		}

		msleep(100);

		for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
			rxt1_card_dsp_framestats(rxt1_card, span_num);
		}

		msleep(100);

		for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
			rxt1_card_dsp_framestats(rxt1_card, span_num);
		}

		msleep(100);

		for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
			rxt1_card_dsp_framestats(rxt1_card, span_num);
		}

		msleep(100);

		for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
			rxt1_card_dsp_framestats(rxt1_card, span_num);
		}

		msleep(100);

		for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
			rxt1_card_dsp_framestats(rxt1_card, span_num);
		}

		msleep(100);

		for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
			rxt1_card_dsp_framestats(rxt1_card, span_num);
		}

		msleep(100);

		for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
			rxt1_card_dsp_framestats(rxt1_card, span_num);
		}
	}
#if 1
	__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_HPIC,
						(EC_ON | __rxt1_card_pci_in(rxt1_card, TARG_REGS + RXT1_HPIC)),
						target_regs[RXT1_HPIC].iomask);
#else
	__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_HPIC,
						(~EC_ON & __rxt1_card_pci_in(rxt1_card, TARG_REGS + RXT1_HPIC)),
						target_regs[RXT1_HPIC].iomask);

#endif

	/* this is selecting the A EC (route around the B) */
	__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_ECB1, 0, 0);
	__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_ECB2, 0, 0);
	__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_ECB3, 0, 0);
	__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_ECB4, 0, 0);

	__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_ECA1, 0xffffffff, 0);
	__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_ECA2, 0xffffffff, 0);
	__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_ECA3, 0xffffffff, 0);
	__rxt1_card_pci_out(rxt1_card, TARG_REGS + RXT1_ECA4, 0xffffffff, 0);

	for (span_num = 0; span_num < rxt1_card->numspans; span_num++) {
		rxt1_card_dsp_ping(rxt1_card, span_num);
	}

	rxt1_card->dspwq = create_singlethread_workqueue("rxt1_ec");

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
	INIT_WORK(&rxt1_card->dspwork, echocan_bh, rxt1_card);
#else
	INIT_WORK(&rxt1_card->dspwork, echocan_bh);
#endif

	printk("R%dT1: G168 DSP configured successfully\n", rxt1_card->numspans);

	return (0);
}



static int __devinit rxt1_card_launch(struct rxt1_card_t *rxt1_card)
{
	int span_num;
	if (rxt1_card->rxt1_spans[0]->span.flags & DAHDI_FLAG_REGISTERED)
		return 0;
	printk("R%dT1: Launching card: %d\n", rxt1_card->numspans, rxt1_card->order);

	/* Setup serial parameters and system interface */
	for (span_num = 0; span_num < rxt1_card->numspans; span_num++)
		rxt1_span_serial_setup(rxt1_card, span_num);

	if (dahdi_register(&rxt1_card->rxt1_spans[0]->span, 0)) {
		printk(KERN_ERR "Unable to register span %s\n",
			   rxt1_card->rxt1_spans[0]->span.name);
		return -1;
	}
	if (dahdi_register(&rxt1_card->rxt1_spans[1]->span, 0)) {
		printk(KERN_ERR "Unable to register span %s\n",
			   rxt1_card->rxt1_spans[1]->span.name);
		dahdi_unregister(&rxt1_card->rxt1_spans[0]->span);
		return -1;
	}

	if (rxt1_card->numspans == 4) {
		if (dahdi_register(&rxt1_card->rxt1_spans[2]->span, 0)) {
			printk(KERN_ERR "Unable to register span %s\n",
				   rxt1_card->rxt1_spans[2]->span.name);
			dahdi_unregister(&rxt1_card->rxt1_spans[0]->span);
			dahdi_unregister(&rxt1_card->rxt1_spans[1]->span);
			return -1;
		}
		if (dahdi_register(&rxt1_card->rxt1_spans[3]->span, 0)) {
			printk(KERN_ERR "Unable to register span %s\n",
				   rxt1_card->rxt1_spans[3]->span.name);
			dahdi_unregister(&rxt1_card->rxt1_spans[0]->span);
			dahdi_unregister(&rxt1_card->rxt1_spans[1]->span);
			dahdi_unregister(&rxt1_card->rxt1_spans[2]->span);
			return -1;
		}
	}
	rxt1_card->checktiming = 1;
	__rxt1_card_set_timing_source(rxt1_card, 4, 0, 0);
#ifdef ENABLE_TASKLETS
	tasklet_init(&rxt1_card->t4_tlet, t4_tasklet, (unsigned long) rxt1_card);
#endif

	/* check to see if the hardware version is compativle with this driver version */
	if (rxt1_card->version > 35)
		rxt1_card_init_dsp(rxt1_card);
	else
		printk("R%dT1: Found HW version less than 36, need to upgrade your hardware! \n",
			   rxt1_card->numspans);

	return 0;
}

static int __devinit rxt1_driver_init_one(struct pci_dev *pdev,
										  const struct pci_device_id *ent)
{
	struct rxt1_card_t *rxt1_card;
	struct devtype *dt;
	int x, f;
	int basesize;
	/* used for kmalloc'ing large blocks */
	struct rxt1_span_t *span_block;
	struct dahdi_chan *chan_block;
	struct dahdi_echocan_state *ec_block;
	int chan_count;

	if (pci_enable_device(pdev))
		return -EIO;

	for (x = 0; x < MAX_RXT1_CARDS; x++) {
		if (!rxt1_cards[x])
			break;
	}

	if (x >= MAX_RXT1_CARDS) {
		printk("rxt1: No cards[] slot available!!\n");
		return -ENOMEM;
	}

	rxt1_card = kmalloc(sizeof *rxt1_card, GFP_KERNEL);
	if (rxt1_card == NULL)
		return -ENOMEM;

	memset(rxt1_card, 0x0, sizeof *rxt1_card);

	dt = (struct devtype *) (ent->driver_data);
	if (dt->flags & FLAG_2PORT)
		rxt1_card->numspans = 2;
	else
		rxt1_card->numspans = 4;

	span_block = kmalloc(rxt1_card->numspans * sizeof *span_block, GFP_KERNEL);

	if (span_block == NULL) {
		kfree(rxt1_card);
		return -ENOMEM;
	}

	memset(span_block, 0, rxt1_card->numspans * sizeof *span_block);

	/* Read T1/E1 status or something */
	if (t1e1override > -1)
		rxt1_card->t1e1 = t1e1override;
	else
		rxt1_card->t1e1 = 0x0;

	rxt1_cards[x] = rxt1_card;
	rxt1_card->num = x;
	spin_lock_init(&rxt1_card->reglock);
	basesize = DAHDI_MAX_CHUNKSIZE * 32 * 2 * 4;

	rxt1_card->variety = dt->desc;

	rxt1_card->memaddr = pci_resource_start(pdev, 0);
	rxt1_card->memlen = pci_resource_len(pdev, 0);
	rxt1_card->membase = ioremap(rxt1_card->memaddr, rxt1_card->memlen);

	/* This rids of the Double missed interrupt message after loading */
	rxt1_card->last0 = 1;

	if (pci_request_regions(pdev, rxt1_card->variety))
		printk("R%dT1: Unable to request regions\n", rxt1_card->numspans);

	printk("Found R%dT1 at base address %08lx, remapped to %p\n",
		   rxt1_card->numspans, rxt1_card->memaddr, rxt1_card->membase);

	rxt1_card->dev = pdev;
	rxt1_card->dsp_type = DSP_5510;
	rxt1_card->writechunk =		/* 32 channels, Double-buffer, Read/Write, 4 spans */
		pci_alloc_consistent(pdev, basesize * 2, &rxt1_card->writedma);

	if (!rxt1_card->writechunk) {
		printk("R%dT1: Unable to allocate DMA-able memory\n", rxt1_card->numspans);
		kfree(span_block);
		iounmap(rxt1_card->membase);
		pci_release_regions(pdev);
		rxt1_cards[rxt1_card->num] = NULL;
		kfree(rxt1_card);
		return -ENOMEM;
	}

	/* Read is after the whole write piece (in words) */
	if (memloop == 0)
		rxt1_card->readchunk = rxt1_card->writechunk + basesize / 4;
	else
		rxt1_card->readchunk = rxt1_card->writechunk;	// PC memory loop back

	/* Same thing but in bytes...  */
	rxt1_card->readdma = rxt1_card->writedma + basesize;

	printk("R%dT1: writechunk %p readchunk %p writedma %x readdma %x basesize %x\n",
		   rxt1_card->numspans, rxt1_card->writechunk, rxt1_card->readchunk,
		   rxt1_card->writedma, rxt1_card->readdma, basesize);

	/* Initialize Write/Buffers to all blank data */
	memset((void *) rxt1_card->writechunk, 0x00, basesize);
	if (memloop == 0)
		memset((void *) rxt1_card->readchunk, 0xff, basesize);

	/* Enable bus mastering */
	pci_set_master(pdev);

	/* Keep track of which device we are */
	pci_set_drvdata(pdev, rxt1_card);

	/* Initialize hardware */
	rxt1_card_hardware_init_1(rxt1_card, 1);

#ifdef ENABLE_WORKQUEUES
	char tmp[20];
	sprintf(tmp, "R%dT1[%d]", rxt1_card->numspans, rxt1_card->num);
	rxt1_card->workq = create_workqueue(tmp);
#endif

	/* Allocate pieces we need here */
	for (x = 0; x < rxt1_card->numspans; x++) {
		rxt1_card->rxt1_spans[x] = span_block + x;

		if (rxt1_card->t1e1 & (1 << x)) {
			rxt1_card->rxt1_spans[x]->spantype = TYPE_E1;
		} else if (j1mode) {
			rxt1_card->rxt1_spans[x]->spantype = TYPE_J1;
		} else {
			rxt1_card->rxt1_spans[x]->spantype = TYPE_T1;
		}

		chan_count = rxt1_card->rxt1_spans[x]->spantype == TYPE_E1 ? 31 : 24;
		chan_block = kmalloc(chan_count * sizeof *chan_block, GFP_KERNEL);
		ec_block = kmalloc(chan_count * sizeof *ec_block, GFP_KERNEL);

		if (chan_block == NULL || ec_block == NULL) {
			if (chan_block)
				kfree(chan_block);

			if (ec_block)
				kfree(ec_block);

			/* free all the preceding chans, and all of the ecs.
			 * note that rxt1_card->rxt1_spans[x]->chans[0] is the block kmalloc'd previously.
			 * note that rxt1_card->rxt1_spans[x]->ec[0] is the block kmalloc'd previously.
			 */
			while (--x >= 0) {
				kfree(rxt1_card->rxt1_spans[x]->chans[0]);
				kfree(rxt1_card->rxt1_spans[x]->ec[0]);
			}

			kfree(span_block);
			iounmap(rxt1_card->membase);
			pci_free_consistent(pdev, DAHDI_MAX_CHUNKSIZE * 2 * 2 * 32 * 4,
								(void *) rxt1_card->writechunk, rxt1_card->writedma);
			pci_release_regions(pdev);
			rxt1_cards[rxt1_card->num] = NULL;
			kfree(rxt1_card);
			return -ENOMEM;
		}

		memset(chan_block, 0, chan_count * sizeof *chan_block);
		memset(ec_block, 0, chan_count * sizeof *ec_block);

		for (f = 0; f < chan_count; f++) {
			rxt1_card->rxt1_spans[x]->chans[f] = chan_block + f;
			rxt1_card->rxt1_spans[x]->ec[f] = ec_block + f;
		}

#ifdef ENABLE_WORKQUEUES
		INIT_WORK(&rxt1_card->rxt1_spans[x]->swork, workq_handlespan,
				  rxt1_card->rxt1_spans[x]);
#endif
		rxt1_card->rxt1_spans[x]->spanflags |= dt->flags;
	}

	/* Continue hardware intiialization */
	rxt1_card_hardware_init_2(rxt1_card);

	if (request_irq
		(pdev->irq, rxt1_card_interrupt_gen2, DAHDI_IRQ_SHARED, "rxt1", rxt1_card)) {
		printk("R%dT1: Unable to request IRQ %d\n", rxt1_card->numspans, pdev->irq);
		for (x = 0; x < rxt1_card->numspans; x++) {
			kfree(rxt1_card->rxt1_spans[x]->chans[0]);
			kfree(rxt1_card->rxt1_spans[x]->ec[0]);
		}

		kfree(span_block);
		iounmap(rxt1_card->membase);

		pci_free_consistent(pdev, DAHDI_MAX_CHUNKSIZE * 2 * 2 * 32 * 4,
							(void *) rxt1_card->writechunk, rxt1_card->writedma);

		kfree(rxt1_card);
		pci_release_regions(pdev);
		rxt1_cards[rxt1_card->num] = NULL;
		return -EIO;
	}

	rxt1_card_init_spans(rxt1_card);

	/* Launch cards as appropriate */
	x = 0;
	for (;;) {
		/* Find a card to activate */
		f = 0;
		for (x = 0; rxt1_cards[x]; x++) {
			if (rxt1_cards[x]->order <= highestorder) {
				rxt1_card_launch(rxt1_cards[x]);
				if (rxt1_cards[x]->order == highestorder)
					f = 1;
			}
		}

		/* If we found at least one, increment the highest order and search again, 
		 *    otherwise stop */
		if (f)
			highestorder++;
		else
			break;
	}

	printk("rxt1: Found a Rhino: %s\n", rxt1_card->variety);

	return 0;
}

static int rxt1_card_hardware_stop(struct rxt1_card_t *rxt1_card)
{

	/* Turn off DMA, leave interrupts enabled */
	rxt1_card->stopdma = 1;

	current->state = TASK_UNINTERRUPTIBLE;
	schedule_timeout((25 * HZ) / 1000);

	/* Turn off counter, address, etc */
	rxt1_card_tsi_reset(rxt1_card);
	printk("\nStopped R%dT1, Turned off DMA\n", rxt1_card->numspans);
	return 0;
}

static void __devexit rxt1_driver_remove_one(struct pci_dev *pdev)
{
	struct rxt1_card_t *rxt1_card = pci_get_drvdata(pdev);
	int x;

	if (rxt1_card) {
		/* Stop hardware */
		rxt1_card_hardware_stop(rxt1_card);

		/* Unregister spans */
		if (rxt1_card->rxt1_spans[0]->span.flags & DAHDI_FLAG_REGISTERED)
			dahdi_unregister(&rxt1_card->rxt1_spans[0]->span);
		if (rxt1_card->rxt1_spans[1]->span.flags & DAHDI_FLAG_REGISTERED)
			dahdi_unregister(&rxt1_card->rxt1_spans[1]->span);
		if (rxt1_card->numspans == 4) {
			if (rxt1_card->rxt1_spans[2]->span.flags & DAHDI_FLAG_REGISTERED)
				dahdi_unregister(&rxt1_card->rxt1_spans[2]->span);
			if (rxt1_card->rxt1_spans[3]->span.flags & DAHDI_FLAG_REGISTERED)
				dahdi_unregister(&rxt1_card->rxt1_spans[3]->span);
		}
#ifdef ENABLE_WORKQUEUES
		if (rxt1_card->workq) {
			flush_workqueue(rxt1_card->workq);
			destroy_workqueue(rxt1_card->workq);
		}
#endif

		if (rxt1_card->membase)
			iounmap(rxt1_card->membase);

		pci_release_regions(pdev);

		if (rxt1_card->memaddr)
			release_mem_region(rxt1_card->memaddr, rxt1_card->memlen);

		/* Immediately free resources */
		pci_free_consistent(pdev, DAHDI_MAX_CHUNKSIZE * 2 * 2 * 32 * 4,
							(void *) rxt1_card->writechunk, rxt1_card->writedma);

		free_irq(pdev->irq, rxt1_card);

		rxt1_cards[rxt1_card->num] = NULL;
		pci_set_drvdata(pdev, NULL);

		for (x = 0; x < rxt1_card->numspans; x++) {
			/*
			 * recall that from init_one, a large block is allocated, one
			 *  each for the chans and ec blocks.
			 */
			kfree(rxt1_card->rxt1_spans[x]->chans[0]);
			kfree(rxt1_card->rxt1_spans[x]->ec[0]);
		}

		/*
		 * recall that from init_one, a large block is allocated for all
		 *  the spans for the rxt1_card.
		 */
		kfree(rxt1_card->rxt1_spans[0]);

		kfree(rxt1_card);
	}
}

static struct pci_device_id rxt1_pci_tbl[] __devinitdata = {
	/* vendor  device */
	{0x0b0b, 0x0305, PCI_ANY_ID, PCI_ANY_ID, 0, 0, (unsigned long) &r4t1},
	{0x0b0b, 0x0605, PCI_ANY_ID, PCI_ANY_ID, 0, 0, (unsigned long) &r2t1},
	{0,}
};

static struct pci_driver rxt1_driver = {
  name:"Rhino Equipment 1-2-4 Span T1-E1-J1 PCI Driver",
  probe:rxt1_driver_init_one,
  remove:__devexit_p(rxt1_driver_remove_one),
  suspend:NULL,
  resume:NULL,
  id_table:rxt1_pci_tbl,
};

static int __init rxt1_driver_init(void)
{
	int res;
	res = dahdi_pci_module(&rxt1_driver);
	if (res)
		return -ENODEV;
	return 0;
}

static void __exit rxt1_cleanup(void)
{
	pci_unregister_driver(&rxt1_driver);
}


MODULE_AUTHOR
	("Bob Conklin <helpdesk@rhinoequipment.com>\n\tBryce Chidester <helpdesk@rhinoequipment.com>\n\tMatthew Gessner <helpdesk@rhinoequipment.com");
MODULE_DESCRIPTION("Rhino Equipment 1-2-4 Span T1-E1-J1 PCI Driver ver. " DAHDI_VERSION);
MODULE_VERSION(DAHDI_VERSION);
#ifdef MODULE_LICENSE
MODULE_LICENSE("GPL");
#endif
module_param(debug, int, 0600);
module_param(porboot, int, 0600);
module_param(monitor_mode, int, 0600);
module_param(loopback, int, 0600);
module_param(noburst, int, 0600);
module_param(debugslips, int, 0600);
module_param(polling, int, 0600);
module_param(timingcable, int, 0600);
module_param(t1e1override, int, 0600);
module_param(alarmdebounce, int, 0600);
module_param(j1mode, int, 0600);
module_param(sigmode, int, 0600);
module_param(regdump, int, 0600);
module_param(test_pat, int, 0600);
module_param(recd_off, int, 0600);
module_param(recq_off, int, 0600);
module_param(xmit_off, int, 0600);
module_param(sic3_set, int, 0600);
module_param(insert_idle, int, 0600);
module_param(local_loop, int, 0600);
module_param(double_buffer, int, 0600);
module_param(show_pointers, int, 0600);
module_param(ec_disable_1, int, 0600);
module_param(ec_disable_2, int, 0600);
module_param(ec_disable_3, int, 0600);
module_param(ec_disable_4, int, 0600);
module_param(no_ec, int, 0600);
module_param(nlp_type, int, 0600);
MODULE_PARM_DESC(nlp_type, "0 - off, 1 - mute, 2 - rand, 3 - hoth, 4 - supp");
module_param(gen_clk, int, 0600);


MODULE_DEVICE_TABLE(pci, rxt1_pci_tbl);

module_init(rxt1_driver_init);
module_exit(rxt1_cleanup);
