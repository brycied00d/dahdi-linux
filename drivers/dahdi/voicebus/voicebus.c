/*
 * VoiceBus(tm) Interface Library.
 *
 * Written by Shaun Ruffell <sruffell@digium.com>
 * and based on previous work by Mark Spencer <markster@digium.com>,
 * Matthew Fredrickson <creslin@digium.com>, and
 * Michael Spiceland <mspiceland@digium.com>
 *
 * Copyright (C) 2007-2009 Digium, Inc.
 *
 * All rights reserved.

 * VoiceBus is a registered trademark of Digium.
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

#include <linux/version.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/ioport.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/timer.h>
#include <linux/module.h>

#include <dahdi/kernel.h>
#include "voicebus.h"
#include "vpmadtreg.h"
#include "GpakCust.h"

#define INTERRUPT 0	/* Run the deferred processing in the ISR. */
#define TASKLET 1	/* Run in a tasklet. */
#define TIMER 2		/* Run in a system timer. */
#define WORKQUEUE 3	/* Run in a workqueue. */
#ifndef VOICEBUS_DEFERRED
#define VOICEBUS_DEFERRED INTERRUPT
#endif
#if VOICEBUS_DEFERRED == WORKQUEUE
#define VOICEBUS_ALLOC_FLAGS GFP_KERNEL
#else
#define VOICEBUS_ALLOC_FLAGS GFP_ATOMIC
#endif

/* Define CONFIG_VOICEBUS_SYSFS to create some attributes under the pci device.
 * This is disabled by default because it hasn't been tested on the full range
 * of supported kernels. */
#undef CONFIG_VOICEBUS_SYSFS

#if VOICEBUS_DEFERRED == TIMER
#if HZ < 1000
/* \todo Put an error message here. */
#endif
#endif

/*! The number of descriptors in both the tx and rx descriptor ring. */
#define DRING_SIZE	(1 << 7)  /* Must be a power of 2 */
#define DRING_MASK	(DRING_SIZE-1)

/* Interrupt status' reported in SR_CSR5 */
#define TX_COMPLETE_INTERRUPT 		0x00000001
#define TX_STOPPED_INTERRUPT 		0x00000002
#define TX_UNAVAILABLE_INTERRUPT	0x00000004
#define TX_JABBER_TIMEOUT_INTERRUPT	0x00000008
#define TX_UNDERFLOW_INTERRUPT		0x00000020
#define RX_COMPLETE_INTERRUPT		0x00000040
#define RX_UNAVAILABLE_INTERRUPT	0x00000080
#define RX_STOPPED_INTERRUPT		0x00000100
#define RX_WATCHDOG_TIMEOUT_INTERRUPT	0x00000200
#define TIMER_INTERRUPT			0x00000800
#define FATAL_BUS_ERROR_INTERRUPT	0x00002000
#define ABNORMAL_INTERRUPT_SUMMARY	0x00008000
#define NORMAL_INTERRUPT_SUMMARY	0x00010000

#define SR_CSR5				0x0028
#define NAR_CSR6			0x0030

#define IER_CSR7			0x0038
#define		CSR7_TCIE		0x00000001 /* tx complete */
#define		CSR7_TPSIE		0x00000002 /* tx processor stopped */
#define		CSR7_TDUIE		0x00000004 /* tx desc unavailable */
#define 	CSR7_TUIE		0x00000020 /* tx underflow */
#define		CSR7_RCIE		0x00000040 /* rx complete */
#define 	CSR7_RUIE		0x00000080 /* rx desc unavailable */
#define		CSR7_RSIE		0x00000100 /* rx processor stopped */
#define 	CSR7_FBEIE		0x00002000 /* fatal bus error */
#define		CSR7_AIE		0x00008000 /* abnormal enable */
#define 	CSR7_NIE		0x00010000 /* normal enable */

#define DEFAULT_INTERRUPTS	(CSR7_TCIE | CSR7_TPSIE | CSR7_TDUIE |  \
				 CSR7_RUIE | CSR7_RSIE | CSR7_FBEIE | \
				 CSR7_AIE | CSR7_NIE)

#define CSR9				0x0048
#define 	CSR9_MDC		0x00010000
#define 	CSR9_MDO		0x00020000
#define 	CSR9_MMC		0x00040000
#define 	CSR9_MDI		0x00080000

#define OWN_BIT (1 << 31)

/* In memory structure shared by the host and the adapter. */
struct voicebus_descriptor {
	volatile __le32 des0;
	volatile __le32 des1;
	volatile __le32 buffer1;
	volatile __le32 container; /* Unused */
} __attribute__((packed));

struct voicebus_descriptor_list {
	/* Pointer to an array of descriptors to give to hardware. */
	struct voicebus_descriptor *desc;
	/* Read completed buffers from the head. */
	unsigned int 	head;
	/* Write ready buffers to the tail. */
	unsigned int 	tail;
	/* Array to save the kernel virtual address of pending buffers. */
	void  		*pending[DRING_SIZE];
	/* PCI Bus address of the descriptor list. */
	dma_addr_t	desc_dma;
	/*! The number of buffers currently submitted to the hardware. */
	atomic_t 	count;
	/*! The number of bytes to pad each descriptor for cache alignment. */
	unsigned int	padding;
};

/**
 * struct voicebus -
 *
 * @tx_idle_vbb:
 * @tx_idle_vbb_dma_addr:
 * @max_latency: Do not allow the driver to automatically insert more than this
 * 		 much latency to the tdm stream by default.
 * @count:	The number of non-idle buffers that we should be expecting.
 */
struct voicebus {
	/*! The system pci device for this VoiceBus interface. */
	struct pci_dev *pdev;
	/*! Protects access to card registers and this structure. You should
	 * hold this lock before accessing most of the members of this data
	 * structure or the card registers. */
	spinlock_t lock;
	/*! The size of the transmit and receive buffers for this card. */
	u32 framesize;
	/*! The number of u32s in the host system cache line. */
	u8 cache_line_size;
	/*! Pool to allocate memory for the tx and rx descriptor rings. */
	struct voicebus_descriptor_list rxd;
	struct voicebus_descriptor_list txd;
	void 		*idle_vbb;
	dma_addr_t	idle_vbb_dma_addr;
	/*! Level of debugging information.  0=None, 5=Insane. */
	atomic_t debuglevel;
	/*! Cache of buffer objects. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
	kmem_cache_t *buffer_cache;
#else
	struct kmem_cache *buffer_cache;
#endif
	/*! Base address of the VoiceBus interface registers in I/O space. */
	u32 iobase;
	/*! The IRQ line for this VoiceBus interface. */
	unsigned int irq;
#if VOICEBUS_DEFERRED == WORKQUEUE
	/*! Process buffers in the context of this workqueue. */
	struct workqueue_struct *workqueue;
	/*! Work item to process tx / rx buffers. */
	struct work_struct workitem;
#elif VOICEBUS_DEFERRED == TASKLET
	/*! Process buffers in the context of a tasklet. */
	struct tasklet_struct 	tasklet;
#elif VOICEBUS_DEFERRED == TIMER
	/*! Process buffers in a timer without generating interrupts. */
	struct timer_list timer;
#endif
	/*! Callback function to board specific module to process frames. */
	void (*handle_receive)(void *vbb, void *context);
	void (*handle_transmit)(void *vbb, void *context);
	/*! Data to pass to the receive and transmit callback. */
	void *context;
	struct completion stopped_completion;
	/*! Flags */
	unsigned long flags;
	/*! Number of tx buffers to queue up before enabling interrupts. */
	unsigned int 	min_tx_buffer_count;
	unsigned int	max_latency;
	void		*vbb_stash[DRING_SIZE];
	unsigned int	count;
};

static inline void handle_transmit(struct voicebus *vb, void *vbb)
{
	vb->handle_transmit(vbb, vb->context);
}

/*
 * Use the following macros to lock the VoiceBus interface, and it won't
 * matter if the deferred processing is running inside the interrupt handler,
 * in a tasklet, or in a workqueue.
 */
#if VOICEBUS_DEFERRED == WORKQUEUE
/*
 * When the deferred processing is running in a workqueue, voicebus will never
 * be locked from the context of the interrupt handler, and therefore we do
 * not need to lock interrupts.
 */
#define LOCKS_VOICEBUS
#define LOCKS_FROM_DEFERRED
#define VBLOCK(_vb_) 			spin_lock(&((_vb_)->lock))
#define VBUNLOCK(_vb_)			spin_unlock(&((_vb_)->lock))
#define VBLOCK_FROM_DEFERRED(_vb_) 	spin_lock(&((_vb_)->lock))
#define VBUNLOCK_FROM_DEFERRED(_vb_)	spin_lock(&((_vb_)->lock))
#else
#define LOCKS_VOICEBUS			unsigned long _irqflags
#define LOCKS_FROM_DEFERRED
#define VBLOCK(_vb_) 			spin_lock_irqsave(&((_vb_)->lock), _irqflags)
#define VBUNLOCK(_vb_)			spin_unlock_irqrestore(&((_vb_)->lock), _irqflags)
#define VBLOCK_FROM_DEFERRED(_vb_) 	spin_lock(&((_vb_)->lock))
#define VBUNLOCK_FROM_DEFERRED(_vb_)	spin_lock(&((_vb_)->lock))
#endif

/* Bit definitions for struct voicebus.flags */
#define TX_UNDERRUN			1
#define RX_UNDERRUN			2
#define IN_DEFERRED_PROCESSING		3
#define STOP				4
#define STOPPED				5
#define LATENCY_LOCKED			6

#if VOICEBUS_DEFERRED == WORKQUEUE
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 18)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
/*! \brief Make the current task real-time. */
static void
vb_setup_deferred(void *data)
#else
static void
vb_setup_deferred(struct work_struct *work)
#endif
{
	struct sched_param param = { .sched_priority = 99 };
	sched_setscheduler(current, SCHED_FIFO, &param);
}
/*! \brief Schedule a work item to make the voicebus workqueue real-time. */
static void
vb_set_workqueue_priority(struct voicebus *vb)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
	DECLARE_WORK(deferred_setup, vb_setup_deferred, NULL);
#else
	DECLARE_WORK(deferred_setup, vb_setup_deferred);
#endif
	queue_work(vb->workqueue, &deferred_setup);
	flush_workqueue(vb->workqueue);
}
#endif
#endif

static inline struct voicebus_descriptor *
vb_descriptor(const struct voicebus_descriptor_list *dl,
	      const unsigned int index)
{
	struct voicebus_descriptor *d;
	d = (struct voicebus_descriptor *)((u8*)dl->desc +
		((sizeof(*d) + dl->padding) * index));
	return d;
}

static int
vb_initialize_descriptors(struct voicebus *vb, struct voicebus_descriptor_list *dl,
	u32 des1, unsigned int direction)
{
	int i;
	struct voicebus_descriptor *d;
	const u32 END_OF_RING = 0x02000000;

	BUG_ON(!dl);

	/*
	 * Add some padding to each descriptor to ensure that they are
	 * aligned on host system cache-line boundaries, but only for the
	 * cache-line sizes that we support.
	 *
	 */
	if ((0x08 == vb->cache_line_size) || (0x10 == vb->cache_line_size) ||
	    (0x20 == vb->cache_line_size)) {
		dl->padding = (vb->cache_line_size*sizeof(u32)) - sizeof(*d);
	} else {
		dl->padding = 0;
	}

	dl->desc = pci_alloc_consistent(vb->pdev,
		(sizeof(*d) + dl->padding) * DRING_SIZE, &dl->desc_dma);
	if (!dl->desc)
		return -ENOMEM;

	memset(dl->desc, 0, (sizeof(*d) + dl->padding) * DRING_SIZE);
	for (i = 0; i < DRING_SIZE; ++i) {
		d = vb_descriptor(dl, i);
		d->des1 = des1;
	}
	d->des1 |= cpu_to_le32(END_OF_RING);
	atomic_set(&dl->count, 0);
	return 0;
}

#define OWNED(_d_) (((_d_)->des0)&OWN_BIT)
#define SET_OWNED(_d_) do { wmb(); (_d_)->des0 |= OWN_BIT; wmb(); } while (0)

static int
vb_initialize_tx_descriptors(struct voicebus *vb)
{
	int i;
	int des1 = 0xe4800000 | vb->framesize;
	struct voicebus_descriptor *d;
	struct voicebus_descriptor_list *dl = &vb->txd;
	const u32 END_OF_RING = 0x02000000;

	WARN_ON(!dl);
	WARN_ON((NULL == vb->idle_vbb) || (0 == vb->idle_vbb_dma_addr));

	/*
	 * Add some padding to each descriptor to ensure that they are
	 * aligned on host system cache-line boundaries, but only for the
	 * cache-line sizes that we support.
	 *
	 */
	if ((0x08 == vb->cache_line_size) || (0x10 == vb->cache_line_size) ||
	    (0x20 == vb->cache_line_size)) {
		dl->padding = (vb->cache_line_size*sizeof(u32)) - sizeof(*d);
	} else {
		dl->padding = 0;
	}

	dl->desc = pci_alloc_consistent(vb->pdev,
					(sizeof(*d) + dl->padding) *
					DRING_SIZE, &dl->desc_dma);
	if (!dl->desc)
		return -ENOMEM;

	memset(dl->desc, 0, (sizeof(*d) + dl->padding) * DRING_SIZE);
	for (i = 0; i < DRING_SIZE; ++i) {
		d = vb_descriptor(dl, i);
		d->des1 = des1;
		d->buffer1 = vb->idle_vbb_dma_addr;
		dl->pending[i] = vb->idle_vbb;
		SET_OWNED(d);
	}
	d->des1 |= cpu_to_le32(END_OF_RING);
	atomic_set(&dl->count, 0);
	return 0;
}

static int
vb_initialize_rx_descriptors(struct voicebus *vb)
{
	return vb_initialize_descriptors(
		vb, &vb->rxd, vb->framesize, DMA_FROM_DEVICE);
}

/*! \brief  Use to set the minimum number of buffers queued to the hardware
 * before enabling interrupts.
 */
int
voicebus_set_minlatency(struct voicebus *vb, unsigned int ms)
{
	LOCKS_VOICEBUS;
	/*
	 * One millisecond of latency means that we have 3 buffers pending,
	 * since two are always going to be waiting in the TX fifo on the
	 * interface chip.
	 *
	 */
#define MESSAGE "%d ms is an invalid value for minumum latency.  Setting to %d ms.\n"
	if (DRING_SIZE < ms) {
		dev_warn(&vb->pdev->dev, MESSAGE, ms, DRING_SIZE);
		return -EINVAL;
	} else if (VOICEBUS_DEFAULT_LATENCY > ms) {
		dev_warn(&vb->pdev->dev, MESSAGE, ms, VOICEBUS_DEFAULT_LATENCY);
		return -EINVAL;
	}
	VBLOCK(vb);
	vb->min_tx_buffer_count = ms;
	VBUNLOCK(vb);
	return 0;
}
EXPORT_SYMBOL(voicebus_set_minlatency);

void
voicebus_get_handlers(struct voicebus *vb, void **handle_receive,
	void **handle_transmit, void **context)
{
	LOCKS_VOICEBUS;
	BUG_ON(!handle_receive);
	BUG_ON(!handle_transmit);
	BUG_ON(!context);
	VBLOCK(vb);
	*handle_receive = vb->handle_receive;
	*handle_transmit = vb->handle_transmit;
	*context = vb->context;
	VBUNLOCK(vb);
	return;
}
EXPORT_SYMBOL(voicebus_get_handlers);

void
voicebus_set_handlers(struct voicebus *vb,
	void (*handle_receive)(void *buffer, void *context),
	void (*handle_transmit)(void *buffer, void *context),
	void *context)
{
	LOCKS_VOICEBUS;
	BUG_ON(!handle_receive);
	BUG_ON(!handle_transmit);
	BUG_ON(!context);
	VBLOCK(vb);
	vb->handle_receive = handle_receive;
	vb->handle_transmit = handle_transmit;
	vb->context = context;
	VBUNLOCK(vb);
}
EXPORT_SYMBOL(voicebus_set_handlers);

/*! \brief Returns the number of buffers currently on the transmit queue. */
int
voicebus_current_latency(struct voicebus *vb)
{
	LOCKS_VOICEBUS;
	int latency;
	VBLOCK(vb);
	latency = vb->min_tx_buffer_count;
	VBUNLOCK(vb);
	return latency;
}
EXPORT_SYMBOL(voicebus_current_latency);

/**
 * voicebus_lock_latency() - Do not increase the latency during underruns.
 *
 */
void voicebus_lock_latency(struct voicebus *vb)
{
	set_bit(LATENCY_LOCKED, &vb->flags);
}
EXPORT_SYMBOL(voicebus_lock_latency);

/**
 * voicebus_unlock_latency() - Bump up the latency during underruns.
 *
 */
void voicebus_unlock_latency(struct voicebus *vb)
{
	clear_bit(LATENCY_LOCKED, &vb->flags);
}
EXPORT_SYMBOL(voicebus_unlock_latency);

/**
 * voicebus_is_latency_locked() - Return 1 if latency is currently locked.
 *
 */
int voicebus_is_latency_locked(const struct voicebus *vb)
{
	return test_bit(LATENCY_LOCKED, &vb->flags);
}
EXPORT_SYMBOL(voicebus_is_latency_locked);

/*!
 * \brief Read one of the hardware control registers without acquiring locks.
 */
static inline u32
__vb_getctl(struct voicebus *vb, u32 addr)
{
	return le32_to_cpu(inl(vb->iobase + addr));
}

/*!
 * \brief Read one of the hardware control registers with locks held.
 */
static inline u32
vb_getctl(struct voicebus *vb, u32 addr)
{
	LOCKS_VOICEBUS;
	u32 val;
	VBLOCK(vb);
	val = __vb_getctl(vb, addr);
	VBUNLOCK(vb);
	return val;
}

/*!
 * \brief Returns whether or not the interface is running.
 *
 * NOTE:  Running in this case means whether or not the hardware reports the
 *        transmit processor in any state but stopped.
 *
 * \return 1 of the process is stopped, 0 if running.
 */
static int
vb_is_stopped(struct voicebus *vb)
{
	u32 reg;
	reg = vb_getctl(vb, SR_CSR5);
	reg = (reg >> 17) & 0x38;
	return (0 == reg) ? 1 : 0;
}

static void
vb_cleanup_tx_descriptors(struct voicebus *vb)
{
	unsigned int i;
	struct voicebus_descriptor_list *dl = &vb->txd;
	struct voicebus_descriptor *d;

	BUG_ON(!vb_is_stopped(vb));

	for (i = 0; i < DRING_SIZE; ++i) {
		d = vb_descriptor(dl, i);
		if (d->buffer1 && (d->buffer1 != vb->idle_vbb_dma_addr)) {
			WARN_ON(!dl->pending[i]);
			dma_unmap_single(&vb->pdev->dev, d->buffer1,
					 vb->framesize, DMA_TO_DEVICE);
			voicebus_free(vb, dl->pending[i]);
		}
		d->buffer1 = vb->idle_vbb_dma_addr;
		dl->pending[i] = vb->idle_vbb;
		SET_OWNED(d);
	}
	/* Send out two idle buffers to start because sometimes the first buffer
	 * doesn't make it back to us. */
	dl->head = dl->tail = 2;
	atomic_set(&dl->count, 0);
}

static void
vb_cleanup_rx_descriptors(struct voicebus *vb)
{
	unsigned int i;
	struct voicebus_descriptor_list *dl = &vb->rxd;
	struct voicebus_descriptor *d;

	BUG_ON(!vb_is_stopped(vb));

	for (i = 0; i < DRING_SIZE; ++i) {
		d = vb_descriptor(dl, i);
		if (d->buffer1) {
			dma_unmap_single(&vb->pdev->dev, d->buffer1,
					 vb->framesize, DMA_FROM_DEVICE);
			d->buffer1 = 0;
			BUG_ON(!dl->pending[i]);
			voicebus_free(vb, dl->pending[i]);
			dl->pending[i] = NULL;
		}
		d->des0 &= ~OWN_BIT;
	}
	dl->head = 0;
	dl->tail = 0;
	atomic_set(&dl->count, 0);
}

static void vb_cleanup_descriptors(struct voicebus *vb,
				   struct voicebus_descriptor_list *dl)
{
	if (dl == &vb->txd)
		vb_cleanup_tx_descriptors(vb);
	else
		vb_cleanup_rx_descriptors(vb);
}

static void
vb_free_descriptors(struct voicebus *vb, struct voicebus_descriptor_list *dl)
{
	if (NULL == dl->desc) {
		WARN_ON(1);
		return;
	}
	vb_cleanup_descriptors(vb, dl);
	pci_free_consistent(
		vb->pdev,
		(sizeof(struct voicebus_descriptor)+dl->padding)*DRING_SIZE,
		dl->desc, dl->desc_dma);
}

/*!
 * \brief Write one of the hardware control registers without acquiring locks.
 */
static inline void
__vb_setctl(struct voicebus *vb, u32 addr, u32 val)
{
	wmb();
	outl(cpu_to_le32(val), vb->iobase + addr);
}

/*!
 * \brief Write one of the hardware control registers with locks held.
 */
static inline void
vb_setctl(struct voicebus *vb, u32 addr, u32 val)
{
	LOCKS_VOICEBUS;
	VBLOCK(vb);
	__vb_setctl(vb, addr, val);
	VBUNLOCK(vb);
}

static int
__vb_sdi_clk(struct voicebus *vb, u32 *sdi)
{
	unsigned int ret;
	*sdi &= ~CSR9_MDC;
	__vb_setctl(vb, 0x0048, *sdi);
	ret = __vb_getctl(vb, 0x0048);
	*sdi |= CSR9_MDC;
	__vb_setctl(vb, 0x0048, *sdi);
	return (ret & CSR9_MDI) ? 1 : 0;
}

static void
__vb_sdi_sendbits(struct voicebus *vb, u32 bits, int count, u32 *sdi)
{
	*sdi &= ~CSR9_MMC;
	__vb_setctl(vb, 0x0048, *sdi);
	while (count--) {

		if (bits & (1 << count))
			*sdi |= CSR9_MDO;
		else
			*sdi &= ~CSR9_MDO;

		__vb_sdi_clk(vb, sdi);
	}
}

static void
vb_setsdi(struct voicebus *vb, int addr, u16 val)
{
	LOCKS_VOICEBUS;
	u32 bits;
	u32 sdi = 0;
	/* Send preamble */
	bits = 0xffffffff;
	VBLOCK(vb);
	__vb_sdi_sendbits(vb, bits, 32, &sdi);
	bits = (0x5 << 12) | (1 << 7) | (addr << 2) | 0x2;
	__vb_sdi_sendbits(vb, bits, 16, &sdi);
	__vb_sdi_sendbits(vb, val, 16, &sdi);
	VBUNLOCK(vb);
}

static void
vb_enable_io_access(struct voicebus *vb)
{
	LOCKS_VOICEBUS;
	u32 reg;
	BUG_ON(!vb->pdev);
	VBLOCK(vb);
	pci_read_config_dword(vb->pdev, 0x0004, &reg);
	reg |= 0x00000007;
	pci_write_config_dword(vb->pdev, 0x0004, reg);
	VBUNLOCK(vb);
}

/*! \todo Insert comments...
 * context: !in_interrupt()
 */
void*
voicebus_alloc(struct voicebus *vb)
{
	void *vbb;
	vbb = kmem_cache_alloc(vb->buffer_cache, VOICEBUS_ALLOC_FLAGS);
	return vbb;
}

void
voicebus_setdebuglevel(struct voicebus *vb, u32 level)
{
	atomic_set(&vb->debuglevel, level);
}
EXPORT_SYMBOL(voicebus_setdebuglevel);

int
voicebus_getdebuglevel(struct voicebus *vb)
{
	return atomic_read(&vb->debuglevel);
}
EXPORT_SYMBOL(voicebus_getdebuglevel);

/*! \brief Resets the voicebus hardware interface. */
static int
vb_reset_interface(struct voicebus *vb)
{
	unsigned long timeout;
	u32 reg;
	u32 pci_access;
	const u32 DEFAULT_PCI_ACCESS = 0xfff80002;
	BUG_ON(in_interrupt());

	switch (vb->cache_line_size) {
	case 0x08:
		pci_access = DEFAULT_PCI_ACCESS | (0x1 << 14);
		break;
	case 0x10:
		pci_access = DEFAULT_PCI_ACCESS | (0x2 << 14);
		break;
	case 0x20:
		pci_access = DEFAULT_PCI_ACCESS | (0x3 << 14);
		break;
	default:
		if (atomic_read(&vb->debuglevel)) {
			dev_warn(&vb->pdev->dev, "Host system set a cache "
				 "size of %d which is not supported. "
				 "Disabling memory write line and memory "
				 "read line.\n", vb->cache_line_size);
		}
		pci_access = 0xfe584202;
		break;
	}

	/* The transmit and receive descriptors will have the same padding. */
	pci_access |= ((vb->txd.padding / sizeof(u32)) << 2) & 0x7c;

	vb_setctl(vb, 0x0000, pci_access | 1);

	timeout = jiffies + HZ/10; /* 100ms interval */
	do {
		reg = vb_getctl(vb, 0x0000);
	} while ((reg & 0x00000001) && time_before(jiffies, timeout));

	if (reg & 0x00000001) {
		dev_warn(&vb->pdev->dev, "Hardware did not come out of reset "
			 "within 100ms!");
		return -EIO;
	}

	vb_setctl(vb, 0x0000, pci_access);

	return 0;
}

static int
vb_initialize_interface(struct voicebus *vb)
{
	u32 reg;

	vb_cleanup_tx_descriptors(vb);
	vb_cleanup_rx_descriptors(vb);

	/* Pass bad packets, runt packets, disable SQE function,
	 * store-and-forward */
	vb_setctl(vb, 0x0030, 0x00280048);
	/* ...disable jabber and the receive watchdog. */
	vb_setctl(vb, 0x0078, 0x00000013);

	/* Tell the card where the descriptors are in host memory. */
	vb_setctl(vb, 0x0020, (u32)vb->txd.desc_dma);
	vb_setctl(vb, 0x0018, (u32)vb->rxd.desc_dma);

	reg = vb_getctl(vb, 0x00fc);
	vb_setctl(vb, 0x00fc, (reg & ~0x7) | 0x7);
	vb_setsdi(vb, 0x00, 0x0100);
	vb_setsdi(vb, 0x16, 0x2100);

	reg = vb_getctl(vb, 0x00fc);

	vb_setctl(vb, 0x00fc, (reg & ~0x7) | 0x4);
	vb_setsdi(vb, 0x00, 0x0100);
	vb_setsdi(vb, 0x16, 0x2100);
	reg = vb_getctl(vb, 0x00fc);

	/*
	 * The calls to setsdi above toggle the reset line of the CPLD.  Wait
	 * here to give the CPLD time to stabilize after reset.
	 */
	msleep(10);

	return ((reg&0x7) == 0x4) ? 0 : -EIO;
}

#ifdef DBG
static void
dump_descriptor(struct voicebus *vb, struct voicebus_descriptor *d)
{
	VB_PRINTK(vb, DEBUG, "Displaying descriptor at address %08x\n", (unsigned int)d);
	VB_PRINTK(vb, DEBUG, "   des0:      %08x\n", d->des0);
	VB_PRINTK(vb, DEBUG, "   des1:      %08x\n", d->des1);
	VB_PRINTK(vb, DEBUG, "   buffer1:   %08x\n", d->buffer1);
	VB_PRINTK(vb, DEBUG, "   container: %08x\n", d->container);
}

static void
show_buffer(struct voicebus *vb, void *vbb)
{
	int x;
	unsigned char *c;
	c = vbb;
	printk(KERN_DEBUG "Packet %d\n", count);
	printk(KERN_DEBUG "");
	for (x = 1; x <= vb->framesize; ++x) {
		printk("%02x ", c[x]);
		if (x % 16 == 0)
			printk("\n");
	}
	printk(KERN_DEBUG "\n\n");
}
#endif

/**
 * voicebus_transmit - Queue a buffer on the hardware descriptor ring.
 *
 */
int voicebus_transmit(struct voicebus *vb, void *vbb)
{
	struct voicebus_descriptor *d;
	struct voicebus_descriptor_list *dl = &vb->txd;

	d = vb_descriptor(dl, dl->tail);

	if (unlikely(d->buffer1 != vb->idle_vbb_dma_addr)) {
		if (printk_ratelimit())
			dev_warn(&vb->pdev->dev, "Dropping tx buffer buffer\n");
		voicebus_free(vb, vbb);
		return -EFAULT;
	}

	dl->pending[dl->tail] = vbb;
	dl->tail = (++(dl->tail)) & DRING_MASK;
	d->buffer1 = dma_map_single(&vb->pdev->dev, vbb,
				    vb->framesize, DMA_TO_DEVICE);
	SET_OWNED(d); /* That's it until the hardware is done with it. */
	atomic_inc(&dl->count);
	return 0;
}
EXPORT_SYMBOL(voicebus_transmit);

/*!
 * \brief Give a frame to the hardware to use for receiving.
 *
 */
static inline int
vb_submit_rxb(struct voicebus *vb, void *vbb)
{
	struct voicebus_descriptor *d;
	struct voicebus_descriptor_list *dl = &vb->rxd;
	unsigned int tail = dl->tail;

	d = vb_descriptor(dl, tail);

	if (unlikely(d->buffer1)) {
		/* Do not overwrite a buffer that is still in progress. */
		WARN_ON(1);
		voicebus_free(vb, vbb);
		return -EBUSY;
	}

	dl->pending[tail] = vbb;
	dl->tail = (++tail) & DRING_MASK;
	d->buffer1 = dma_map_single(&vb->pdev->dev, vbb,
				    vb->framesize, DMA_FROM_DEVICE);
	SET_OWNED(d); /* That's it until the hardware is done with it. */
	atomic_inc(&dl->count);
	return 0;
}

/*!
 * \brief Remove the next completed transmit buffer (txb) from the tx
 * descriptor ring.
 *
 * NOTE:  This function doesn't need any locking because only one instance is
 * 	  ever running on the deferred processing routine and it only looks at
 * 	  the head pointer. The deferred routine should only ever be running
 * 	  on one processor at a time (no multithreaded workqueues allowed!)
 *
 * Context: Must be called from the voicebus deferred workqueue.
 *
 * \return Pointer to buffer, or NULL if not available.
 */
static inline void *
vb_get_completed_txb(struct voicebus *vb)
{
	struct voicebus_descriptor_list *dl = &vb->txd;
	struct voicebus_descriptor *d;
	void *vbb;
	unsigned int head = dl->head;

	d = vb_descriptor(dl, head);

	if (OWNED(d) || (d->buffer1 == vb->idle_vbb_dma_addr))
		return NULL;

	dma_unmap_single(&vb->pdev->dev, d->buffer1,
			 vb->framesize, DMA_TO_DEVICE);

	vbb = dl->pending[head];
	dl->head = (++head) & DRING_MASK;
	d->buffer1 = vb->idle_vbb_dma_addr;
	SET_OWNED(d);
	atomic_dec(&dl->count);
	return vbb;
}

static inline void *
vb_get_completed_rxb(struct voicebus *vb)
{
	struct voicebus_descriptor *d;
	struct voicebus_descriptor_list *dl = &vb->rxd;
	unsigned int head = dl->head;
	void *vbb;

	d = vb_descriptor(dl, head);

	if ((0 == d->buffer1) || OWNED(d))
		return NULL;

	dma_unmap_single(&vb->pdev->dev, d->buffer1,
			 vb->framesize, DMA_FROM_DEVICE);
	vbb = dl->pending[head];
	dl->head = (++head) & DRING_MASK;
	d->buffer1 = 0;
	atomic_dec(&dl->count);
	return vbb;
}

/*!
 * \brief Free a buffer for reuse.
 *
 */
void
voicebus_free(struct voicebus *vb, void *vbb)
{
	kmem_cache_free(vb->buffer_cache, vbb);
}

/*!
 * \brief Instruct the hardware to check for a new tx descriptor.
 */
static inline void
__vb_tx_demand_poll(struct voicebus *vb)
{
	__vb_setctl(vb, 0x0008, 0x00000000);
}

/*!
 * \brief Command the hardware to check if it owns the next transmit
 * descriptor.
 */
static void
vb_tx_demand_poll(struct voicebus *vb)
{
	LOCKS_VOICEBUS;
	VBLOCK(vb);
	__vb_tx_demand_poll(vb);
	VBUNLOCK(vb);
}

/*!
 * \brief Command the hardware to check if it owns the next receive
 * descriptor.
 */
static inline void
__vb_rx_demand_poll(struct voicebus *vb)
{
	__vb_setctl(vb, 0x0010, 0x00000000);
}

static void
vb_rx_demand_poll(struct voicebus *vb)
{
	LOCKS_VOICEBUS;
	VBLOCK(vb);
	__vb_rx_demand_poll(vb);
	VBUNLOCK(vb);
}

static void
__vb_enable_interrupts(struct voicebus *vb)
{
	__vb_setctl(vb, IER_CSR7, DEFAULT_INTERRUPTS);
}

static void
__vb_disable_interrupts(struct voicebus *vb)
{
	__vb_setctl(vb, IER_CSR7, 0);
}

static void
vb_disable_interrupts(struct voicebus *vb)
{
	LOCKS_VOICEBUS;
	VBLOCK(vb);
	__vb_disable_interrupts(vb);
	VBUNLOCK(vb);
}

/*!
 * \brief Starts the VoiceBus interface.
 *
 * When the VoiceBus interface is started, it is actively transferring
 * frames to and from the backend of the card.  This means the card will
 * generate interrupts.
 *
 * This function should only be called from process context, with interrupts
 * enabled, since it can sleep while running the self checks.
 *
 * \return zero on success. -EBUSY if device is already running.
 */
int
voicebus_start(struct voicebus *vb)
{
	LOCKS_VOICEBUS;
	u32 reg;
	int i;
	void *vbb;
	int ret;

	WARN_ON(pci_get_drvdata(vb->pdev) != vb);
	if (pci_get_drvdata(vb->pdev) != vb)
		return -EFAULT;

	if (!vb_is_stopped(vb))
		return -EBUSY;

	ret = vb_reset_interface(vb);
	if (ret)
		return ret;
	ret = vb_initialize_interface(vb);
	if (ret)
		return ret;

	/* We must set up a minimum of three buffers to start with, since two
	 * are immediately read into the TX FIFO, and the descriptor of the
	 * third is read as soon as the first buffer is done.
	 */

	/*
	 * NOTE: handle_transmit is normally only called in the context of the
	 *  deferred processing thread.  Since the deferred processing thread
	 *  is known to not be running at this point, it is safe to call the
	 *  handle transmit as if it were.
	 */
	/* Ensure that all the rx slots are ready for a buffer. */
	for (i = 0; i < DRING_SIZE; ++i) {
		vbb = voicebus_alloc(vb);
		if (unlikely(NULL == vbb)) {
			BUG_ON(1);
			/* \todo I need to make sure the driver can recover
			 * from this condition. .... */
		} else {
			vb_submit_rxb(vb, vbb);
		}
	}

	for (i = 0; i < vb->min_tx_buffer_count; ++i) {
		vbb = voicebus_alloc(vb);

		if (unlikely(NULL == vbb))
			BUG_ON(1);
		else
			handle_transmit(vb, vbb);

	}

	VBLOCK(vb);
	clear_bit(STOP, &vb->flags);
	clear_bit(STOPPED, &vb->flags);
#if VOICEBUS_DEFERRED == TIMER
	vb->timer.expires = jiffies + HZ/1000;
	add_timer(&vb->timer);
#else
	/* Clear the interrupt status register. */
	__vb_setctl(vb, SR_CSR5, 0xffffffff);
	__vb_enable_interrupts(vb);
#endif
	/* Start the transmit and receive processors. */
	reg = __vb_getctl(vb, 0x0030);
	__vb_setctl(vb, 0x0030, reg|0x00002002);
	/* Tell the interface to poll the tx and rx descriptors. */
	__vb_rx_demand_poll(vb);
	__vb_tx_demand_poll(vb);
	VBUNLOCK(vb);

	BUG_ON(vb_is_stopped(vb));

	return 0;
}
EXPORT_SYMBOL(voicebus_start);

static void
vb_clear_start_transmit_bit(struct voicebus *vb)
{
	LOCKS_VOICEBUS;
	u32 reg;
	VBLOCK(vb);
	reg = __vb_getctl(vb, NAR_CSR6);
	reg &= ~0x00002000;
	__vb_setctl(vb, NAR_CSR6, reg);
	VBUNLOCK(vb);
}

static void
vb_clear_start_receive_bit(struct voicebus *vb)
{
	LOCKS_VOICEBUS;
	u32 reg;
	VBLOCK(vb);
	reg = __vb_getctl(vb, NAR_CSR6);
	reg &= ~0x00000002;
	__vb_setctl(vb, NAR_CSR6, reg);
	VBUNLOCK(vb);
}

static unsigned long
vb_wait_for_completion_timeout(struct completion *x, unsigned long timeout)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 11)
	/* There is a race condition here.  If x->done is reset to 0
	 * before the call to wait_for_completion after this thread wakes.
	 */
	timeout = wait_event_timeout(x->wait, x->done, timeout);
	if (timeout)
		wait_for_completion(x);

	return timeout;
#else
	return wait_for_completion_timeout(x, timeout);
#endif
}

/*!
 * \brief Stops the VoiceBus interface.
 *
 * Stops the VoiceBus interface and waits for any outstanding DMA transactions
 * to complete.  When this functions returns the VoiceBus interface tx and rx
 * states will both be suspended.
 *
 * Only call this function from process context, with interrupt enabled,
 * without any locks held since it sleeps.
 *
 * \return zero on success, -1 on error.
 */
int
voicebus_stop(struct voicebus *vb)
{
	if (vb_is_stopped(vb))
		return 0;

	INIT_COMPLETION(vb->stopped_completion);
	set_bit(STOP, &vb->flags);
	vb_clear_start_transmit_bit(vb);
	vb_clear_start_receive_bit(vb);
	if (vb_wait_for_completion_timeout(&vb->stopped_completion, HZ)) {
		BUG_ON(!vb_is_stopped(vb));
	} else {
		dev_warn(&vb->pdev->dev, "Timeout while waiting for board to "
			 "stop.\n");

		vb_clear_start_transmit_bit(vb);
		vb_clear_start_receive_bit(vb);
		set_bit(STOPPED, &vb->flags);
		vb_disable_interrupts(vb);
	}

#if VOICEBUS_DEFERRED == TIMER
	del_timer_sync(&vb->timer);
#endif

	return 0;
}
EXPORT_SYMBOL(voicebus_stop);

#ifdef CONFIG_VOICEBUS_SYSFS
static ssize_t
voicebus_current_latency_show(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	unsigned long flags;
	struct voicebus *vb = dev_get_drvdata(dev);
	unsigned int current_latency;
	spin_lock_irqsave(&vb->lock, flags);
	current_latency = vb->min_tx_buffer_count;
	spin_unlock_irqrestore(&vb->lock, flags);
	return sprintf(buf, "%d\n", current_latency);
}

DEVICE_ATTR(voicebus_current_latency, 0444,
	    voicebus_current_latency_show, NULL);
#endif

/*!
 * \brief Prepare the interface for module unload.
 *
 * Stop the interface and free all the resources allocated by the driver.  The
 * caller should have returned all VoiceBus buffers to the VoiceBus layer
 * before calling this function.
 *
 * context: !in_interrupt()
 */
void
voicebus_release(struct voicebus *vb)
{
#ifdef CONFIG_VOICEBUS_SYSFS
	device_remove_file(&vb->pdev->dev, &dev_attr_voicebus_current_latency);
#endif

	/* quiesce the hardware */
	voicebus_stop(vb);
#if VOICEBUS_DEFERRED == WORKQUEUE
	destroy_workqueue(vb->workqueue);
#elif VOICEBUS_DEFERRED == TASKLET
	tasklet_kill(&vb->tasklet);
#endif
	vb_reset_interface(vb);
#if VOICEBUS_DEFERRED != TIMER
	free_irq(vb->pdev->irq, vb);
#endif

	/* Cleanup memory and software resources. */
	vb_free_descriptors(vb, &vb->txd);
	vb_free_descriptors(vb, &vb->rxd);
	if (vb->idle_vbb_dma_addr) {
		dma_free_coherent(&vb->pdev->dev, vb->framesize,
				  vb->idle_vbb, vb->idle_vbb_dma_addr);
	}
	kmem_cache_destroy(vb->buffer_cache);
	release_region(vb->iobase, 0xff);
	pci_disable_device(vb->pdev);
	kfree(vb);
}
EXPORT_SYMBOL(voicebus_release);

static void
vb_increase_latency(struct voicebus *vb, unsigned int increase)
{
	void *vbb;
	int i;

	if (0 == increase)
		return;

	if (test_bit(LATENCY_LOCKED, &vb->flags))
		return;

	if (unlikely(increase > VOICEBUS_MAXLATENCY_BUMP))
		increase = VOICEBUS_MAXLATENCY_BUMP;

	if ((increase + vb->min_tx_buffer_count) > vb->max_latency)
		increase = vb->max_latency - vb->min_tx_buffer_count;

	/* Because there are 2 buffers in the transmit FIFO on the hardware,
	 * setting 3 ms of latency means that the host needs to be able to
	 * service the cards within 1ms.  This is because the interface will
	 * load up 2 buffers into the TX FIFO then attempt to read the 3rd
	 * descriptor.  If the OWN bit isn't set, then the hardware will set the
	 * TX descriptor not available interrupt. */

	/* Set the minimum latency in case we're restarted...we don't want to
	 * wait for the buffer to grow to this depth again in that case. */
	for (i = 0; i < increase; ++i) {
		vbb = voicebus_alloc(vb);
		WARN_ON(NULL == vbb);
		if (likely(NULL != vbb))
			handle_transmit(vb, vbb);
	}

	/* Set the new latency (but we want to ensure that there aren't any
	 * printks to the console, so we don't call the function) */
	spin_lock(&vb->lock);
	vb->min_tx_buffer_count += increase;
	spin_unlock(&vb->lock);
}

static void vb_set_all_owned(struct voicebus *vb,
			     struct voicebus_descriptor_list *dl)
{
	int i;
	struct voicebus_descriptor *d;

	for (i = 0; i < DRING_SIZE; ++i) {
		d = vb_descriptor(dl, i);
		SET_OWNED(d);
	}
}

static inline void vb_set_all_tx_owned(struct voicebus *vb)
{
	vb_set_all_owned(vb, &vb->txd);
}

/**
 * __vb_get_default_behind_count() - Returns how many idle buffers are loaded in tx fifo.
 *
 * These buffers are going to be set, but the AN983 does not clear the owned
 * bit on the descriptors until they've actually been sent around.
 *
 * If you do not check for both the current and next descriptors, you could have
 * a condition where idle buffers are being sent around, but we don't detect
 * them because our current descriptor always points to a non-idle buffer.
 */
static unsigned int __vb_get_default_behind_count(const struct voicebus *vb)
{
	const struct voicebus_descriptor_list *const dl = &vb->txd;
	const struct voicebus_descriptor *current_descriptor;
	const struct voicebus_descriptor *next_descriptor;

	current_descriptor = vb_descriptor(dl, dl->head);
	if (current_descriptor->buffer1 == vb->idle_vbb_dma_addr)
		return 2;

	next_descriptor = vb_descriptor(dl, ((dl->head + 1) & DRING_MASK));
	if (next_descriptor->buffer1 == vb->idle_vbb_dma_addr)
		return 1;

	return 0;
}

/**
 * vb_check_softunderrun() - Return true if the TX FIFO has underrun valid data.
 *
 * Returns true if we're processing the idle buffers, which means that the host
 * was not able to keep up with the hardware.  This differs from a normal
 * underrun in that the interface chip on the board still has descriptors to
 * transmit (just in this case, they are the idle buffers).
 *
 */
static inline int vb_is_softunderrun(const struct voicebus *vb)
{
	return __vb_get_default_behind_count(vb) > 0;
}

/**
 * vb_recover_tx_descriptor_list() - Recovers descriptor list
 *
 * Returns the number of descriptors that we're behind.
 *
 * Called if the [head | head + 1] pointer points to one of the idle buffers.
 * This means that the host computer has failed to keep far enough ahead of the
 * voicebus card.  This function acks the completed idle descriptors and gets
 * everything setup for normal operation again.
 *
 */
static unsigned int vb_recover_tx_descriptor_list(struct voicebus *vb)
{
	struct voicebus_descriptor_list *const dl = &vb->txd;
	struct voicebus_descriptor *d;
	struct vbb *vbb = NULL;
	unsigned int behind = __vb_get_default_behind_count(vb);
	WARN_ON(0 == behind);

	d = vb_descriptor(dl, dl->head);

	/* If we're only behind by one descriptor, we need to wait for all
	 * descriptors to go owned so we're starting with a fresh slate. This
	 * will also means we send an additional idle buffer. */
	if (1 == behind) {
		unsigned long stop;
		stop = jiffies + HZ/100;
		while (OWNED(d) && time_after(stop, jiffies))
			continue;
		WARN_ON(time_before(stop, jiffies));
		WARN_ON(d->buffer1 == vb->idle_vbb_dma_addr);
		WARN_ON(!dl->pending[dl->head]);
		dma_unmap_single(&vb->pdev->dev, d->buffer1,
				 vb->framesize, DMA_TO_DEVICE);
		vbb = dl->pending[dl->head];
		atomic_dec(&dl->count);
		--behind;
	}

	/* First complete any "idle" buffers that the hardware was to actually
	 * complete.  We've already preloaded the behind variable for the idle
	 * buffers that are in progress but may not be complete. */
	while (!OWNED(d)) {
		d->buffer1 = vb->idle_vbb_dma_addr;
		dl->pending[dl->head] = vb->idle_vbb;
		SET_OWNED(d);
		dl->head = ++dl->head & DRING_MASK;
		d = vb_descriptor(dl, dl->head);
		++behind;
	}

	/* Next get a little further ahead, because the hardware will be
	 * currently working on one of the idle buffers that we can't detect is
	 * completed yet in the previous block. Set the head and tail pointers
	 * to this new position so that everything can pick up normally. */
	dl->tail = dl->head = (dl->head + 10) & DRING_MASK;

	if (NULL != vbb)
		handle_transmit(vb, vbb);

	return behind;
}

/**
 * vb_deferred() - Manage the transmit and receive descriptor rings.
 *
 */
static void vb_deferred(struct voicebus *vb)
{
	unsigned int buffer_count;
	unsigned int i;
	unsigned int idle_buffers;
	int softunderrun;

	int underrun = test_bit(TX_UNDERRUN, &vb->flags);

	buffer_count = 0;

	/* First, temporarily store any non-idle buffers that the hardware has
	 * indicated it's finished transmitting.  Non idle buffers are those
	 * buffers that contain actual data and was filled out by the client
	 * driver (as of this writing, the wcte12xp or wctdm24xxp drivers) when
	 * passed up through the handle_transmit callback.
	 *
	 * On the other hand, idle buffers are "dummy" buffers that solely exist
	 * to in order to prevent the transmit descriptor ring from ever
	 * completely draining. */
	while ((vb->vbb_stash[buffer_count] = vb_get_completed_txb(vb))) {
		++buffer_count;
		if (unlikely(VOICEBUS_DEFAULT_MAXLATENCY < buffer_count)) {
			dev_warn(&vb->pdev->dev, "Critical problem detected "
				 "in transmit ring descriptor\n");
			if (buffer_count >= DRING_SIZE)
				buffer_count = DRING_SIZE - 1;
			break;
		}
	}

	vb->count += buffer_count;

	/* Next, check to see if we're in a softunderrun condition.
	 *
	 * A soft under is when the hardware has possibly copied at least one of
	 * the idle buffers into it's transmit queue.  Since all the buffers are
	 * nearly always owned by the hardware, the way we detect this is by
	 * checking if either of the next two buffers that we expect to complete
	 * are idle buffers.  We need to check the next two, because the
	 * hardware can read two buffers into it's tx fifo where they are
	 * definitely going to be sent on the interface, but the hardware does
	 * not flip the OWN bit on the descriptors until after the buffer has
	 * finished being sent to the CPLD.  Therefore, just looking at the own
	 * bit and the buffer pointed to by the dl->head is not enough.
	 *
	 * NOTE: Do not print anything to the console from the time a soft
	 * underrun is detected until the transmit descriptors are fixed up
	 * again. Otherwise the hardware could advance past where you set the
	 * head and tail pointers and then eventually run into the desciptor
	 * that it was currently working on when the softunderrun condition was
	 * first hit. */
	if (unlikely(vb_is_softunderrun(vb))) {
		softunderrun = 1;
		/* If we experienced a soft underrun, the recover function will
		 * a) ensure that all non-idle buffers have been completely
		 * tranmitted by the hardware b) Reset any idle buffers that
		 * have been completely transmitted c) Reset the head and tail
		 * pointers to somwhere in advance of where the hardware is
		 * currently processing and d) return how many idle_buffers were
		 * transmitted before our interrupt handler was called. */
		idle_buffers = vb_recover_tx_descriptor_list(vb);
		vb_increase_latency(vb, idle_buffers);
	} else {
		softunderrun = 0;
		idle_buffers = 0;
	}

	/* Now we can process the completed non-idle buffers since we know at
	 * this point that the transmit descriptor is in a "good" state. */
	for (i = 0; i < buffer_count; ++i)
		handle_transmit(vb, vb->vbb_stash[i]);

	/* If underrun is set, it means that the hardware signalled that it
	 * completely ran out of transmit descriptors.  This is what we are
	 * trying to avoid with all this racy softunderun business, but alas,
	 * it's still possible to happen if interrupts are locked longer than
	 * DRING_SIZE milliseconds for some reason. We should have already fixed
	 * up the descriptor ring in this case, so let's just tell the hardware
	 * to reread what it believes the next descriptor is. */
	if (unlikely(underrun)) {
		if (printk_ratelimit()) {
			dev_info(&vb->pdev->dev, "Host failed to service "
				 "card interrupt within %d ms which is a "
				 "hardunderun.\n", DRING_SIZE);
		}
		vb_rx_demand_poll(vb);
		vb_tx_demand_poll(vb);
		clear_bit(TX_UNDERRUN, &vb->flags);
	}

	/* Print any messages about soft latency bumps after we fix the transmit
	 * descriptor ring. Otherwise it's possible to take so much time
	 * printing the dmesg output that we lose the lead that we got on the
	 * hardware, resulting in a hard underrun condition. */
	if (unlikely(softunderrun &&
	    !test_bit(LATENCY_LOCKED, &vb->flags) && printk_ratelimit())) {
		if (vb->max_latency != vb->min_tx_buffer_count) {
			dev_info(&vb->pdev->dev, "Missed interrupt. "
				 "Increasing latency to %d ms in order to "
				 "compensate.\n", vb->min_tx_buffer_count);
		} else {
			dev_info(&vb->pdev->dev, "ERROR: Unable to service "
				 "card within %d ms and unable to further "
				 "increase latency.\n", vb->max_latency);
		}
	}

	/* And finally, pass up any receive buffers.  We also use vb->count to
	 * make a half-hearted attempt to not pass any recieved idle buffers to
	 * the caller, but this needs more work.... */
	while ((vb->vbb_stash[0] = vb_get_completed_rxb(vb))) {
		if (vb->count) {
			vb->handle_receive(vb->vbb_stash[0], vb->context);
			--vb->count;
		}
		vb_submit_rxb(vb, vb->vbb_stash[0]);
	}
}

/*!
 * \brief Interrupt handler for VoiceBus interface.
 *
 * NOTE: This handler is optimized for the case where only a single interrupt
 * condition will be generated at a time.
 *
 * ALSO NOTE:  Only access the interrupt status register from this function
 * since it doesn't employ any locking on the voicebus interface.
 */
static irqreturn_t
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
vb_isr(int irq, void *dev_id, struct pt_regs *regs)
#else
vb_isr(int irq, void *dev_id)
#endif
{
	struct voicebus *vb = dev_id;
	u32 int_status;

	int_status = __vb_getctl(vb, SR_CSR5);
	/* Mask out the reserved bits. */
	int_status &= ~(0xfc004010);
	int_status &= 0x7fff;

	if (!int_status)
		return IRQ_NONE;

	if (likely(int_status & TX_COMPLETE_INTERRUPT)) {
		/* ******************************************************** */
		/* NORMAL INTERRUPT CASE				    */
		/* ******************************************************** */
#		if VOICEBUS_DEFERRED == WORKQUEUE
		queue_work(vb->workqueue, &vb->workitem);
#		elif VOICEBUS_DEFERRED == TASKLET
		tasklet_schedule(&vb->tasklet);
#		else
		vb_deferred(vb);
#		endif
		__vb_setctl(vb, SR_CSR5, TX_COMPLETE_INTERRUPT);
	} else {
		/* ******************************************************** */
		/* ABNORMAL / ERROR CONDITIONS 				    */
		/* ******************************************************** */
		if ((int_status & TX_UNAVAILABLE_INTERRUPT)) {
			/* This can happen if the host fails to service the
			 * interrupt within the required time interval (1ms
			 * for each buffer on the queue).  Increasing the
			 * depth of the tx queue (up to a maximum of
			 * DRING_SIZE) can make the driver / system more
			 * tolerant of interrupt latency under periods of
			 * heavy system load, but also increases the general
			 * latency that the driver adds to the voice
			 * conversations.
			 */
			set_bit(TX_UNDERRUN, &vb->flags);
#			if VOICEBUS_DEFERRED == WORKQUEUE
			queue_work(vb->workqueue, &vb->workitem);
#			elif VOICEBUS_DEFERRED == TASKLET
			tasklet_schedule(&vb->tasklet);
#			else
			vb_deferred(vb);
#			endif
		}

		if (int_status & FATAL_BUS_ERROR_INTERRUPT)
			dev_err(&vb->pdev->dev, "Fatal Bus Error detected!\n");

		if (int_status & TX_STOPPED_INTERRUPT) {
			BUG_ON(!test_bit(STOP, &vb->flags));
			__vb_disable_interrupts(vb);
			complete(&vb->stopped_completion);
		}
		if (int_status & RX_STOPPED_INTERRUPT) {
			BUG_ON(!test_bit(STOP, &vb->flags));
			if (vb_is_stopped(vb)) {
				__vb_disable_interrupts(vb);
				complete(&vb->stopped_completion);
			}
		}

		/* Clear the interrupt(s) */
		__vb_setctl(vb, SR_CSR5, int_status);
	}

	return IRQ_HANDLED;
}

#if VOICEBUS_DEFERRED == TIMER
/*! \brief Called if the deferred processing is to happen in the context of
 * the timer.
 */
static void
vb_timer(unsigned long data)
{
	unsigned long start = jiffies;
	struct voicebus *vb = (struct voicebus *)data;
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
	vb_isr(0, vb, 0);
#else
	vb_isr(0, vb);
#endif
	if (!test_bit(STOPPED, &vb->flags)) {
		vb->timer.expires = start + HZ/1000;
		add_timer(&vb->timer);
	}
}
#endif

#if VOICEBUS_DEFERRED == WORKQUEUE
static void
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
vb_workfunc(void *data)
{
	struct voicebus *vb = data;
#else
vb_workfunc(struct work_struct *work)
{
	struct voicebus *vb = container_of(work, struct voicebus, workitem);
#endif
	vb_deferred(vb);
}
#elif VOICEBUS_DEFERRED == TASKLET
static void
vb_tasklet(unsigned long data)
{
	struct voicebus *vb = (struct voicebus *)data;
	vb_deferred(vb);
}
#endif /* #if VOICEBUS_DEFERRED == WORKQUEUE */

/*!
 * \brief Initalize the voicebus interface.
 *
 * This function must be called in process context since it may sleep.
 * \todo Complete this description.
 */
int
voicebus_init(struct pci_dev *pdev, u32 framesize, const char *board_name,
		  void (*handle_receive)(void *vbb, void *context),
		  void (*handle_transmit)(void *vbb, void *context),
		  void *context,
		  u32 debuglevel,
		  struct voicebus **vbp
		  )
{
	int retval = 0;
	struct voicebus *vb;

	BUG_ON(NULL == pdev);
	BUG_ON(NULL == board_name);
	BUG_ON(0 == framesize);
	BUG_ON(NULL == handle_receive);
	BUG_ON(NULL == handle_transmit);

	/* ----------------------------------------------------------------
	   Initialize the pure software constructs.
	   ---------------------------------------------------------------- */
	*vbp = NULL;
	vb = kmalloc(sizeof(*vb), GFP_KERNEL);
	if (NULL == vb) {
		dev_dbg(&vb->pdev->dev, "Failed to allocate memory for "
			"voicebus interface.\n");
		retval = -ENOMEM;
		goto cleanup;
	}
	memset(vb, 0, sizeof(*vb));
	vb->pdev = pdev;
	pci_set_drvdata(pdev, vb);
	voicebus_setdebuglevel(vb, debuglevel);
	vb->max_latency = VOICEBUS_DEFAULT_MAXLATENCY;

	spin_lock_init(&vb->lock);
	init_completion(&vb->stopped_completion);
	set_bit(STOP, &vb->flags);
	clear_bit(IN_DEFERRED_PROCESSING, &vb->flags);
	vb->framesize = framesize;
	vb->min_tx_buffer_count = VOICEBUS_DEFAULT_LATENCY;

#if VOICEBUS_DEFERRED == WORKQUEUE
	/* NOTE: This workqueue must be single threaded because locking is not
	 * used when buffers are removed or added to the descriptor list, and
	 * there should only be one producer / consumer (the hardware or the
	 * deferred processing function). */
	vb->workqueue = create_singlethread_workqueue(board_name);
#	if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 20)
	INIT_WORK(&vb->workitem, vb_workfunc, vb);
#	else
	INIT_WORK(&vb->workitem, vb_workfunc);
#	endif
#	if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 18)
	vb_set_workqueue_priority(vb);
#	endif
#elif VOICEBUS_DEFERRED == TASKLET
	tasklet_init(&vb->tasklet, vb_tasklet, (unsigned long)vb);
#elif VOICEBUS_DEFERRED == TIMER
	init_timer(&vb->timer);
	vb->timer.function = vb_timer;
	vb->timer.data = (unsigned long)vb;
#endif

	vb->handle_receive = handle_receive;
	vb->handle_transmit = handle_transmit;
	vb->context = context;

	/* \todo This cache should be shared by all instances supported by
	 * this driver. */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 23)
	vb->buffer_cache = kmem_cache_create(board_name, vb->framesize, 0,
#if defined(CONFIG_SLUB) && (LINUX_VERSION_CODE == KERNEL_VERSION(2, 6, 22))
				SLAB_HWCACHE_ALIGN | SLAB_STORE_USER, NULL, NULL);
#else
				SLAB_HWCACHE_ALIGN, NULL, NULL);
#endif
#else
#ifdef DEBUG
	vb->buffer_cache = kmem_cache_create(board_name, vb->framesize, 0,
				SLAB_HWCACHE_ALIGN | SLAB_STORE_USER |
				SLAB_POISON | SLAB_DEBUG_FREE, NULL);
#else
	vb->buffer_cache = kmem_cache_create(board_name, vb->framesize, 0,
				SLAB_HWCACHE_ALIGN, NULL);
#endif
#endif
	if (NULL == vb->buffer_cache) {
		dev_err(&vb->pdev->dev, "Failed to allocate buffer cache.\n");
		goto cleanup;
	}

#ifdef CONFIG_VOICEBUS_SYSFS
	dev_dbg(&vb->pdev->dev, "Creating sysfs attributes.\n");
	retval = device_create_file(&vb->pdev->dev,
				    &dev_attr_voicebus_current_latency);
	if (retval) {
		dev_dbg(&vb->pdev->dev,
			"Failed to create device attributes.\n");
		goto cleanup;
	}
#endif
	/* ----------------------------------------------------------------
	   Configure the hardware / kernel module interfaces.
	   ---------------------------------------------------------------- */
	if (pci_set_dma_mask(vb->pdev, DMA_BIT_MASK(32))) {
		dev_err(&vb->pdev->dev, "No suitable DMA available.\n");
		goto cleanup;
	}

	if (pci_read_config_byte(vb->pdev, 0x0c, &vb->cache_line_size)) {
		dev_err(&vb->pdev->dev, "Failed read of cache line "
			"size from PCI configuration space.\n");
		goto cleanup;
	}

	if (pci_enable_device(pdev)) {
		dev_err(&vb->pdev->dev, "Failed call to pci_enable_device.\n");
		retval = -EIO;
		goto cleanup;
	}

	/* \todo This driver should be modified to use the memory mapped I/O
	   as opposed to IO space for portability and performance. */
	if (0 == (pci_resource_flags(pdev, 0)&IORESOURCE_IO)) {
		dev_err(&vb->pdev->dev, "BAR0 is not IO Memory.\n");
		retval = -EIO;
		goto cleanup;
	}
	vb->iobase = pci_resource_start(pdev, 0);
	if (NULL == request_region(vb->iobase, 0xff, board_name)) {
		dev_err(&vb->pdev->dev, "IO Registers are in use by another "
			"module.\n");
		retval = -EIO;
		goto cleanup;
	}

	vb->idle_vbb = dma_alloc_coherent(&vb->pdev->dev, vb->framesize,
					  &vb->idle_vbb_dma_addr, GFP_KERNEL);

	retval = vb_initialize_tx_descriptors(vb);
	if (retval)
		goto cleanup;

	retval = vb_initialize_rx_descriptors(vb);
	if (retval)
		goto cleanup;

	/* ----------------------------------------------------------------
	   Configure the hardware interface.
	   ---------------------------------------------------------------- */
	if (pci_set_dma_mask(pdev, DMA_BIT_MASK(32))) {
		release_region(vb->iobase, 0xff);
		dev_warn(&vb->pdev->dev, "No suitable DMA available.\n");
		goto cleanup;
	}

	pci_set_master(pdev);
	vb_enable_io_access(vb);

#if VOICEBUS_DEFERRED != TIMER
	retval = request_irq(pdev->irq, vb_isr, DAHDI_IRQ_SHARED,
			     board_name, vb);
	if (retval) {
		dev_warn(&vb->pdev->dev, "Failed to request interrupt line.\n");
		goto cleanup;
	}
#endif

	*vbp = vb;
	return retval;
cleanup:
	if (NULL == vb)
		return retval;

#if VOICEBUS_DEFERRED == WORKQUEUE

	if (vb->workqueue)
		destroy_workqueue(vb->workqueue);

#elif VOICEBUS_DEFERRED == TASKLET
	tasklet_kill(&vb->tasklet);
#endif
	/* Cleanup memory and software resources. */

	if (vb->txd.desc)
		vb_free_descriptors(vb, &vb->txd);

	if (vb->rxd.desc)
		vb_free_descriptors(vb, &vb->rxd);

	dma_free_coherent(&vb->pdev->dev, vb->framesize,
			  vb->idle_vbb, vb->idle_vbb_dma_addr);

	if (vb->buffer_cache)
		kmem_cache_destroy(vb->buffer_cache);

	if (vb->iobase)
		release_region(vb->iobase, 0xff);

	if (vb->pdev)
		pci_disable_device(vb->pdev);

	kfree(vb);
	WARN_ON(0 == retval);
	return retval;
}
EXPORT_SYMBOL(voicebus_init);


/*! \brief Return the pci_dev in use by this voicebus interface. */
struct pci_dev *
voicebus_get_pci_dev(struct voicebus *vb)
{
	return vb->pdev;
}
EXPORT_SYMBOL(voicebus_get_pci_dev);

void *voicebus_pci_dev_to_context(struct pci_dev *pdev)
{
	return ((struct voicebus *)pci_get_drvdata(pdev))->context;
}
EXPORT_SYMBOL(voicebus_pci_dev_to_context);

static spinlock_t loader_list_lock;
static struct list_head binary_loader_list;

/**
 * vpmadtreg_loadfirmware - Load the vpmadt032 firmware.
 * @vb: The voicebus device to load.
 */
int vpmadtreg_loadfirmware(struct voicebus *vb)
{
	struct vpmadt_loader *loader;
	int ret = 0;
	int loader_present = 0;
	might_sleep();

	/* First check to see if a loader is already loaded into memory. */
	spin_lock(&loader_list_lock);
	loader_present = !(list_empty(&binary_loader_list));
	spin_unlock(&loader_list_lock);

	if (!loader_present) {
		ret = request_module("dahdi_vpmadt032_loader");
		if (ret)
			return ret;
	}

	spin_lock(&loader_list_lock);
	if (!list_empty(&binary_loader_list)) {
		loader = list_entry(binary_loader_list.next,
				struct vpmadt_loader, node);
		if (try_module_get(loader->owner)) {
			spin_unlock(&loader_list_lock);
			ret = loader->load(vb);
			module_put(loader->owner);
		} else {
			spin_unlock(&loader_list_lock);
			dev_info(&vb->pdev->dev, "Failed to find a "
				 "registered loader after loading module.\n");
			ret = -ENODEV;
		}
	} else {
		spin_unlock(&loader_list_lock);
		dev_info(&vb->pdev->dev, "Failed to find a registered "
			 "loader after loading module.\n");
		ret = -1;
	}
	return ret;
}

/* Called by the binary loader module when it is ready to start loading
 * firmware. */
int vpmadtreg_register(struct vpmadt_loader *loader)
{
	spin_lock(&loader_list_lock);
	list_add_tail(&loader->node, &binary_loader_list);
	spin_unlock(&loader_list_lock);
	return 0;
}
EXPORT_SYMBOL(vpmadtreg_register);

int vpmadtreg_unregister(struct vpmadt_loader *loader)
{
	int removed = 0;
	struct vpmadt_loader *cur, *temp;
	list_for_each_entry_safe(cur, temp, &binary_loader_list, node) {
		if (loader == cur) {
			list_del_init(&cur->node);
			removed = 1;
			break;
		}
	}
	WARN_ON(!removed);
	return 0;
}
EXPORT_SYMBOL(vpmadtreg_unregister);

int __init voicebus_module_init(void)
{
	int res;
	/* This registration with dahdi.ko will fail since the span is not
	 * defined, but it will make sure that this module is a dependency of
	 * dahdi.ko, so that when it is being unloded, this module will be
	 * unloaded as well.
	 */
	dahdi_register(0, 0);
	INIT_LIST_HEAD(&binary_loader_list);
	spin_lock_init(&loader_list_lock);
	res = vpmadt032_module_init();
	if (res)
		return res;
	return 0;
}

void __exit voicebus_module_cleanup(void)
{
	WARN_ON(!list_empty(&binary_loader_list));
}

MODULE_DESCRIPTION("Voicebus Interface w/VPMADT032 support");
MODULE_AUTHOR("Digium Incorporated <support@digium.com>");
MODULE_LICENSE("GPL");

module_init(voicebus_module_init);
module_exit(voicebus_module_cleanup);
