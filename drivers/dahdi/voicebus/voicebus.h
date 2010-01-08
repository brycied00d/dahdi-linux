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


#ifndef __VOICEBUS_H__
#define __VOICEBUS_H__

#define VOICEBUS_DEFAULT_LATENCY	3
#define VOICEBUS_DEFAULT_MAXLATENCY	25
#define VOICEBUS_MAXLATENCY_BUMP	6

#define VOICEBUS_SFRAME_SIZE 1004

/*! The number of descriptors in both the tx and rx descriptor ring. */
#define DRING_SIZE	(1 << 7)  /* Must be a power of 2 */
#define DRING_MASK	(DRING_SIZE-1)

/* Define CONFIG_VOICEBUS_SYSFS to create some attributes under the pci device.
 * This is disabled by default because it hasn't been tested on the full range
 * of supported kernels. */
#undef CONFIG_VOICEBUS_SYSFS

#define INTERRUPT 0	/* Run the deferred processing in the ISR. */
#define TASKLET   1	/* Run in a tasklet. */
#define TIMER	  2	/* Run in a system timer. */
#define WORKQUEUE 3	/* Run in a workqueue. */

#ifndef VOICEBUS_DEFERRED
#define VOICEBUS_DEFERRED INTERRUPT
#endif

struct voicebus;

struct voicebus_operations {
	void (*handle_receive)(struct voicebus *vb, void *vbb);
	void (*handle_transmit)(struct voicebus *vb, void *vbb);
};

/**
 * struct voicebus_descriptor_list - A single descriptor list.
 */
struct voicebus_descriptor_list {
	struct voicebus_descriptor *desc;
	unsigned int 	head;
	unsigned int 	tail;
	void  		*pending[DRING_SIZE];
	dma_addr_t	desc_dma;
	atomic_t 	count;
	unsigned int	padding;
};

/**
 * struct voicebus - Represents physical interface to voicebus card.
 *
 */
struct voicebus {
	struct pci_dev		*pdev;
	spinlock_t		lock;
	struct voicebus_descriptor_list rxd;
	struct voicebus_descriptor_list txd;
	void			*idle_vbb;
	dma_addr_t		idle_vbb_dma_addr;
	const int		*debug;
	u32			iobase;
#if VOICEBUS_DEFERRED == WORKQUEUE
	struct workqueue_struct *workqueue;
	struct work_struct	workitem;
#elif VOICEBUS_DEFERRED == TASKLET
	struct tasklet_struct 	tasklet;
#elif VOICEBUS_DEFERRED == TIMER
	struct timer_list	timer;
#endif
	const struct voicebus_operations *ops;
	struct completion	stopped_completion;
	unsigned long		flags;
	unsigned int		min_tx_buffer_count;
	unsigned int		max_latency;
	void			*vbb_stash[DRING_SIZE];
	unsigned int		count;
};

int voicebus_init(struct voicebus *vb, const char *board_name);
void voicebus_release(struct voicebus *vb);
int voicebus_start(struct voicebus *vb);
int voicebus_stop(struct voicebus *vb);
void *voicebus_alloc(struct voicebus* vb);
void voicebus_free(struct voicebus *vb, void *vbb);
int voicebus_transmit(struct voicebus *vb, void *vbb);
int voicebus_set_minlatency(struct voicebus *vb, unsigned int milliseconds);
int voicebus_current_latency(struct voicebus *vb);
void voicebus_lock_latency(struct voicebus *vb);
void voicebus_unlock_latency(struct voicebus *vb);
int voicebus_is_latency_locked(const struct voicebus *vb);
 
#endif /* __VOICEBUS_H__ */
