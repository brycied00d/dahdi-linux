/*
 * DAHDI Telephony Interface to the Open Source Line Echo Canceller (OSLEC)
 *
 * Written by Tzafrir Cohen <tzafrir.cohen@xorcom.com>
 * Copyright (C) 2008 Xorcom, Inc.
 *
 * All rights reserved.
 *
 * Based on dahdi_echocan_hpec.c, Copyright (C) 2006-2008 Digium, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA. 
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ctype.h>
#include <linux/moduleparam.h>

/* Fix this if OSLEC is elsewhere */
#include "../staging/echo/oslec.h"
//#include <linux/oslec.h>
/* "provide" struct echo_can_state */
//#define oslec_state echo_can_state

#include <dahdi/kernel.h>

#define module_printk(level, fmt, args...) printk(level "%s: " fmt, THIS_MODULE->name, ## args)

static void echo_can_free(struct echo_can_state *ec)
{
	oslec_free((struct oslec_state *)ec);
}

static void echo_can_update(struct echo_can_state *ec, short *iref, short *isig)
{
	unsigned int SampleNum;

	for (SampleNum = 0; SampleNum < DAHDI_CHUNKSIZE; SampleNum++, iref++)
	{
		short iCleanSample;
		iCleanSample = (short) oslec_update((struct oslec_state *)ec, *iref, *isig);
		*isig++ = iCleanSample;
	}
}

static int echo_can_create(struct dahdi_echocanparams *ecp, struct dahdi_echocanparam *p,
			   struct echo_can_state **ec)
{
	if (ecp->param_count > 0) {
		printk(KERN_WARNING "OSLEC does not support parameters; failing request\n");
		return -EINVAL;
	}

	*ec = (struct echo_can_state *)oslec_create(ecp->tap_length, ECHO_CAN_USE_ADAPTION | ECHO_CAN_USE_NLP  | ECHO_CAN_USE_CLIP | ECHO_CAN_USE_TX_HPF | ECHO_CAN_USE_RX_HPF);

	return *ec ? 0 : -ENOTTY;
}

static inline int echo_can_traintap(struct echo_can_state *ec, int pos, short val)
{
	return 1;
}

static const struct dahdi_echocan me = {
	.name = "OSLEC",
	.owner = THIS_MODULE,
	.echo_can_create = echo_can_create,
	.echo_can_free = echo_can_free,
	.echo_can_array_update = echo_can_update,
	.echo_can_traintap = echo_can_traintap,
};

static int __init mod_init(void)
{
	if (dahdi_register_echocan(&me)) {
		module_printk(KERN_ERR, "could not register with DAHDI core\n");

		return -EPERM;
	}

	module_printk(KERN_INFO, "Registered echo canceler '%s'\n", me.name);

	return 0;
}

static void __exit mod_exit(void)
{
	dahdi_unregister_echocan(&me);
}

MODULE_DESCRIPTION("DAHDI OSLEC wrapper");
MODULE_AUTHOR("Tzafrir Cohen <tzafrir.cohen@xorcom.com>");
MODULE_LICENSE("GPL");

module_init(mod_init);
module_exit(mod_exit);
