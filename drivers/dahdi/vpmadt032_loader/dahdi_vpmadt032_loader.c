/*
 * DAHDI Telephony Interface to VPMADT032 Firmware Loader
 *
 * Copyright (C) 2008-2009 Digium, Inc. All rights reserved.
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
#include <linux/pci.h>

#include <dahdi/kernel.h>

static int debug;

#define module_printk(level, fmt, args...) \
	printk(level "%s: " fmt, THIS_MODULE->name, ## args)
#define debug_printk(level, fmt, args...) if (debug >= level) \
	printk(KERN_DEBUG "%s (%s): " fmt, THIS_MODULE->name, \
	__func__, ## args)

#include "voicebus/voicebus.h"
#include "voicebus/vpmadtreg.h"
#include "vpmadt032_loader.h"

vpmlinkage static int __attribute__((format (printf, 1, 2)))
logger(const char *format, ...)
{
	int res;
	va_list args;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 9)
	va_start(args, format);
	res = vprintk(format, args);
	va_end(args);
#else
	char buf[256];

	va_start(args, format);
	res = vsnprintf(buf, sizeof(buf), format, args);
	va_end(args);
	printk(KERN_INFO "%s" buf);
#endif

	return res;
}

vpmlinkage static void *memalloc(size_t len)
{
	return kmalloc(len, GFP_KERNEL);
}

vpmlinkage static void memfree(void *ptr)
{
	kfree(ptr);
}

struct private_context {
	struct voicebus *vb;
	void *old_rx;
	void *old_tx;
	void *old_context;
	void *pvt;
	struct completion done;
};

static void init_private_context(struct private_context *ctx)
{
	memset(ctx, 0, sizeof(ctx));
	init_completion(&ctx->done);
}

static void handle_receive(void *vbb, void *context)
{
	struct private_context *ctx = context;
	__vpmadt032_receive(ctx->pvt, vbb);
	if (__vpmadt032_done(ctx->pvt))
		complete(&ctx->done);
}

static void handle_transmit(void *vbb, void *context)
{
	struct private_context *ctx = context;
	__vpmadt032_transmit(ctx->pvt, vbb);
	voicebus_transmit(ctx->vb, vbb);
}

static int vpmadt032_load_firmware(struct voicebus *vb)
{
	int ret = 0;
	struct private_context *ctx;
	struct pci_dev *pdev = voicebus_get_pci_dev(vb);
	might_sleep();
	ctx = kmalloc(sizeof(struct private_context), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;
	init_private_context(ctx);
	ctx->vb = vb;
	ret = __vpmadt032_start_load(
			0, pdev->vendor << 16 | pdev->device,
			&ctx->pvt);
	if (ret)
		goto error_exit;
	voicebus_get_handlers(vb, &ctx->old_rx, &ctx->old_tx,
		&ctx->old_context);
	voicebus_set_handlers(vb, handle_receive, handle_transmit, ctx);
	wait_for_completion(&ctx->done);
	voicebus_set_handlers(vb, ctx->old_rx, ctx->old_tx, ctx->old_context);
	__vpmadt032_cleanup(ctx->pvt);
error_exit:
	kfree(ctx);
	return 0;
}

static struct vpmadt_loader loader = {
	.owner = THIS_MODULE,
	.load = vpmadt032_load_firmware,
};

static int __init vpmadt032_loader_init(void)
{
	__vpmadt032_init(logger, debug, memalloc, memfree);
	vpmadtreg_register(&loader);
	return 0;
}

static void __exit vpmadt032_loader_exit(void)
{
	vpmadtreg_unregister(&loader);
	return;
}

module_param(debug, int, S_IRUGO | S_IWUSR);
MODULE_DESCRIPTION("DAHDI VPMADT032 (Hardware Echo Canceller) Firmware Loader");
MODULE_AUTHOR("Digium Incorporated <support@digium.com>");
MODULE_LICENSE("Digium Commercial");

module_init(vpmadt032_loader_init);
module_exit(vpmadt032_loader_exit);
