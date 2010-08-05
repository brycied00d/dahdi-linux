/*
 * Linux devices (SysFS) support in DAHDI spans
 *
 * Written by Tzafrir Cohen <tzafrir.cohen@xorcom.com>
 * Copyright (C) 2010, Xorcom
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


#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 0)
#  warning "This module is tested only with 2.6 kernels"
#endif


#define DAHDI_PRINK_MACROS_USE_debug
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/workqueue.h>
#include <linux/delay.h>	/* for msleep() to debug */
#include <dahdi/kernel.h>
#include "dahdi-sysfs.h"

static const char rcsid[] = "$Id$";

/* Command line parameters */
extern int debug;

static char *initdir = "/usr/share/dahdi";
module_param(initdir, charp, 0644);

static int span_match(struct device *dev, struct device_driver *driver)
{
	//dev_info(dev, "SYSFS MATCH: driver->name = %s\n", driver->name);
	return 1;
}

#ifdef OLD_HOTPLUG_SUPPORT
static int span_hotplug(struct device *dev, char **envp, int envnum,
		char *buff, int bufsize)
{
	struct dahdi_span *span;

	if (!dev)
		return -ENODEV;
	span = dev_to_span(dev);
	envp[0] = buff;
	if (snprintf(buff, bufsize, "SPAN_NAME=%s", span->name) >= bufsize)
		return -ENOMEM;
	envp[1] = NULL;
	return 0;
}
#else

#define	SPAN_VAR_BLOCK	\
	do {		\
		DAHDI_ADD_UEVENT_VAR("DAHDI_INIT_DIR=%s", initdir);	\
		DAHDI_ADD_UEVENT_VAR("SPAN_NUM=%d", span->spanno);	\
		DAHDI_ADD_UEVENT_VAR("SPAN_NAME=%s", span->name);	\
	} while (0)

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 24)
#define DAHDI_ADD_UEVENT_VAR(fmt, val...)			\
	do {							\
		int err = add_uevent_var(envp, num_envp, &i,	\
				buffer, buffer_size, &len,	\
				fmt, val);			\
		if (err)					\
			return err;				\
	} while (0)

static int span_uevent(struct device *dev, char **envp, int num_envp,
		char *buffer, int buffer_size)
{
	struct dahdi_span	*span;
	int			i = 0;
	int			len = 0;

	if (!dev)
		return -ENODEV;
	span = dev_to_span(dev);
	dahdi_dbg(GENERAL, "SYFS dev_name=%s span=%s\n",
			dev_name(dev), span->name);
	SPAN_VAR_BLOCK;
	envp[i] = NULL;
	return 0;
}

#else
#define DAHDI_ADD_UEVENT_VAR(fmt, val...)			\
	do {							\
		int err = add_uevent_var(kenv, fmt, val);	\
		if (err)					\
			return err;				\
	} while (0)

static int span_uevent(struct device *dev, struct kobj_uevent_env *kenv)
{
	struct dahdi_span *span;

	if (!dev)
		return -ENODEV;
	span = dev_to_span(dev);
	dahdi_dbg(GENERAL, "SYFS dev_name=%s span=%s\n",
			dev_name(dev), span->name);
	SPAN_VAR_BLOCK;
	return 0;
}

#endif

#endif	/* OLD_HOTPLUG_SUPPORT */

#define span_attr(field, format_string)				\
static BUS_ATTR_READER(field##_show, dev, buf)			\
{								\
	struct dahdi_span *span;				\
								\
	span = dev_to_span(dev);				\
	return sprintf(buf, format_string, span->field);	\
}

span_attr(name, "%s\n");
span_attr(desc, "%s\n");
span_attr(spantype, "%s\n");
span_attr(manufacturer, "%s\n");
span_attr(devicetype, "%s\n");
span_attr(location, "%s\n");
span_attr(hardware_id, "%s\n");
span_attr(span_id, "%02d\n");
span_attr(alarms, "0x%x\n");
span_attr(irq, "%d\n");
span_attr(irqmisses, "%d\n");
span_attr(lbo, "%d\n");
span_attr(syncsrc, "%d\n");

static BUS_ATTR_READER(is_digital_show, dev, buf)
{
	struct dahdi_span *span;
	int is_digital;

	span = dev_to_span(dev);
	is_digital = span->linecompat && (
		/* TODO: is this mask too liberal? */
		DAHDI_CONFIG_AMI | DAHDI_CONFIG_B8ZS | DAHDI_CONFIG_D4 |
		DAHDI_CONFIG_ESF |
		DAHDI_CONFIG_HDB3 | DAHDI_CONFIG_CCS | DAHDI_CONFIG_CRC4
	);
	return sprintf(buf, "%d\n", is_digital);
}

static BUS_ATTR_READER(is_sync_master_show, dev, buf)
{
	struct dahdi_span *span;

	span = dev_to_span(dev);
	return sprintf(buf, "%d\n", dahdi_is_sync_master(span));
}

static int configured_channels(struct dahdi_span *span)
{
	unsigned long flags;
	int ready = 0;
	int i;

	spin_lock_irqsave(&span->lock, flags);
	for (i = 0; i < span->channels; i++) {
		struct dahdi_chan *chan = span->chans[i];

		if (chan->sig == DAHDI_SIG_NONE)
			continue;
		if (chan->sig == __DAHDI_SIG_DACS && span->lineconfig == 0)
			continue;
		ready++;
	}
	spin_unlock_irqrestore(&span->lock, flags);
	return ready;
}

static BUS_ATTR_READER(configured_channels_show, dev, buf)
{
	struct dahdi_span *span;

	span = dev_to_span(dev);
	return sprintf(buf, "%d\t%d/%d\t%s\n",
		span->spanno,
		configured_channels(span),
		span->channels,
		span->name);
}

static BUS_ATTR_WRITER(user_ready_store, dev, buf, count)
{
	struct dahdi_span *span;

	span = dev_to_span(dev);
	span->user_ready++;
	span_dbg(DEVICES, span, "Got user_ready(%s)\n", buf);
	wake_up_interruptible(&span->wait_user);
	return count;
}

#define	SPAN_WAIT_TIMEOUT	(20*HZ)

static BUS_ATTR_READER(user_ready_show, dev, buf)
{
	struct dahdi_span *span;
	int ret;

	span = dev_to_span(dev);
	ret = wait_event_interruptible_timeout(
		span->wait_user,
		span->user_ready,
		SPAN_WAIT_TIMEOUT);
	if (ret == 0) {
		span_dbg(DEVICES, span, "Span wait user timeout\n");
		return -ETIMEDOUT;
	} else if (ret < 0) {
		span_dbg(DEVICES, span, "Span wait user interrupted\n");
		return -ERESTARTSYS;
	}
	return sprintf(buf, "%d\t%d/%d\t%s\n",
		span->spanno,
		configured_channels(span),
		span->channels,
		span->name);
}

static struct device_attribute span_dev_attrs[] = {
	__ATTR_RO(name),
	__ATTR_RO(desc),
	__ATTR_RO(spantype),
	__ATTR_RO(manufacturer),
	__ATTR_RO(devicetype),
	__ATTR_RO(location),
	__ATTR_RO(hardware_id),
	__ATTR_RO(span_id),
	__ATTR_RO(alarms),
	__ATTR_RO(irq),
	__ATTR_RO(irqmisses),
	__ATTR_RO(lbo),
	__ATTR_RO(syncsrc),
	__ATTR_RO(is_digital),
	__ATTR_RO(is_sync_master),
	__ATTR_RO(configured_channels),
	__ATTR(user_ready, S_IRUGO | S_IWUSR, user_ready_show,
			user_ready_store),
	__ATTR_NULL,
};


static struct driver_attribute dahdi_attrs[] = {
	__ATTR_NULL,
};

static struct bus_type spans_bus_type = {
	.name           = "dahdi_spans",
	.match          = span_match,
#ifdef OLD_HOTPLUG_SUPPORT
	.hotplug	= span_hotplug,
#else
	.uevent         = span_uevent,
#endif
	.dev_attrs	= span_dev_attrs,
	.drv_attrs	= dahdi_attrs,
};


static int span_probe(struct device *dev)
{
	struct dahdi_span *span;

	span = dev_to_span(dev);
	//dev_info(dev, "span %d\n", span->spanno);
	return 0;
}

static int span_remove(struct device *dev)
{
	struct dahdi_span *span;

	span = dev_to_span(dev);
	//dev_info(dev, "span %d\n", span->spanno);
	return 0;
}

static struct device_driver dahdi_driver = {
	.name		= "generic_lowlevel",
	.bus		= &spans_bus_type,
	.probe		= span_probe,
	.remove		= span_remove,
#ifndef OLD_HOTPLUG_SUPPORT
	.owner		= THIS_MODULE
#endif
};


static void span_uevent_send(struct dahdi_span *span, enum kobject_action act)
{
	struct kobject	*kobj;

	kobj = &span->span_device.kobj;
	span_dbg(DEVICES, span, "SYFS dev_name=%s action=%d\n",
		dev_name(&span->span_device), act);

#if defined(OLD_HOTPLUG_SUPPORT_269)
	{
		/* Copy from new kernels lib/kobject_uevent.c */
		static const char	*str[] = {
			[KOBJ_ADD]	"add",
			[KOBJ_REMOVE]	"remove",
			[KOBJ_CHANGE]	"change",
			[KOBJ_MOUNT]	"mount",
			[KOBJ_UMOUNT]	"umount",
			[KOBJ_OFFLINE]	"offline",
			[KOBJ_ONLINE]	"online"
		};
		kobject_hotplug(str[act], kobj);
	}
#elif defined(OLD_HOTPLUG_SUPPORT)
	kobject_hotplug(kobj, act);
#else
	kobject_uevent(kobj, act);
#endif
}

static void span_release(struct device *dev)
{
	dahdi_dbg(DEVICES, "%s: %s\n", __func__, dev_name(dev));
}

void span_sysfs_remove(struct dahdi_span *span)
{
	struct device	*span_device;
	int		x;

	BUG_ON(!span);
	span_dbg(DEVICES, span, "\n");
	span_device = &span->span_device;
	BUG_ON(!span_device);
	for (x = 0; x < span->channels; x++) {
		struct dahdi_chan *chan = span->chans[x];
		chan_sysfs_remove(chan);
	}
	if (!dev_get_drvdata(span_device))
		return;
	BUG_ON(dev_get_drvdata(span_device) != span);
	span_uevent_send(span, KOBJ_OFFLINE);
	device_unregister(&span->span_device);
}

int span_sysfs_create(struct dahdi_span *span)
{
	struct device	*span_device;
	int		res = 0;
	int		x;

	BUG_ON(!span);
	span_device = &span->span_device;
	BUG_ON(!span_device);
	span_dbg(DEVICES, span, "\n");

	span_device->bus = &spans_bus_type;
	span_device->parent = (span->parent)
		? span->parent
		: NULL;
	dev_set_name(span_device, "span-%d", span->spanno);
	dev_set_drvdata(span_device, span);
	span_device->release = span_release;
	res = device_register(span_device);
	if (res) {
		span_err(span, "%s: device_register failed: %d\n", __func__,
				res);
		goto err_device_register;
	}

	for (x = 0; x < span->channels; x++) {
		struct dahdi_chan *chan = span->chans[x];
		res = chan_sysfs_create(chan);
		if (res) {
			chan_err(chan, "Failed registering in sysfs: %d.\n",
					res);
			goto err_chan_device_register;
		}
	}
	span_uevent_send(span, KOBJ_ONLINE);
	return res;

err_chan_device_register:
	for (--x; x >= 0; x--) {
		struct dahdi_chan *chan = span->chans[x];
		chan_sysfs_remove(chan);
	}
	device_unregister(span_device);
err_device_register:
	dev_set_drvdata(span_device, NULL);
	span_device->parent = NULL;
	return res;
}

int __init dahdi_driver_init(const struct file_operations *fops)
{
	int	res;

	dahdi_dbg(DEVICES, "SYSFS\n");
	res = bus_register(&spans_bus_type);
	if (res != 0) {
		dahdi_err("%s: bus_register(%s) failed. Error number %d",
			__func__, spans_bus_type.name, res);
		goto failed_bus;
	}
	res = driver_register(&dahdi_driver);
	if (res < 0) {
		dahdi_err("%s: driver_register(%s) failed. Error number %d",
			__func__, dahdi_driver.name, res);
		goto failed_driver;
	}
	res = dahdi_driver_chan_init(fops);
	if (res < 0)
		goto failed_chan_bus;
	return 0;
failed_chan_bus:
	driver_unregister(&dahdi_driver);
failed_driver:
	bus_unregister(&spans_bus_type);
failed_bus:
	return res;
}

void dahdi_driver_exit(void)
{
	dahdi_dbg(DEVICES, "SYSFS\n");
	dahdi_driver_chan_exit();
	driver_unregister(&dahdi_driver);
	bus_unregister(&spans_bus_type);
}
