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
#include <linux/slab.h>
#include <linux/kobject.h>
#include <linux/device.h>

#include <dahdi/kernel.h>
#include "dahdi-sysfs.h"

static const char rcsid[] = "$Id$";

/* Command line parameters */
extern int debug;

static char *initdir = "/usr/share/dahdi";
module_param(initdir, charp, 0644);

#if 0
static int span_match(struct device *dev, struct device_driver *driver)
{
	//dev_info(dev, "SYSFS MATCH: driver->name = %s\n", driver->name);
	return 1;
}
#endif

#ifdef OLD_HOTPLUG_SUPPORT
static int span_hotplug(struct device *dev, char **envp, int envnum,
		char *buff, int bufsize)
{
	struct dahdi_span *span;

	if (!dev)
		return -ENODEV;
	span = kobj_to_span(dev);
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

#endif	/* OLD_HOTPLUG_SUPPORT */

#define span_attr(field, format_string)				\
static ATTR_READER(field##_show, kobj, buf)			\
{								\
	struct dahdi_span *span;				\
								\
	span = kobj_to_span(kobj);				\
	if (!span)						\
		return -ENODEV;					\
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

static ATTR_READER(is_digital_show, kobj, buf)
{
	struct dahdi_span *span;
	int is_digital;

	span = kobj_to_span(kobj);
	is_digital = span->linecompat && (
		/* TODO: is this mask too liberal? */
		DAHDI_CONFIG_AMI | DAHDI_CONFIG_B8ZS | DAHDI_CONFIG_D4 |
		DAHDI_CONFIG_ESF |
		DAHDI_CONFIG_HDB3 | DAHDI_CONFIG_CCS | DAHDI_CONFIG_CRC4
	);
	return sprintf(buf, "%d\n", is_digital);
}

static ATTR_READER(is_sync_master_show, kobj, buf)
{
	struct dahdi_span *span;

	span = kobj_to_span(kobj);
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

static ATTR_READER(configured_channels_show, kobj, buf)
{
	struct dahdi_span *span;

	span = kobj_to_span(kobj);
	return sprintf(buf, "%d\t%d/%d\t%s\n",
		span->spanno,
		configured_channels(span),
		span->channels,
		span->name);
}

#if 0
static ATTR_WRITER(user_ready_store, kobj, buf, count)
{
	struct dahdi_span *span;

	span = kobj_to_span(kobj);
	span->user_ready++;
	span_dbg(DEVICES, span, "Got user_ready(%s)\n", buf);
	wake_up_interruptible(&span->wait_user);
	return count;
}

#define	SPAN_WAIT_TIMEOUT	(20*HZ)

static ATTR_READER(user_ready_show, kobj, buf)
{
	struct dahdi_span *span;
	int ret;

	span = kobj_to_span(kobj);
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
#endif

DECLARE_ATTR_RO(name);
DECLARE_ATTR_RO(desc);
DECLARE_ATTR_RO(spantype);
DECLARE_ATTR_RO(manufacturer);
DECLARE_ATTR_RO(devicetype);
DECLARE_ATTR_RO(location);
DECLARE_ATTR_RO(hardware_id);
DECLARE_ATTR_RO(span_id);
DECLARE_ATTR_RO(alarms);
DECLARE_ATTR_RO(irq);
DECLARE_ATTR_RO(irqmisses);
DECLARE_ATTR_RO(lbo);
DECLARE_ATTR_RO(syncsrc);
DECLARE_ATTR_RO(is_digital);
DECLARE_ATTR_RO(is_sync_master);
DECLARE_ATTR_RO(configured_channels);

#if 0
	__ATTR(user_ready, S_IRUGO | S_IWUSR, user_ready_show,
			user_ready_store),
	__ATTR_NULL,
#endif


void dahdi_uevent_send(struct kobject *kobj, enum kobject_action act)
{
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

static void span_uevent_send(struct dahdi_span *span, enum kobject_action act)
{
	span_dbg(DEVICES, span, "SYFS dev_name=%s action=%d\n",
		kobject_name(&span->kobj->kobj), act);

	dahdi_uevent_send(&span->kobj->kobj, act);
}

static void span_kobj_release(struct kobject *kobj)
{
	struct dahdi_span_kobject *dkobj;
	dkobj = container_of(kobj, struct dahdi_span_kobject, kobj);
	WARN_ON(dkobj->span);
	kfree(dkobj);
}

ssize_t dahdi_attr_show(struct kobject *kobj, struct attribute *attr,
			char *buf)
{
        struct kobj_attribute *kattr;
        ssize_t ret = -EIO;

        kattr = container_of(attr, struct kobj_attribute, attr);
        if (kattr->show)
                ret = kattr->show(kobj, kattr, buf);
        return ret;
}

ssize_t dahdi_attr_store(struct kobject *kobj, struct attribute *attr,
				const char *buf, size_t count)
{
        struct kobj_attribute *kattr;
        ssize_t ret = -EIO;

        kattr = container_of(attr, struct kobj_attribute, attr);
        if (kattr->store)
                ret = kattr->store(kobj, kattr, buf, count);
        return ret;
}

struct sysfs_ops dahdi_sysfs_ops = {
        .show   = dahdi_attr_show,
        .store  = dahdi_attr_store,
};

static struct attribute *dahdi_span_attrs[] = {
	__ATTR_PTR(name),
	__ATTR_PTR(desc),
	__ATTR_PTR(spantype),
	__ATTR_PTR(manufacturer),
	__ATTR_PTR(devicetype),
	__ATTR_PTR(location),
	__ATTR_PTR(hardware_id),
	__ATTR_PTR(span_id),
	__ATTR_PTR(alarms),
	__ATTR_PTR(irq),
	__ATTR_PTR(irqmisses),
	__ATTR_PTR(lbo),
	__ATTR_PTR(syncsrc),
	__ATTR_PTR(is_digital),
	__ATTR_PTR(is_sync_master),
	__ATTR_PTR(configured_channels),
	NULL
};

static struct kobj_type dahdi_span_ktype = {
	.release = span_kobj_release,
	.sysfs_ops = &dahdi_sysfs_ops,
	.default_attrs = dahdi_span_attrs,
};

void span_sysfs_remove(struct dahdi_span *span)
{
	struct dahdi_span_kobject *dkobj;
	int		x;

	BUG_ON(!span);
	span_dbg(DEVICES, span, "\n");

	for (x = 0; x < span->channels; x++) {
		struct dahdi_chan *chan = span->chans[x];
		chan_sysfs_remove(chan);
	}

	span_uevent_send(span, KOBJ_REMOVE);
	dkobj = span->kobj;
	span->kobj = NULL;
	dkobj->span = NULL;
	kobject_put(&dkobj->kobj);
}

static struct kset *dahdi_spans_kset;

int span_sysfs_create(struct dahdi_span *span)
{
	int		res = 0;
	int		x;

	BUG_ON(!span);
	span_dbg(DEVICES, span, "\n");

	/* The board driver needs to set the parent before registering the
	 * device. */
	if (!span->parent)
		return -EINVAL;

	span->kobj = kzalloc(sizeof(*span->kobj), GFP_KERNEL);
	span->kobj->span = span;
	kobject_init(&span->kobj->kobj, &dahdi_span_ktype);

	res = kobject_add(&span->kobj->kobj, &span->parent->dev.kobj,
			  "%d", span->offset);
	if (res) {
		span_err(span, "%s: device_register failed: %d\n", __func__,
				res);
		goto err_device_register;
	}

	for (x = 0; x < span->channels; x++) {
		struct dahdi_chan *chan = span->chans[x];
		chan->_span = span;
		res = chan_sysfs_create(chan, span);
		if (res) {
			chan_err(chan, "Failed registering in sysfs: %d.\n",
					res);
			goto err_chan_device_register;
		}
	}
	span_uevent_send(span, KOBJ_ADD);
	return res;

err_chan_device_register:
	for (--x; x >= 0; x--) {
		struct dahdi_chan *chan = span->chans[x];
		chan_sysfs_remove(chan);
	}
	kobject_put(&span->kobj->kobj);
err_device_register:
	return res;
}

int __init dahdi_driver_init(const struct file_operations *fops)
{
	int	res;

	dahdi_dbg(DEVICES, "SYSFS\n");

	dahdi_dbg(DEVICES, "sizeof(struct dahdi_chan): %Zd\n", sizeof(struct dahdi_chan));
	dahdi_dbg(DEVICES, "sizeof(struct device): %Zd\n", sizeof(struct device));

#if 0
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
#endif
	res = dahdi_driver_chan_init(fops);
	if (res < 0)
		goto failed_chan_bus;

	dahdi_spans_kset = kset_create_and_add("spans", NULL, dahdi_class->dev_kobj);
	if (!dahdi_spans_kset) {
		res = -ENOMEM;
		goto failed_chan_bus;
	}

	return 0;
failed_chan_bus:
#if 0
	driver_unregister(&dahdi_driver);
failed_driver:
	bus_unregister(&spans_bus_type);
failed_bus:
#endif
	return res;
}

void dahdi_driver_exit(void)
{
	dahdi_dbg(DEVICES, "SYSFS\n");
	kset_unregister(dahdi_spans_kset);
	dahdi_driver_chan_exit();
#if 0
	driver_unregister(&dahdi_driver);
	bus_unregister(&spans_bus_type);
#endif
}
