/*
 * Linux devices (SysFS) support in DAHDI channels
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
#include <linux/workqueue.h>
#include <linux/delay.h>	/* for msleep() to debug */
#include <linux/device.h>
#include <linux/cdev.h>

#include <dahdi/kernel.h>
#include "dahdi-sysfs.h"

static const char rcsid[] = "$Id$";

extern int debug;

static dev_t dahdi_channels_devt;
static struct cdev dahdi_channels_cdev;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
#define create_dev_file(class, devt, name, ...) \
	device_create(class, NULL, devt, NULL, name, ## __VA_ARGS__)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
#define create_dev_file(class, devt, name, ...) \
	device_create(class, NULL, devt, name, ## __VA_ARGS__)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 20)
#define create_dev_file(class, devt, name, ...) \
	device_create(class, NULL, devt, NULL, name, ## __VA_ARGS__)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 17)
#define create_dev_file(class, devt, name, ...) \
	class_device_create(class, NULL, devt, NULL, name, ## __VA_ARGS__)
#else
#define create_dev_file(class, devt, name, ...) \
	class_simple_device_add(class, devt, NULL, name, ## __VA_ARGS__)
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
#define destroy_dev_file(class, devt) \
	device_destroy(class, devt)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 13)
#define destroy_dev_file(class, devt) \
	class_device_destroy(class, devt)
#elif LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 9)
#define destroy_dev_file(class, devt) \
	class_simple_device_remove(devt)
#else
#define destroy_dev_file(class, devt) \
	class_simple_device_remove(class, devt)
#endif

static struct kset *dahdi_chan_kset;

/*--------- Sysfs channel handling ----*/

#define chan_attr(field, format_string)				\
static ATTR_READER(field##_show, kobj, buf)			\
{								\
	struct dahdi_chan	*chan;				\
								\
	chan = kobj_to_chan(kobj);				\
	return sprintf(buf, format_string, chan->field);	\
}

chan_attr(name, "%s\n");
chan_attr(channo, "%d\n");
chan_attr(chanpos, "%d\n");
chan_attr(sigcap, "0x%x\n");
chan_attr(blocksize, "%d\n");
#ifdef OPTIMIZE_CHANMUTE
chan_attr(chanmute, "%d\n");
#endif

static ATTR_READER(sig_show, kobj, buf)
{
	struct dahdi_chan	*chan;

	chan = kobj_to_chan(dev);
	return sprintf(buf, "%s\n", sigstr(chan->sig));
}

static ATTR_READER(in_use_show, kobj, buf)
{
	struct dahdi_chan	*chan;

	chan = kobj_to_chan(dev);
	return sprintf(buf, "%d\n", test_bit(DAHDI_FLAGBIT_OPEN, &chan->flags));
}

static ATTR_READER(alarms_show, kobj, buf)
{
	struct dahdi_chan	*chan;
	int			len;

	chan = kobj_to_chan(dev);
	len = fill_alarm_string(buf, PAGE_SIZE, chan->chan_alarms);
	buf[len++] = '\n';
	return len;
}

static ATTR_READER(dev_show, kobj, buf)
{
	struct dahdi_chan *chan = kobj_to_chan(kobj);
	return snprintf(buf, PAGE_SIZE, "%d:%d\n",
			MAJOR(chan->devt), MINOR(chan->devt));
}

DECLARE_ATTR_RO(name);
DECLARE_ATTR_RO(channo);
DECLARE_ATTR_RO(chanpos);
DECLARE_ATTR_RO(sig);
DECLARE_ATTR_RO(sigcap);
DECLARE_ATTR_RO(alarms);
DECLARE_ATTR_RO(blocksize);
#ifdef OPTIMIZE_CHANMUTE
DECLARE_ATTR_RO(chanmute);
#endif
DECLARE_ATTR_RO(in_use);
DECLARE_ATTR_RO(dev);

static struct attribute *chan_attrs[] = {
	__ATTR_PTR(name),
	__ATTR_PTR(channo),
	__ATTR_PTR(chanpos),
	__ATTR_PTR(sig),
	__ATTR_PTR(sigcap),
	__ATTR_PTR(alarms),
	__ATTR_PTR(blocksize),
#ifdef OPTIMIZE_CHANMUTE
	__ATTR_PTR(chanmute),
#endif
	__ATTR_PTR(in_use),
	__ATTR_PTR(dev),
	NULL
};

static struct attribute_group chan_attrs_group = {
	.attrs = chan_attrs,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 13)
struct class *chan_class;
#else
struct class_simple *chan_class;
#define class_create class_simple_create
#define class_destroy class_simple_destroy
#endif

static void chan_release(struct kobject *kobj)
{
	struct dahdi_chan	*chan;

	BUG_ON(!kobj);
	chan = kobj_to_chan(kobj);
        sysfs_remove_group(&chan->kobj, &chan_attrs_group);
	chan_dbg(DEVICES, chan, "SYSFS\n");
}

static struct kobj_type dahdi_chan_ktype = {
	.release = chan_release,
	.sysfs_ops = &dahdi_sysfs_ops,
	.default_attrs = chan_attrs,
};

int chan_sysfs_create(struct dahdi_chan *chan)
{
	struct dahdi_span	*span;
	int			res;

	BUG_ON(!chan);
	span = chan->span;
	BUG_ON(!span);
	chan->devt = MKDEV(MAJOR(dahdi_channels_devt), chan->channo);
	chan_dbg(DEVICES, chan, "SYSFS\n");

	/*
	 * WARNING: the name cannot be longer than KOBJ_NAME_LEN
	 */
	kobject_init(&chan->kobj, &dahdi_chan_ktype);
	chan->devt = MKDEV(DAHDI_MAJOR, chan->channo);
	chan->kobj.kset = dahdi_chan_kset;
	res = kobject_add(&chan->kobj, &span->kobj, "%d", chan->chanpos);
	if (res) {
		chan_err(chan, "%s: device_register failed: %d\n",
				__func__, res);
		return res;
	}

	dahdi_uevent_send(&chan->kobj, KOBJ_ADD);
	return res;
}

void chan_sysfs_remove(struct dahdi_chan *chan)
{
	chan_dbg(DEVICES, chan, "SYSFS\n");
	chan_dbg(DEVICES, chan, "Destroying channel %d\n", chan->channo);
	/* FIXME: should have been done earlier in dahdi_chan_unreg */
	dahdi_uevent_send(&chan->kobj, KOBJ_REMOVE);
	chan->channo = -1;
	kobject_put(&chan->kobj);
}

int dahdi_register_chardev(struct dahdi_chardev *dev)
{
	/* FIXME: Error handling */
	create_dev_file(chan_class, MKDEV(DAHDI_MAJOR, dev->minor),
			"dahdi!%s", dev->name);
	return 0;
}
EXPORT_SYMBOL(dahdi_register_chardev);

int dahdi_unregister_chardev(struct dahdi_chardev *dev)
{
	destroy_dev_file(chan_class, MKDEV(DAHDI_MAJOR, dev->minor));
	return 0;
}
EXPORT_SYMBOL(dahdi_unregister_chardev);

/*--------- Sysfs Device handling ----*/

int __init dahdi_driver_chan_init(const struct file_operations *fops)
{
	int	res;

	dahdi_dbg(DEVICES, "SYSFS\n");
	chan_class = class_create(THIS_MODULE, "dahdi");
	if (IS_ERR(chan_class)) {
		res = PTR_ERR(chan_class);
		dahdi_err("%s: class_create(dahi_chan) failed. Error: %d\n",
			__func__, res);
		goto failed_class;
	}
	res = alloc_chrdev_region(&dahdi_channels_devt, 0, DAHDI_MAX_CHANNELS,
			"dahdi-channels");
	if (res) {
		dahdi_err("%s: Failed allocating chrdev for %d channels (%d)",
			__func__, DAHDI_MAX_CHANNELS, res);
		goto failed_chrdev_region;
	}

	dahdi_chan_kset = kset_create_and_add("dahdi_channels", NULL, NULL);
	if (!dahdi_chan_kset) {
		res = -ENOMEM;
		goto failed_chrdev_region;
	}

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 17)
	cdev_init(&dahdi_channels_cdev, (struct file_operations *)fops);
#else
	cdev_init(&dahdi_channels_cdev, fops);
#endif
	res = cdev_add(&dahdi_channels_cdev, dahdi_channels_devt,
			DAHDI_MAX_CHANNELS);
	if (res) {
		dahdi_err("%s: cdev_add() failed (%d)", __func__, res);
		goto failed_cdev_add;
	}

	/* FIXME: error handling */
	create_dev_file(chan_class, MKDEV(DAHDI_MAJOR, DAHDI_TIMER),
			"dahdi!timer");
	create_dev_file(chan_class, MKDEV(DAHDI_MAJOR, DAHDI_CHANNEL),
			"dahdi!channel");
	create_dev_file(chan_class, MKDEV(DAHDI_MAJOR, DAHDI_PSEUDO),
			"dahdi!pseudo");
	create_dev_file(chan_class, MKDEV(DAHDI_MAJOR, DAHDI_CTL),
			"dahdi!ctl");
	return 0;

failed_cdev_add:
	unregister_chrdev_region(dahdi_channels_devt, DAHDI_MAX_CHANNELS);
failed_chrdev_region:
	class_destroy(chan_class);
failed_class:
	return res;
}

void dahdi_driver_chan_exit(void)
{
	dahdi_dbg(DEVICES, "SYSFS\n");
	destroy_dev_file(chan_class, MKDEV(DAHDI_MAJOR, DAHDI_CTL));
	destroy_dev_file(chan_class, MKDEV(DAHDI_MAJOR, DAHDI_PSEUDO));
	destroy_dev_file(chan_class, MKDEV(DAHDI_MAJOR, DAHDI_CHANNEL));
	destroy_dev_file(chan_class, MKDEV(DAHDI_MAJOR, DAHDI_TIMER));
	cdev_del(&dahdi_channels_cdev);
	unregister_chrdev_region(dahdi_channels_devt, DAHDI_MAX_CHANNELS);
	kset_unregister(dahdi_chan_kset);
	class_destroy(chan_class);
#if 0
	driver_unregister(&chan_driver);
	bus_unregister(&chan_bus_type);
#endif
}
