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
#include <dahdi/kernel.h>
#include <linux/cdev.h>
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

/*--------- Sysfs channel handling ----*/

#define chan_attr(field, format_string)				\
static BUS_ATTR_READER(field##_show, dev, buf)			\
{								\
	struct dahdi_chan	*chan;				\
								\
	chan = dev_to_chan(dev);				\
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

static BUS_ATTR_READER(sig_show, dev, buf)
{
	struct dahdi_chan	*chan;

	chan = dev_to_chan(dev);
	return sprintf(buf, "%s\n", sigstr(chan->sig));
}

static BUS_ATTR_READER(in_use_show, dev, buf)
{
	struct dahdi_chan	*chan;

	chan = dev_to_chan(dev);
	return sprintf(buf, "%d\n", test_bit(DAHDI_FLAGBIT_OPEN, &chan->flags));
}

static BUS_ATTR_READER(alarms_show, dev, buf)
{
	struct dahdi_chan	*chan;
	int			len;

	chan = dev_to_chan(dev);
	len = fill_alarm_string(buf, PAGE_SIZE, chan->chan_alarms);
	buf[len++] = '\n';
	return len;
}


static struct device_attribute chan_dev_attrs[] = {
	__ATTR_RO(name),
	__ATTR_RO(channo),
	__ATTR_RO(chanpos),
	__ATTR_RO(sig),
	__ATTR_RO(sigcap),
	__ATTR_RO(alarms),
	__ATTR_RO(blocksize),
#ifdef OPTIMIZE_CHANMUTE
	__ATTR_RO(chanmute),
#endif
	__ATTR_RO(in_use),
	__ATTR_NULL,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 13)
static struct class *chan_class;
#else
static struct class_simple *chan_class;
#define class_create class_simple_create
#define class_destroy class_simple_destroy
#endif


static void chan_release(struct device *dev)
{
	struct dahdi_chan	*chan;

	BUG_ON(!dev);
	chan = dev_to_chan(dev);
	chan_dbg(DEVICES, chan, "SYSFS\n");
}

static int chan_match(struct device *dev, struct device_driver *driver)
{
	struct dahdi_chan	*chan;

	chan = dev_to_chan(dev);
	chan_dbg(DEVICES, chan, "%s: SYSFS\n", __func__);
	return 1;
}

static struct bus_type chan_bus_type = {
	.name           = "dahdi_chans",
	.match          = chan_match,
	.dev_attrs	= chan_dev_attrs,
};

static int chan_probe(struct device *dev)
{
	struct dahdi_chan	*chan;

	chan = dev_to_chan(dev);
	return 0;
}

static int chan_remove(struct device *dev)
{
	struct dahdi_chan	*chan;

	chan = dev_to_chan(dev);
	return 0;
}

static struct device_driver	chan_driver = {
	.name = "generic_chan",
	.bus = &chan_bus_type,
#ifndef OLD_HOTPLUG_SUPPORT
	.owner = THIS_MODULE,
#endif
	.probe = chan_probe,
	.remove = chan_remove
};

int chan_sysfs_create(struct dahdi_chan *chan)
{
	struct device		*dev = &chan->chan_device;
	struct dahdi_span	*span;
	int			res;
	dev_t			devt;

	BUG_ON(!chan);
	span = chan->span;
	BUG_ON(!span);
	devt = MKDEV(MAJOR(dahdi_channels_devt), chan->channo);
	chan_dbg(DEVICES, chan, "SYSFS\n");
	dev = &chan->chan_device;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 18)
	dev->devt = devt;
	dev->class = NULL;
#endif
	dev->bus = &chan_bus_type;
	dev->parent = &span->span_device;
	/*
	 * WARNING: the name cannot be longer than KOBJ_NAME_LEN
	 */
	dev_set_name(dev, "dahdi!spans!%d!%d", span->spanno, chan->chanpos);
	dev_set_drvdata(dev, chan);
	dev->release = chan_release;
	res = device_register(dev);
	if (res) {
		chan_err(chan, "%s: device_register failed: %d\n",
				__func__, res);
		return res;
	}
	return 0;
}

void chan_sysfs_remove(struct dahdi_chan *chan)
{
	struct device	*dev = &chan->chan_device;

	chan_dbg(DEVICES, chan, "SYSFS\n");
	chan_dbg(DEVICES, chan, "Destroying channel %d\n", chan->channo);
	dev = &chan->chan_device;
	if (!dev_get_drvdata(dev))
		return;
	BUG_ON(dev_get_drvdata(dev) != chan);
	device_unregister(dev);
	dev_set_drvdata(dev, NULL);
	/* FIXME: should have been done earlier in dahdi_chan_unreg */
	chan->channo = -1;
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
	res = bus_register(&chan_bus_type);
	if (res != 0) {
		dahdi_err("%s: bus_register(%s) failed. Error number %d",
			__func__, chan_bus_type.name, res);
		goto failed_bus;
	}
	res = driver_register(&chan_driver);
	if (res < 0) {
		dahdi_err("%s: driver_register(%s) failed. Error number %d",
			__func__, chan_driver.name, res);
		goto failed_driver;
	}
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
		driver_unregister(&chan_driver);
		goto failed_chrdev_region;
	}
	cdev_init(&dahdi_channels_cdev, fops);
	res = cdev_add(&dahdi_channels_cdev, dahdi_channels_devt,
			DAHDI_MAX_CHANNELS);
	if (res) {
		dahdi_err("%s: cdev_add() failed (%d)", __func__, res);
		goto failed_cdev_add;
	}

	/* NULL parent, NULL drvdata */
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
	driver_unregister(&chan_driver);
failed_driver:
	bus_unregister(&chan_bus_type);
failed_bus:
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
	class_destroy(chan_class);
	driver_unregister(&chan_driver);
	bus_unregister(&chan_bus_type);
}
