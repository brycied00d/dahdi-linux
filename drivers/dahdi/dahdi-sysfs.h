#ifndef	DAHDI_SYSFS_H
#define	DAHDI_SYSFS_H

/*
 * Very old hotplug support
 */
#if LINUX_VERSION_CODE <= KERNEL_VERSION(2, 6, 9)
#define	OLD_HOTPLUG_SUPPORT	/* for older kernels */
#define	OLD_HOTPLUG_SUPPORT_269
#endif

#ifdef	OLD_HOTPLUG_SUPPORT_269
/* Copy from new kernels lib/kobject_uevent.c */
enum kobject_action {
	KOBJ_ADD,
	KOBJ_REMOVE,
	KOBJ_CHANGE,
	KOBJ_MOUNT,
	KOBJ_UMOUNT,
	KOBJ_OFFLINE,
	KOBJ_ONLINE,
};
#endif

/*
 * Hotplug replaced with uevent in 2.6.16
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 16)
#define	OLD_HOTPLUG_SUPPORT	/* for older kernels */
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 14)
#define	DEVICE_ATTR_READER(name, dev, buf)	\
		ssize_t name(struct device *dev, struct device_attribute *attr,\
				char *buf)
#define	DEVICE_ATTR_WRITER(name, dev, buf, count)	\
		ssize_t name(struct device *dev, struct device_attribute *attr,\
				const char *buf, size_t count)
#define BUS_ATTR_READER(name, dev, buf) \
   ssize_t name(struct device *dev, struct device_attribute *attr, char *buf)
#define BUS_ATTR_WRITER(name, dev, buf, count) \
   ssize_t name(struct device *dev, struct device_attribute *attr, \
		   const char *buf, size_t count)
#else
#define	DEVICE_ATTR_READER(name, dev, buf)	\
		ssize_t name(struct device *dev, char *buf)
#define	DEVICE_ATTR_WRITER(name, dev, buf, count)	\
		ssize_t name(struct device *dev, const char *buf, size_t count)
#define BUS_ATTR_READER(name, dev, buf) \
   ssize_t name(struct device *dev, char *buf)
#define BUS_ATTR_WRITER(name, dev, buf, count) \
   ssize_t name(struct device *dev, const char *buf, size_t count)
#endif

#define	DRIVER_ATTR_READER(name, drv, buf)	\
		ssize_t name(struct device_driver *drv, char * buf)

/* Global */
int __init dahdi_driver_init(const struct file_operations *fops);
void dahdi_driver_exit(void);
int __init dahdi_driver_chan_init(const struct file_operations *fops);
void dahdi_driver_chan_exit(void);

/* per-span */
int span_sysfs_create(struct dahdi_span *span);
void span_sysfs_remove(struct dahdi_span *span);

/* For use in dahdi-base only: */
int chan_sysfs_create(struct dahdi_chan *chan);
void chan_sysfs_remove(struct dahdi_chan *chan);

#endif	/* DAHDI_SYSFS_H */
