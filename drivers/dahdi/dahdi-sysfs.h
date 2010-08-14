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

struct dahdi_span_kobject {
	struct kobject	kobj;
	struct dahdi_span *span;
};
#define kobj_to_span(_kobj) \
	(container_of(_kobj, struct dahdi_span_kobject, kobj)->span)

extern int debug;
extern int default_ordering;

/*
 * Hotplug replaced with uevent in 2.6.16
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 16)
#define	OLD_HOTPLUG_SUPPORT	/* for older kernels */
#endif

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 14)
#define	ATTR_READER(name, kobj, buf)	\
		ssize_t name(struct kobject *kobj, struct kobj_attribute *attr,\
				char *buf)
#define	ATTR_WRITER(name, kobj, buf, count)	\
		ssize_t name(struct kobject *kobj, struct kobj_attribute *attr,\
				const char *buf, size_t count)
#else
#define	DEVICE_ATTR_READER(name, dev, buf)	\
		ssize_t name(struct device *dev, char *buf)
#define	DEVICE_ATTR_WRITER(name, dev, buf, count)	\
		ssize_t name(struct device *dev, const char *buf, size_t count)
#endif

#define	DRIVER_ATTR_READER(name, drv, buf)	\
		ssize_t name(struct device_driver *drv, char * buf)

#define DECLARE_ATTR_RO(_field) \
	static struct kobj_attribute attr_##_field  = __ATTR_RO(_field)

#define __ATTR_PTR(_field)\
	&attr_##_field.attr

/* Global */
int __init dahdi_driver_init(const struct file_operations *fops);
void dahdi_driver_exit(void);
int __init dahdi_driver_chan_init(const struct file_operations *fops);
void dahdi_driver_chan_exit(void);
ssize_t dahdi_attr_show(struct kobject *kobj, struct attribute *attr,
			char *buf);
ssize_t dahdi_attr_store(struct kobject *kobj, struct attribute *attr,
			 const char *buf, size_t count);

extern struct sysfs_ops dahdi_sysfs_ops;

/* per-span */
int span_sysfs_create(struct dahdi_span *span);
void span_sysfs_remove(struct dahdi_span *span);

/* For use in dahdi-base only: */
int chan_sysfs_create(struct dahdi_chan *chan, struct dahdi_span *span);
void chan_sysfs_remove(struct dahdi_chan *chan);

#endif	/* DAHDI_SYSFS_H */
