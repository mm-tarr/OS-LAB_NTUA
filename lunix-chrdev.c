/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * Marios Tatarakis <el21851@mail.ntua.gr>
 * Konstantinos Tolias <el20187@mail.ntua.gr>
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */

static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	
	WARN_ON ( !(sensor = state->sensor));
	
	// Compare the timestamp of the last cached data with the sensor's current timestamp
	if (sensor->msr_data[state->type]->last_update != state->buf_timestamp)
		return 1; // Need refresh
	
	/* The following return is bogus, just for the stub to compile */
	return 0; 
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	struct lunix_msr_data_struct *msr_data;
	uint32_t raw_val, timestamp;
	long value;
	unsigned long flags; // For interrupt-safe spinlock
	
	debug("entering\n");

	sensor = state->sensor;
	msr_data = sensor->msr_data[state->type];

	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 * We use irqsave because the writer (lunix-sensors.c) runs in interrupt context.
	 */
	spin_lock_irqsave(&sensor->lock, flags);
	raw_val = msr_data->values[0];
	timestamp = msr_data->last_update;
	spin_unlock_irqrestore(&sensor->lock, flags);

	/*
	 * Any new data available?
	 */
	if (timestamp == state->buf_timestamp)
		return -EAGAIN; // Data is old, try again later

	/*
	 * Now we can take our time to format them,
	 * holding only the private state semaphore
	 */
	switch (state->type) {
	case BATT:
		value = lookup_voltage[raw_val];
		break;
	case TEMP:
		value = lookup_temperature[raw_val];
		break;
	case LIGHT:
		value = lookup_light[raw_val];
		break;
	default:
		return -EINVAL;
	}

	// Format: "27.123\n"
	// The lookup tables return (value * 1000), so we split logic
	state->buf_lim = snprintf(state->buf_data, LUNIX_CHRDEV_BUFSZ, "%ld.%ld\n", value / 1000, value % 1000);
	state->buf_timestamp = timestamp;

	debug("leaving\n");
	return 0;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	struct lunix_chrdev_state_struct *state;
	unsigned int minor, sensor_idx, type;
	int ret;

	debug("entering\n");
	ret = -ENODEV;
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;

	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
	minor = iminor(inode);
	sensor_idx = minor >> 3; // Top bits are sensor ID
	type = minor & 0x7;      // Bottom 3 bits are measurement type

	if (sensor_idx >= lunix_sensor_cnt || type >= N_LUNIX_MSR)
		goto out;

	/* Allocate a new Lunix character device private state structure */
	state = kzalloc(sizeof(*state), GFP_KERNEL);
	if (!state) {
		ret = -ENOMEM;
		goto out;
	}

	state->type = type;
	state->sensor = &lunix_sensors[sensor_idx];
	state->buf_lim = 0;
	state->buf_timestamp = 0;
	sema_init(&state->lock, 1); // Initialize the semaphore as unlocked

	filp->private_data = state;
	ret = 0;
out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	kfree(filp->private_data);
	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return -EINVAL;
}

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret;
	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;

	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);

	/* Lock to protect our private state buffer */
	if (down_interruptible(&state->lock))
		return -ERESTARTSYS;

	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement), do so
	 */
	
	/* Auto-rewind on EOF mode */
	/* This allows tools like 'cat' to keep reading stream updates */
	if (*f_pos >= state->buf_lim)
		*f_pos = 0;

	if (*f_pos == 0) {
		while (lunix_chrdev_state_update(state) == -EAGAIN) {
			/* The process needs to sleep */
			/* Release the lock before sleeping! */
			up(&state->lock);

			// If non-blocking read is requested, return immediately
			if (filp->f_flags & O_NONBLOCK)
				return -EAGAIN;

			// Sleep until 'needs_refresh' is true
			if (wait_event_interruptible(sensor->wq, lunix_chrdev_state_needs_refresh(state)))
				return -ERESTARTSYS; // Woken by signal (Ctrl+C)
			
			// Re-acquire lock to try updating again
			if (down_interruptible(&state->lock))
				return -ERESTARTSYS;
		}
	}

	/* Determine the number of cached bytes to copy to userspace */
	if (cnt > state->buf_lim - *f_pos)
		cnt = state->buf_lim - *f_pos;

	if (copy_to_user(usrbuf, state->buf_data + *f_pos, cnt)) {
		ret = -EFAULT;
		goto out;
	}

	*f_pos += cnt;
	ret = cnt;

out:
	/* Unlock */
	up(&state->lock);
	return ret;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops = 
{
	.owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap
};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;
	
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);

	/* register_chrdev_region? */
	ret = register_chrdev_region(dev_no, lunix_minor_cnt, "lunix");
	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}
	
	/* cdev_add? */
	ret = cdev_add(&lunix_chrdev_cdev, dev_no, lunix_minor_cnt);
	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
