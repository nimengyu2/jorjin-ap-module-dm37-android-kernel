#include <linux/init.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/ioctl.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <linux/poll.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <mach/gpio.h>

#define DEBUG
//#undef DEBUG

#ifdef DEBUG
#define hood_pr(fmt,arg...)	printk(fmt,##arg)
#else
#define hood_pr(fmt,arg...)	
#endif

#define DNAME "audio_switch"

#define PHONE_OFF	_IO(0x77,0)
#define PHONE_ON	_IO(0x77,1)
#define SPEAKER_OFF	_IO(0x77,2)
#define SPEAKER_ON	_IO(0x77,3)

#define EAR_SW_GPIO		21
#define SPEAKER_EN_GPIO		23
#define DETECT_HD_GPIO		139

struct audio_switch_t {
	struct cdev cdev;
	struct device *device;
	int	minor; 
	int 	irq;
	char 	*name;
	dev_t 	id;
	spinlock_t lock;
};
static struct audio_switch_t *paudio_switch;

static struct class *caudio_class;
static int caudio_major;
dev_t caudio_no;

int audio_switch_release(struct inode *inode, struct file *filp)
{
	struct audio_switch_t *dev;

	dev = container_of(inode->i_cdev, struct audio_switch_t, cdev); 

	free_irq(dev->irq, dev);
	
	return 0;
}

static irqreturn_t  audio_switch_func(int irq, void *dev)
{
	int state;
	hood_pr("AAAA:enter func=%s\n",__FUNCTION__);
	state = gpio_get_value(DETECT_HD_GPIO);

	gpio_set_value(SPEAKER_EN_GPIO, state);	

	return IRQ_HANDLED;
}

int audio_switch_open(struct inode *inode, struct file *filp)
{
	struct audio_switch_t *dev;
	int ret;
	
	dev = container_of(inode->i_cdev, struct audio_switch_t, cdev); 

	filp->private_data = dev;

	ret = request_irq(dev->irq, audio_switch_func, 
			IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING, 
			DNAME, dev);
	if(ret < 0)
		hood_pr("audio_switch irq request failed\n");

	return 0;
}

static long audio_switch_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int state;
	
	switch (cmd) {
		case PHONE_OFF:
			gpio_set_value(EAR_SW_GPIO, 0);
			mdelay(10);
			hood_pr("%s:PHONE_OFF\n",__FUNCTION__);
			break;

		case PHONE_ON:
			gpio_set_value(EAR_SW_GPIO, 1);
			mdelay(10);
			hood_pr("%s:PHONE_ON\n",__FUNCTION__);
			break;

		case SPEAKER_OFF:
			gpio_set_value(SPEAKER_EN_GPIO, 0);	
			mdelay(10);
			hood_pr("%s:SPEAKER_OFF\n",__FUNCTION__);
			break;

		case SPEAKER_ON:
			state = gpio_get_value(DETECT_HD_GPIO); 	
			gpio_set_value(SPEAKER_EN_GPIO, state);	
			mdelay(10);
			hood_pr("%s:SPEAKER_ON\n",__FUNCTION__);
			break;

		default:
			return - EINVAL;
	}

	return 0;
}

const static struct file_operations audio_switch_fops = {
	.owner	=	THIS_MODULE,
	.unlocked_ioctl	=	audio_switch_ioctl,
	.open	= 	audio_switch_open,
	.release = 	audio_switch_release,
};

static int audio_switch_create(void)
{
	int ret = 0;
	struct audio_switch_t *cur = paudio_switch;
	hood_pr("AAAA:enter func=%s\n",__FUNCTION__);
	cur->id = MKDEV(caudio_major, cur->minor);
	cdev_init(&cur->cdev, &audio_switch_fops);

	cur->irq = OMAP_GPIO_IRQ(DETECT_HD_GPIO);
	cur->name = DNAME;

	ret = cdev_add(&cur->cdev, cur->id, 1);
	if(ret)
	{
		hood_pr("Error: cdev_add() failed for %s\n", DNAME);
		goto out_err;
	}

	cur->device = device_create(caudio_class, NULL, cur->id,cur, "%s", DNAME);
	if(IS_ERR(cur->device))
	{
		hood_pr("Error: device_create() failed for %s\n", DNAME);
		ret = - ENODEV;
	}

out_err:
	return ret;
}

static void audio_switch_destroy(void)
{
	struct audio_switch_t *cur = paudio_switch;
	if(cur->device)
	{
		cdev_del(&cur->cdev);
		device_destroy(caudio_class,cur->id);
	}

	kfree(paudio_switch);
}

static int audio_switch_init(void)
{
	int result = 0;
	int ret = 0;
	hood_pr("AAAA:enter func=%s\n",__FUNCTION__);
	caudio_class = class_create(THIS_MODULE,DNAME);
	if(IS_ERR(caudio_class))
	{
		hood_pr("Error: class_create() failed for %s\n", DNAME);
		ret = - ENODEV;
		goto out_err;
	}

	result = alloc_chrdev_region(&caudio_no, 0, 1, DNAME);
	if(result < 0)
	{
		hood_pr("Error: alloc_chrdev_region() failed for %s\n", DNAME);
		ret = - ENODEV;
		goto out_err_1;
	}
	caudio_major = MAJOR(caudio_no);

	paudio_switch = kzalloc(sizeof(struct audio_switch_t), GFP_KERNEL); 
	if(!paudio_switch)
	{
		hood_pr("Error: kzalloc() failed for %s\n", DNAME);
		ret = - ENOMEM;
		goto out_err_2;
	}

	if (gpio_request(EAR_SW_GPIO, "ear switch") < 0)
		hood_pr(KERN_ERR "AAAA:can't control ear switch\n");
	else
		hood_pr("AAAA:can control ear switch\n");
	gpio_direction_output(EAR_SW_GPIO, 0);

	if (gpio_request(SPEAKER_EN_GPIO, "audio enable") < 0)
		hood_pr(KERN_ERR "AAAA:can't control speaker power\n");
	else
		hood_pr("AAAA:can control speaker power\n");
	gpio_direction_output(SPEAKER_EN_GPIO, 0);

	if (gpio_request(DETECT_HD_GPIO, "headphone detect") < 0)
		hood_pr(KERN_ERR "AAAA:can't detect headphone\n");
	else
		hood_pr("AAAA:can detect headphone\n");
	gpio_direction_input(DETECT_HD_GPIO);
	enable_irq(gpio_to_irq(DETECT_HD_GPIO));

	result = audio_switch_create();
	if(result)
	{
		hood_pr("Error: alloc_chrdev_region() failed for %s\n", DNAME);
		ret = - ENODEV;
		goto out_err_3;
	}
	
	return 0;

out_err_3:
	audio_switch_destroy();

out_err_2:
	unregister_chrdev_region(caudio_no, 1);

out_err_1:
	class_destroy(caudio_class);

out_err:
	return ret;
}

static void audio_switch_exit(void)
{
	audio_switch_destroy();

	if(caudio_class)
		class_destroy(caudio_class);

	if(caudio_no)
		unregister_chrdev_region(caudio_no, 1);
	
	gpio_free(EAR_SW_GPIO);
	gpio_free(SPEAKER_EN_GPIO);
}

module_init(audio_switch_init);
module_exit(audio_switch_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hood add for debug");


