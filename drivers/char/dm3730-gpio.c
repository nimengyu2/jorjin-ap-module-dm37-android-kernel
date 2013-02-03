#include <linux/miscdevice.h> 
#include <linux/delay.h> 
#include <linux/kernel.h> 
#include <linux/module.h> 
#include <linux/init.h> 
#include <linux/mm.h> 
#include <linux/fs.h> 
#include <linux/types.h> 
#include <linux/delay.h> 
#include <linux/moduleparam.h> 
#include <linux/slab.h> 
#include <linux/errno.h> 
#include <linux/ioctl.h> 
#include <linux/cdev.h> 
#include <linux/string.h> 
#include <linux/list.h> 
#include <linux/pci.h> 
#include <linux/gpio.h> 
#include <linux/version.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/completion.h>

#include <linux/delay.h>

extern int omap_mux_init_gpio(int gpio, int val);

// 以下是要给到app测试的
#define DEVICE_NAME "dm3730-gpio" //设备名(/dev/led) 
#define MOTOR_MAGIC 'g'
#define SET_LED127			_IOW(MOTOR_MAGIC, 127,int) 
#define SET_LED38			_IOW(MOTOR_MAGIC, 38,int) 
#define SET_LED37			_IOW(MOTOR_MAGIC, 37,int) 

// gpio setting
#define GPIO_LED127   		127
#define GPIO_LED38		38
#define GPIO_LED37		37


unsigned char gpio_input_array[] = {
};


unsigned char gpio_output_array[] = {
	GPIO_LED127,
	GPIO_LED38,
	GPIO_LED37,
};

// ioctl 函数的实现 
// 在应用用户层将通过 ioctl 函数向内核传递参数，以控制 LED的输出状态 
static int am1808_gpio_ioctl( 
 struct file *file,   
 unsigned int cmd,  
 unsigned long arg) 
{ 
	#if 1
	printk("dm3730-gpio:cmd interger is %d,hex is 0x%08x\n",(int)cmd,(unsigned int)cmd);
	printk("dm3730-gpio:arg interger is %d,hex is 0x%08x\n",(int)arg,(unsigned int)arg);
	#endif
   	switch (cmd)
   	{	
		case SET_LED127:   
		if(arg == 0)
		{
			gpio_set_value(GPIO_LED127,0); 
		}
		else
		{
			gpio_set_value(GPIO_LED127,1); 
		}
		break;
		case SET_LED37:   
		if(arg == 0)
		{
			gpio_set_value(GPIO_LED37,0); 
		}
		else
		{
			gpio_set_value(GPIO_LED37,1); 
		}
		break;
		case SET_LED38:   
		if(arg == 0)
		{
			gpio_set_value(GPIO_LED38,0); 
		}
		else
		{
			gpio_set_value(GPIO_LED38,1); 
		}
		break;
		#if 0
		case GET_KEY1:  
 		if(arg != 0)
		{
			if(gpio_get_value(GPIO_KEY1) == 0)			
				*((unsigned long*)arg) = 0;
			else
				*((unsigned long*)arg) = 1;
		}
		break;
		#endif
		default:
		printk("dm3730-gpio:ioctl cmd error\n");
		return -EINVAL; 
   	} 
   	return 0;       
} 
 
 
//  设备函数操作集，在此只有 ioctl函数，通常还有 read, write, open, close 等，因为本 LED驱动在下面已经
//  注册为 misc 设备，因此也可以不用 open/close  
static struct file_operations dev_fops = { 
 .owner = THIS_MODULE, 
 .unlocked_ioctl = am1808_gpio_ioctl, 
}; 
  
//  把 LED驱动注册为 MISC 设备 
static struct miscdevice misc = { 
  //动态设备号
  .minor = MISC_DYNAMIC_MINOR,  
  .name = DEVICE_NAME, 
  .fops = &dev_fops, 
}; 
 
 
// 设备初始化 
static int __init dev_init(void) 
{ 
	unsigned long TmpRegVal;
	int status;
	int ret;
	int gpio;
	int i;

	#if 0
	for(i = 0;i < sizeof(gpio_input_array);i++)
	{
		gpio = 	gpio_input_array[i];
		ret = gpio_request(gpio, "GPIO");
		if(ret)
			printk("request input gpio %d error\n",gpio);
		else
			printk("request input gpio %d  ok\n",gpio);
		gpio_direction_input(gpio);
	}
	#endif

	for(i = 0;i < sizeof(gpio_output_array);i++)
	{
		gpio = 	gpio_output_array[i];
		ret = gpio_request(gpio, "GPIO");
		if(ret)
			printk("request output gpio %d error\n",gpio);
		else
			printk("request output gpio %d  ok\n",gpio);
		gpio_direction_output(gpio,1);
	}

   	ret = misc_register(&misc); //注册设备 
   	printk (DEVICE_NAME"\tinitialized\n"); //打印初始化信息 
   	return ret; 
} 
 
static void __exit dev_exit(void) 
{ 
	misc_deregister(&misc); 
} 
 
// 模块初始化，仅当使用 insmod/podprobe 命令加载时有用，
// 如果设备不是通过模块方式加载，此处将不会被调用 
module_init(dev_init); 

// 卸载模块，当该设备通过模块方式加载后，
// 可以通过 rmmod 命令卸载，将调用此函数 
module_exit(dev_exit);

// 版权信息 
MODULE_LICENSE("GPL"); 
// 开发者信息 
MODULE_AUTHOR("Lierda EA nmy"); 
