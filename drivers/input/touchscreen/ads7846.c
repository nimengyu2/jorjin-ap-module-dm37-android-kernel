/* drivers/input/touchscreen/ft5x06_i2c_ts.c
 *
 * Copyright (C) 2010 Pixcir, Inc.
 *
 * ft5x06_i2c_ts.c V1.0  support multi touch
 * ft5x06_i2c_ts.c V1.5  add Calibration function:
 *
 * CALIBRATION_FLAG	1
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <asm/uaccess.h>
#include <linux/smp_lock.h>
#include <linux/delay.h>
#include <linux/slab.h>  

#include <mach/gpio.h>

#define VIRTUAL_KEYS
#define MAX_KEY_CNT         3
#define DRIVER_VERSION 	"v1.0"
#define DRIVER_AUTHOR 	"Sunyard"
#define DRIVER_DESC 	       "Ft5x06 I2C Touchscreen Driver with tune fuction"
#define DRIVER_LICENSE 	"GPL"

#define IO_RST              73
//#define IO_INT            153
#define SYD_DBG             1
#define SLAVE_ADDR		0x38
#ifndef I2C_MAJOR
	#define I2C_MAJOR 		125
#endif	
#define I2C_MINORS 		256
#define  CALIBRATION_FLAG	1

#define TOUCHSCREEN_MINX 0
#define TOUCHSCREEN_MAXX 800  
#define TOUCHSCREEN_MINY 0
#define TOUCHSCREEN_MAXY 480

int tsp_keycodes[MAX_KEY_CNT] ={
       // KEY_SEARCH, //217
        KEY_BACK,   //432
        KEY_HOME,   //208
        KEY_MENU    //130
};


static unsigned char status_reg = 0;

struct i2c_dev
{
	struct list_head list;
	struct i2c_adapter *adap;
	struct device *dev;

};

static struct i2c_driver ft5x06_i2c_ts_driver;
static struct class *i2c_dev_class;
static LIST_HEAD( i2c_dev_list);
static DEFINE_SPINLOCK( i2c_dev_list_lock);
/*********************************V2.0-Bee-0928-BOTTOM****************************************/

static struct workqueue_struct *ft5x06_wq;

struct ft5x06_i2c_ts_data
{
	char	 phys[32];
	struct i2c_client *client;
	struct input_dev *input;
	struct delayed_work work;
	int irq;
};

static int ts_gpio_init()
{

      int ret ;

      ret = gpio_request(IO_RST, "gpio-ts");
      if(ret)
	{
            printk("get ts gpio failed \n");  
            return -1;   
	}
	ret = gpio_direction_output(IO_RST, 1);
      if(ret)
	{
	  	printk("can not set tp gpio output \n");    
            return -1;    
	} 
      printk("config tp gpio successful\n");
      return 0;

}

static int ts_gpio_ctrl(unsigned char level)
{
     gpio_set_value(IO_RST, level); 
     return 0;
}

static int ts_gpio_free()
{
	gpio_free(IO_RST);
      return 0;
}


#ifdef VIRTUAL_KEYS
static ssize_t virtual_keys_show(struct kobject *kobj,
            struct kobj_attribute *attr, char *buf)
{

     return sprintf(buf,
         //__stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":-46:400:30:30"   //480-80

         __stringify(EV_KEY) ":" __stringify(KEY_BACK) ":850:432:20:20" //480-208

         ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":850:208:20:20" //480-272

         ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":850:130:20:20" //480 -432

         "\n");
}

static struct kobj_attribute virtual_keys_attr = {
    .attr = {
        .name = "virtualkeys.ft5x06",
        .mode = S_IRUGO,
    },
    .show = &virtual_keys_show,
};

static struct attribute *properties_attrs[] = {
    &virtual_keys_attr.attr,
    NULL

};

static struct attribute_group properties_attr_group = {
    .attrs = properties_attrs,
};



static void virtual_keys_init(void)
{
    int ret;
    struct kobject *properties_kobj;    
    properties_kobj = kobject_create_and_add("board_properties", NULL);
    if (properties_kobj)
        ret = sysfs_create_group(properties_kobj,
                     &properties_attr_group);
    if (!properties_kobj || ret)
        pr_err("failed to create board_properties\n");    
}
#endif


static void return_i2c_dev(struct i2c_dev *i2c_dev)
{
	spin_lock(&i2c_dev_list_lock);
	list_del(&i2c_dev->list);
	spin_unlock(&i2c_dev_list_lock);
	kfree(i2c_dev);
}

static struct i2c_dev *i2c_dev_get_by_minor(unsigned index)
{
	struct i2c_dev *i2c_dev;
	i2c_dev = NULL;

	spin_lock(&i2c_dev_list_lock);
	list_for_each_entry(i2c_dev, &i2c_dev_list, list)
	{
		if (i2c_dev->adap->nr == index)
		goto found;
	}
	i2c_dev = NULL;
	found: spin_unlock(&i2c_dev_list_lock);
#if SYD_DBG
      printk(" ft5x06 i2c_dev_get_by_minor is Exiting\n");
#endif
	return i2c_dev;
}

static struct i2c_dev *get_free_i2c_dev(struct i2c_adapter *adap)
{
	struct i2c_dev *i2c_dev;

	if (adap->nr >= I2C_MINORS)
	{
		printk(KERN_ERR "i2c-dev: Out of device minors (%d)\n",
				adap->nr);
		return ERR_PTR(-ENODEV);
	}

	i2c_dev = kzalloc(sizeof(*i2c_dev), GFP_KERNEL);
	if (!i2c_dev)
		return ERR_PTR(-ENOMEM);
	i2c_dev->adap = adap;

	spin_lock(&i2c_dev_list_lock);
	list_add_tail(&i2c_dev->list, &i2c_dev_list);
	spin_unlock(&i2c_dev_list_lock);
#if SYD_DBG
      printk(" ft5x06 get_free_i2c_dev is Exiting\n");
#endif
	return i2c_dev;
}


static void ft5x06_ts_poscheck(struct work_struct *work)
{
	struct ft5x06_i2c_ts_data *tsdata = container_of(work,
			struct ft5x06_i2c_ts_data,
			work.work);
	printk("enter poscheck.....\n\n");
	int touch_event;
	int posx1, posy1;
	unsigned char Rdbuf[7],auotpnum[1],Wrbuf[1];
	static int key_id=0x80;

	int i,ret;

	Wrbuf[0] = 0;

	ret = i2c_master_send(tsdata->client, Wrbuf, 1);
	
	if (ret != 1)
	{
	     #if SYD_DBG
		     printk("Unable to write to i2c touchscreen!\n");
	     #endif
		goto out;
	}

	ret = i2c_master_recv(tsdata->client, Rdbuf, sizeof(Rdbuf));

	if (ret != sizeof(Rdbuf))
	{
		dev_err(&tsdata->client->dev, "Unable to read i2c page!\n");
		goto out;
	}

	posx1 = (( (Rdbuf[5] & 0x0f )<< 8) | Rdbuf[6]);
	posy1 = 480-(( (Rdbuf[3] & 0x0f )<< 8) | Rdbuf[4]);
	

	touch_event = Rdbuf[3] & 0xc0;

      #if SYD_DBG
	     printk("touch event: posx=%d,posy=%d \n",posx1,posy1);
      #endif
	if (touch_event == 0x80)//touch
	{
  
		if (posx1 > 0 && posx1 < 800)  
		{       //report raw touch data
			input_report_abs(tsdata->input, ABS_X, posx1);
			input_report_abs(tsdata->input, ABS_Y, posy1);
			input_report_key(tsdata->input, BTN_TOUCH, 1);
			input_report_abs(tsdata->input, ABS_PRESSURE, 1);
		}
		else if((posx1>=800) && (posx1<=900) )
		{
			//if (posy1 > 397 && posy1 < 403)
			//{
			//	key_id = 0;	 //search
			//}
			 if ( posy1 > 429 && posy1 < 435)
			{
				key_id = 0;	 //back
			}
			else if (posy1 > 205  && posy1 < 213 )
			{
				key_id = 1;	 //home	
			}
			else if ( posy1 > 127 && posy1 < 133)
			{
				key_id = 2;	 //menu
			}				
	            input_report_key(tsdata->input, tsp_keycodes[key_id], 1);		
		}

	}

	else if (touch_event == 0x40)//press
	{
		if(key_id !=0x80 )  	
		{    

			input_report_key(tsdata->input , tsp_keycodes[key_id], 0);	//tsdata->input ,keyboard

			key_id=0x80;	
		}
		else
		{     //report raw touch data
		      input_report_key(tsdata->input, BTN_TOUCH, 0);
			input_report_abs(tsdata->input, ABS_PRESSURE, 0);
		}

	}

	input_sync(tsdata->input);

	out: enable_irq(tsdata->irq);

}

static irqreturn_t ft5x06_ts_isr(int irq, void *dev_id)
{
	struct ft5x06_i2c_ts_data *tsdata = dev_id;

	disable_irq_nosync(irq);
	queue_work(ft5x06_wq, &tsdata->work.work);

	return IRQ_HANDLED;
}

static int ft5x06_i2c_ts_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	struct ft5x06_i2c_ts_data *tsdata;
	struct input_dev *input;
	struct device *dev;
	struct i2c_dev *i2c_dev;
	int error;
	int ret;

	#if SYD_DBG
	      printk("ft5x06_i2c_ts_probe client name is %s, irq is %x\n",client->name, client->irq);
	#endif

      ts_gpio_init();
      ts_gpio_ctrl(1);
	mdelay(20);
      ts_gpio_ctrl(0);
	mdelay(50);
      ts_gpio_ctrl(1);
	mdelay(20);

	tsdata = kzalloc(sizeof(*tsdata), GFP_KERNEL);
	if (!tsdata)
	{
		dev_err(&client->dev, "failed to allocate driver data!\n");
		error = -ENOMEM;
		goto exit_alloc_data_failed;		
	}
#ifdef VIRTUAL_KEYS
      virtual_keys_init();
#endif
	dev_set_drvdata(&client->dev, tsdata);

	input = input_allocate_device();
	if (!input)
	{
		dev_err(&client->dev, "failed to allocate input device!\n");
		error = -ENOMEM;
		goto exit_input_dev_alloc_failed;
	}

	set_bit(EV_SYN, input->evbit);
	set_bit(EV_KEY, input->evbit);
	set_bit(EV_ABS, input->evbit);
	set_bit(BTN_TOUCH, input->keybit);

#ifdef  VIRTUAL_KEYS
	input_set_capability(input, EV_KEY, KEY_SEARCH);
	input_set_capability(input, EV_KEY, KEY_BACK);
	input_set_capability(input, EV_KEY, KEY_HOME);
	input_set_capability(input, EV_KEY, KEY_MENU);
#endif
	input_set_abs_params(input, ABS_X, TOUCHSCREEN_MINX, TOUCHSCREEN_MAXX, 0, 0);
	input_set_abs_params(input, ABS_Y, TOUCHSCREEN_MINY, TOUCHSCREEN_MAXY, 0, 0);	

	input_set_abs_params(input,ABS_PRESSURE,0,1,0,0);

	snprintf(tsdata->phys, sizeof(tsdata->phys), "%s/input0", dev_name(&client->dev));

	input->name = client->name;
	input->phys = tsdata->phys;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;
	input->keycode = tsp_keycodes; //add by jerry
#if 0
      input->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
      input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
      input->absbit[0] = BIT(ABS_X) | BIT(ABS_Y);
#endif

	input_set_drvdata(input, tsdata);

	tsdata->client = client;
	tsdata->input = input;
	printk("poscheck......\n\n");
	
	INIT_WORK(&tsdata->work.work, ft5x06_ts_poscheck);
	printk("exit poschek.......\n\n");

	tsdata->irq = client->irq; //touchscreen  Interrupt (GPIO 153) 

	error = input_register_device(input);
	if (error){
		dev_err(&client->dev, "failed to register input device!\n");
		goto exit_input_register_device_failed;
	}
	error = request_irq(tsdata->irq, ft5x06_ts_isr, IRQF_TRIGGER_FALLING, client->name, tsdata); //	
	if (error)
	{
		dev_err(&client->dev, "Unable to request touchscreen IRQ.\n");
		goto exit_irq_request_failed;
	}

	device_init_wakeup(&client->dev, 1);

	/*********************************V2.0-Bee-0928-TOP****************************************/
	i2c_dev = get_free_i2c_dev(client->adapter);
	if (IS_ERR(i2c_dev))
	{
		error = PTR_ERR(i2c_dev);
		goto  out;
	}
	/*********************************V2.0-Bee-0928-BOTTOM****************************************/
	dev_err(&tsdata->client->dev, "insmod successfully!\n");	
	return 0;
	
out:
	free_irq(tsdata->irq, tsdata);
exit_irq_request_failed:
exit_input_register_device_failed:
	input_free_device(input);
exit_input_dev_alloc_failed:
	kfree(tsdata);
exit_alloc_data_failed:
	return error;	

}

static int ft5x06_i2c_ts_remove(struct i2c_client *client)
{
	int error;
	struct i2c_dev *i2c_dev;
	struct ft5x06_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);
	free_irq(tsdata->irq, tsdata);
	/*********************************V2.0-Bee-0928-TOP****************************************/
	i2c_dev = get_free_i2c_dev(client->adapter);
	if (IS_ERR(i2c_dev))
	{
		error = PTR_ERR(i2c_dev);
		return error;
	}
	return_i2c_dev(i2c_dev);
	device_destroy(i2c_dev_class, MKDEV(I2C_MAJOR, client->adapter->nr));
	/*********************************V2.0-Bee-0928-BOTTOM****************************************/
	input_unregister_device(tsdata->input);
	kfree(tsdata);
	dev_set_drvdata(&client->dev, NULL);

	ts_gpio_free(); // add by jerry
	return 0;
}

static int ft5x06_i2c_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ft5x06_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);
	
	disable_irq(tsdata->irq);
	
	return 0;
}

static int ft5x06_i2c_ts_resume(struct i2c_client *client)
{
	struct ft5x06_i2c_ts_data *tsdata = dev_get_drvdata(&client->dev);
	
	enable_irq(tsdata->irq);

	return 0;
}



static const struct i2c_device_id ft5x06_i2c_ts_id[] =
{
	{ "ft5x06", 0 },
	{ }
};
MODULE_DEVICE_TABLE( i2c, ft5x06_i2c_ts_id);

static struct i2c_driver ft5x06_i2c_ts_driver =
{ 	.driver =
	{
		.owner = THIS_MODULE,
		.name = "ft5x06_touchscreen", 
	}, 
	.probe = ft5x06_i2c_ts_probe, 
	.remove = ft5x06_i2c_ts_remove,
	.suspend = ft5x06_i2c_ts_suspend, 
	.resume = ft5x06_i2c_ts_resume,
	.id_table = ft5x06_i2c_ts_id, 
};

static int __init ft5x06_i2c_ts_init(void)
{
	int ret;
	
	ft5x06_wq = create_singlethread_workqueue("ft5x06_wq");
	if(!ft5x06_wq)
	return -ENOMEM;
	

	i2c_dev_class = class_create(THIS_MODULE, "ft5x06_i2c_dev");
	if (IS_ERR(i2c_dev_class))
	{
		ret = PTR_ERR(i2c_dev_class);
		class_destroy(i2c_dev_class);
	}
	#if SYD_DBG
	      printk("ft5x06_i2c_ts_init is Entrying\n");
	#endif
	/********************************V2.0-Bee-0928-BOTTOM******************************************/
	return i2c_add_driver(&ft5x06_i2c_ts_driver);
}

static void __exit ft5x06_i2c_ts_exit(void)
{
	i2c_del_driver(&ft5x06_i2c_ts_driver);
	/********************************V2.0-Bee-0928-TOP******************************************/
	class_destroy(i2c_dev_class);
	unregister_chrdev(I2C_MAJOR,"ft5x06_i2c_ts");
	/********************************V2.0-Bee-0928-BOTTOM******************************************/
	if(ft5x06_wq)
		destroy_workqueue(ft5x06_wq);
}

MODULE_AUTHOR(DRIVER_AUTHOR);
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE(DRIVER_LICENSE);

module_init( ft5x06_i2c_ts_init);
module_exit( ft5x06_i2c_ts_exit);

