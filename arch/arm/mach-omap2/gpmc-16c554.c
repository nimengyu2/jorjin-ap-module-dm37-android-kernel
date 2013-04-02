/*
 * linux/arch/arm/mach-omap2/gpmc-smc91x.c
 *
 * Copyright (C) 2009 Nokia Corporation
 * Contact:	Tony Lindgren
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/smc91x.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/serial_8250.h>

#include <plat/board.h>
#include <plat/gpmc.h>
#include <plat/gpmc-16c554.h>
#include <linux/lierda_debug.h>

static struct omap_16c554_platform_data *gpmc_cfg;
#define	M_16C554_UARTCLK 7372800
static struct plat_serial8250_port st16c554_platform_data[] = {
	[0] = {			
		.irqflags = IRQF_TRIGGER_RISING,	
		.iotype		= UPIO_MEM,		
		.flags		= UPF_BOOT_AUTOCONF | UPF_IOREMAP,
		.regshift   = 2,
		.uartclk = M_16C554_UARTCLK	
	},
	[1] = {			
		.irqflags = IRQF_TRIGGER_RISING,	
		.iotype		= UPIO_MEM,		
		.flags		= UPF_BOOT_AUTOCONF | UPF_IOREMAP,
		.regshift   = 2,
		.uartclk = M_16C554_UARTCLK	
	},
	[2] = {			
		.irqflags = IRQF_TRIGGER_RISING,	
		.iotype		= UPIO_MEM,		
		.flags		= UPF_BOOT_AUTOCONF | UPF_IOREMAP,
		.regshift   = 2,
		.uartclk = M_16C554_UARTCLK	
	},
	[3] = {			
		.irqflags = IRQF_TRIGGER_RISING,	
		.iotype		= UPIO_MEM,		
		.flags		= UPF_BOOT_AUTOCONF | UPF_IOREMAP,
		.regshift   = 2,
		.uartclk = M_16C554_UARTCLK	
	},
	[4] = {			
		/* end of array */
	},
};



struct platform_device st16c554_device = {
	.name = "serial8250",
	.id = PLAT8250_DEV_PLATFORM,
	//.id = PLAT8250_DEV_EXAR_ST16C554,
	.dev = {
		.platform_data = st16c554_platform_data,
	}
};




static const u32 gpmc_st16c554[18]={
	/* ST16C554_GPMC_CONFIG1,                                                                            
	 ST16C554_GPMC_CONFIG2,                                                                            
	 ST16C554_GPMC_CONFIG3,                                                                           
	 ST16C554_GPMC_CONFIG4,                                                                            
	 ST16C554_GPMC_CONFIG5,                                                                            
	 ST16C554_GPMC_CONFIG6,*/
	0x00000003,
	0x001e1e02,
	0x000e0e02,
	0x1d0c1d0c,
	0x011c1F1F,
	0x00000FCF,

	0x00000003,
	0x001e1e02,
	0x000e0e02,
	0x1d0c1d0c,
	0x011c1F1F,
	0x00000FCF,
	
	0x00000003,
	0x001e1e02,
	0x000e0e02,
	0x1d0c1d0c,
	0x011c1F1F,
	0x00000FCF,
	
};                           

#define ST16C554_UARTA_CS	(0x2D000000)
#define ST16C554_UARTB_CS	(0x2E000000)
#define ST16C554_UARTC_CS	(0x2F000000)

#define ST16C554_UARTA_IRQ_GPIO 159
#define ST16C554_UARTB_IRQ_GPIO 160
#define ST16C554_UARTC_IRQ_GPIO 184

/*
 * Initialize smc91x device connected to the GPMC. Note that we
 * assume that pin multiplexing is done in the board-*.c file,
 * or in the bootloader.
 */
void __init gpmc_16c554_init(struct omap_16c554_platform_data *board_data)
{

	unsigned long cs_mem_base;
	int ret;
	unsigned long val;
	int status,i;
	unsigned int gpio_num[3] = {ST16C554_UARTA_IRQ_GPIO,
				    ST16C554_UARTB_IRQ_GPIO,
				    ST16C554_UARTC_IRQ_GPIO,
				    };

	for(i = 0; i < 3; i++)
	{
		status = gpio_request(gpio_num[i], "gpio_test\n");
		if (status < 0) {
			lsd_dbg(LSD_ERR,"failed to open GPIO %d\n", gpio_num[i]);	
			return status;
		}
		else
		{
			lsd_dbg(LSD_OK,"open GPIO %d ok\n", gpio_num[i]);	
		}
		gpio_direction_input(gpio_num[i]); 
		st16c554_platform_data[i].irq =  gpio_to_irq(gpio_num[i]);
	}


	for(i=0;i<3;i++)
	{                                                                                      
		gpmc_cs_write_reg(i+4, GPMC_CS_CONFIG1, gpmc_st16c554[i*6]);                                   
                gpmc_cs_write_reg(i+4, GPMC_CS_CONFIG2, gpmc_st16c554[i*6+1]);
		gpmc_cs_write_reg(i+4, GPMC_CS_CONFIG3, gpmc_st16c554[i*6+2]);
		gpmc_cs_write_reg(i+4, GPMC_CS_CONFIG4, gpmc_st16c554[i*6+3]);                                   
                gpmc_cs_write_reg(i+4, GPMC_CS_CONFIG5, gpmc_st16c554[i*6+4]);
		gpmc_cs_write_reg(i+4, GPMC_CS_CONFIG6, gpmc_st16c554[i*6+5]);
		//gpmc_cs_write_reg(i, GPMC_CS_CONFIG7, gpmc_nor2[6]+i-3); //f6d ,f6e,f6f
	} 
        gpmc_cs_write_reg(4, GPMC_CS_CONFIG7, 0xf6d); 
	gpmc_cs_write_reg(5, GPMC_CS_CONFIG7, 0xf6e); 
	gpmc_cs_write_reg(6, GPMC_CS_CONFIG7, 0xf6f);

	st16c554_platform_data[0].mapbase = (volatile unsigned long)ST16C554_UARTA_CS;
	st16c554_platform_data[1].mapbase = (volatile unsigned long)ST16C554_UARTB_CS;
	st16c554_platform_data[2].mapbase = (volatile unsigned long)ST16C554_UARTC_CS;
	
	#if 0
	while(1)
	{
		__raw_writeb(0xAA, st16c554_platform_data[3].iobase); 
		udelay(10);
	}
	#endif

	#if 0	
	if (platform_device_register(&st16c554_device) < 0) {
		//printk(KERN_ERR "Unable to register smc91x device\n");
		lsd_dbg(LSD_ERR,"platform_device_register(&16c554_device) not ok\n");
	}
	else
	{
		lsd_dbg(LSD_OK,"platform_device_register(&16c554_device) ok\n");
	}
	#endif

	return 0;	
}
