/*
 * linux/arch/arm/mach-omap2/board-panther.c
 *
 * Copyright (C) 2008 Texas Instruments
 *
 * Modified from mach-omap2/board-3430sdp.c
 *               mach-omap2/board-omap3beagle.c
 *
 * Initial code: Syed Mohammed Khasim
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
// ע����ļ��Ǹ�ap moduleʹ�õ�
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/leds.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>

#include <linux/mtd/mtd.h>
#include <linux/mtd/partitions.h>
#include <linux/mtd/nand.h>
#include <linux/mmc/host.h>

#include <linux/usb/android_composite.h>

#ifdef CONFIG_TOUCHSCREEN_ADS7846   // ���������ads7846��Ŵ�����оƬ
#include <linux/spi/ads7846.h>
#endif

#include <linux/regulator/machine.h>
#include <linux/i2c/twl.h>

#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>

#include <plat/board.h>
#include <plat/common.h>
#include <plat/display.h>
#include <plat/gpmc.h>
#include <plat/nand.h>
#include <plat/usb.h>

#ifdef CONFIG_WL12XX_PLATFORM_DATA  // wl12xxƽ̨��wifiģ��
#include <linux/wl12xx.h>
#include <linux/regulator/fixed.h>
#endif

#ifdef CONFIG_TI_ST
#include <linux/ti_wilink_st.h>
#endif

#include "mux.h"
#include "hsmmc.h"
#include "timer-gp.h"
#include "board-flash.h"
#include "pm34xx.h"

#define NAND_BLOCK_SIZE		SZ_128K   // nand�Ŀ��С

#ifdef CONFIG_USB_ANDROID

#define GOOGLE_VENDOR_ID		0x18d1   // ����ID
#define GOOGLE_PRODUCT_ID		0x9018   // ��ƷID
#define GOOGLE_ADB_PRODUCT_ID		0x9015  // adb ��ƷID

static char *usb_functions_adb[] = {
	"adb",
};

static char *usb_functions_mass_storage[] = {
	"usb_mass_storage",
};
static char *usb_functions_ums_adb[] = {
	"usb_mass_storage",
	"adb",
};

static char *usb_functions_all[] = {
	"adb", "usb_mass_storage",
};

// usb��Ʒ
static struct android_usb_product usb_products[] = {
	{
		.product_id	= GOOGLE_PRODUCT_ID,  // google ��Ʒ id
		.num_functions	= ARRAY_SIZE(usb_functions_adb),
		.functions	= usb_functions_adb,  // adb ����
	},
	{
		.product_id	= GOOGLE_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_mass_storage),
		.functions	= usb_functions_mass_storage,  // �������洢��
	},
	{
		.product_id	= GOOGLE_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
};

// rowboat ������������ ƽ̨����
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "rowboat",
	.product	= "rowboat gadget",
	.release	= 0x100,
};
// rowboat ������������ �豸
static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};
// rowboat usb ƽ̨���ݽṹ��
static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= GOOGLE_VENDOR_ID,
	.product_id	= GOOGLE_PRODUCT_ID,
	.functions	= usb_functions_all,
	.products	= usb_products,
	.num_products	= ARRAY_SIZE(usb_products),
	.version	= 0x0100,
	.product_name	= "rowboat gadget",
	.manufacturer_name	= "rowboat",
	.serial_number	= "20100720",
	.num_functions	= ARRAY_SIZE(usb_functions_all),
};

// android usb�豸
static struct platform_device androidusb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};

// panther android ����
static void panther_android_gadget_init(void)
{
	platform_device_register(&androidusb_device);
}
#endif

extern void omap_pm_sys_offmode_select(int);
extern void omap_pm_sys_offmode_pol(int);
extern void omap_pm_sys_clkreq_pol(int);
extern void omap_pm_auto_off(int);
extern void omap_pm_auto_ret(int);

/**
 *  * Board specific initialization of PM components
 *   */
 // beagle ��Դ�����ʼ��
static void __init omap3_beagle_pm_init(void)
{
    /* Use sys_offmode signal */  // ʹ��sys offmode�ź�
    omap_pm_sys_offmode_select(1);

    /* sys_clkreq - active high */  // sys_clkreq ����Ч 
    omap_pm_sys_clkreq_pol(1);

    /* sys_offmode - active low */   // sys_offmode  ����Ч
    omap_pm_sys_offmode_pol(0);

    /* Automatically send OFF command */  // �Զ�����off����
    omap_pm_auto_off(1);

    /* Automatically send RET command */  // �Զ�����ret����
    omap_pm_auto_ret(1);

    /*initialize sleep relational register for PM components*/
	// ��ʼ��������ؼĴ��������pm�豸
    omap3_pm_prm_polctrl_set(1,0);
    omap3_pm_prm_voltctrl_set(1,1,1);
}


// ���������ads7846������оƬ�����ڵ�����
#ifdef CONFIG_TOUCHSCREEN_ADS7846

#define PANTHER_TS_GPIO       157  // �ж�io��

#include <plat/mcspi.h>
#include <linux/spi/spi.h>

// mcspi����  ����Ϊ����
static struct omap2_mcspi_device_config ads7846_mcspi_config = {
        .turbo_mode     = 0,
        .single_channel = 1,    /* 0: slave, 1: master */
};
// ��ȡ����״̬
static int ads7846_get_pendown_state(void)
{
		// ��Ϊ����gpio��״̬�������ʾû�а���״̬�����ʾ����״̬
        return !gpio_get_value(PANTHER_TS_GPIO);  // ������·��ط��㣬û�а��·���0
}

// ads7846������Ϣ
struct ads7846_platform_data ads7846_config = {
        .x_max                  = 0x0fff,  // x������adֵ  12bit
        .y_max                  = 0x0fff,  // ��� 12bit
        .x_plate_ohms           = 180,  // ����ֵ
        .pressure_max           = 255,  // ѹ�����ֵ
        .debounce_max           = 10,  
        .debounce_tol           = 3,
        .debounce_rep           = 1,
        .get_pendown_state      = ads7846_get_pendown_state,  // ��ȡ����״̬
        .keep_vref_on           = 1,
        .settle_delay_usecs     = 150,    // �趨��ʱʱ��
        .wakeup                         = true,  // ����
};

// mcspi4���ŵĳ�ʼ��
static void __init panther_config_mcspi4_mux(void)
{
        omap_mux_init_signal("mcbsp1_clkr.mcspi4_clk", OMAP_PIN_INPUT);
        omap_mux_init_signal("mcbsp1_fsx.mcspi4_cs0", OMAP_PIN_OUTPUT);
        omap_mux_init_signal("mcbsp1_dx.mcspi4_simo", OMAP_PIN_OUTPUT);
        omap_mux_init_signal("mcbsp1_dr.mcspi4_somi", OMAP_PIN_INPUT_PULLUP);
}
// spi�ӿ���Ϣ�ṹ��
struct spi_board_info panther_spi_board_info[] = {
        [0] = {
                .modalias               = "ads7846",
                .bus_num                = 4,  // ����4��
                .chip_select            = 0,  // Ƭѡ0��
                .max_speed_hz           = 1500000,  // ����ٶ�
                .controller_data        = &ads7846_mcspi_config,
                .irq                    = OMAP_GPIO_IRQ(PANTHER_TS_GPIO),// io�ж�
                .platform_data          = &ads7846_config,
        },
};
// ads7846�豸��ʼ��
static void ads7846_dev_init(void)
{
	printk("Initialize ads7846 touch screen controller\n");
	// ����gpio���ж�
	if (gpio_request(PANTHER_TS_GPIO, "ADS7846 pendown") < 0)
		printk(KERN_ERR "can't get ads7846 pen down GPIO\n");
	// �趨gpio��Ϊ����
	gpio_direction_input(PANTHER_TS_GPIO);
	gpio_set_debounce(PANTHER_TS_GPIO, 0xa);  // �趨����
}
#endif

// nand����
static struct mtd_partition panther_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader",
		.offset		= 0,
		.size		= 4 * NAND_BLOCK_SIZE,  // 512KB
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */  // ֻ��
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 10 * NAND_BLOCK_SIZE,  // 1280KB
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot Env",   
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x1c0000 */
		.size		= 6 * NAND_BLOCK_SIZE,  // 768KB
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 40 * NAND_BLOCK_SIZE,  // 5MB
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x780000 */
		.size		= MTDPART_SIZ_FULL,   // ʣ��ռ�
	},
};

/* DSS */
// ��ʾ��

// ע�⣬����panther android������
// CONFIG_PANEL_INNOLUX_AT070TN83=y ���������ѡ�е�
#ifdef CONFIG_PANEL_INNOLUX_AT070TN83   // ���������at070tn83
// Please note that although we have added blacklight adjustment function in our code, we actually do not have a PWM connected to Chipsee's panel.
// This is just a model for further using. It will not change the backlight intensity.
// Also, this funtion is related to Android's light module at "<ROWBOAT>/hardware/ti/omap3/liblights/pantherboard". Please modify that module too if necessary.
static int panther_set_bl_intensity(struct omap_dss_device *dssdev, int level)
{
	unsigned char c;

	if (level > dssdev->max_backlight_level)
		level = dssdev->max_backlight_level;

	c = ((125 * (100 - level)) / 100) + 1;

/*
 * PWM0 register offsets (TWL4030_MODULE_PWM0)
 */
#define TWL_PWM0ON	0x0
	twl_i2c_write_u8(TWL4030_MODULE_PWM0, c, TWL_PWM0ON);

	return 0;
}

// ʹ��lcd
static int panther_enable_lcd(struct omap_dss_device *dssdev)
{
#define TWL_GPBR1	0xc
#define TWL_PWM0OFF	0x1
#define TWL_PMBR1	0xd
	twl_i2c_write_u8(TWL4030_MODULE_INTBR, 0x04, TWL_PMBR1);	// set GPIO.6 to PWM0 �趨Ϊpwm����
	twl_i2c_write_u8(TWL4030_MODULE_INTBR, 0x05, TWL_GPBR1);	// enable PWM0 output & clock
	twl_i2c_write_u8(TWL4030_MODULE_PWM0, 0x7F, TWL_PWM0OFF);

	// Workaround for pantherboard suspened issue (Unhandled fault: external abort on non-linefetch (0x1028) at 0xfa064010).
	//gpio_direction_input(PANTHER_TS_GPIO);
	//gpio_set_debounce(PANTHER_TS_GPIO, 0xa);

	return 0;
}

static void panther_disable_lcd(struct omap_dss_device *dssdev)
{
	twl_i2c_write_u8(TWL4030_MODULE_INTBR, 0x00, TWL_PMBR1);	// restore GPIO.6
	twl_i2c_write_u8(TWL4030_MODULE_INTBR, 0x00, TWL_GPBR1);	// disable PWM0 output & clock
	twl_i2c_write_u8(TWL4030_MODULE_PWM0, 0x7F, TWL_PWM0OFF);

	// Workaround for pantherboard suspened issue (Unhandled fault: external abort on non-linefetch (0x1028) at 0xfa064010).
	//gpio_direction_output(PANTHER_TS_GPIO, 1);
}

// ��Ӧdriver�е����� Panel-innolux-at070tn83.c�ļ�
static struct omap_dss_device panther_lcd_device = {
	.name			= "lcd",
	.driver_name		= "innolux_at_panel",
	.type			= OMAP_DISPLAY_TYPE_DPI,
	.phy.dpi.data_lines	= 24,
	.max_backlight_level	= 100,
	.platform_enable	= panther_enable_lcd,
	.platform_disable	= panther_disable_lcd,
	.set_backlight	= panther_set_bl_intensity,
};
#endif

static int panther_enable_dvi(struct omap_dss_device *dssdev)
{
	if (gpio_is_valid(dssdev->reset_gpio))
		gpio_set_value(dssdev->reset_gpio, 1);

	return 0;
}

static void panther_disable_dvi(struct omap_dss_device *dssdev)
{
	if (gpio_is_valid(dssdev->reset_gpio))
		gpio_set_value(dssdev->reset_gpio, 0);
}

static struct omap_dss_device panther_dvi_device = {
	.type = OMAP_DISPLAY_TYPE_DPI,
	.name = "dvi",
	.driver_name = "generic_panel",
	.phy.dpi.data_lines = 24,
	.reset_gpio = -EINVAL,
	.platform_enable = panther_enable_dvi,
	.platform_disable = panther_disable_dvi,
};

static struct omap_dss_device panther_tv_device = {
	.name = "tv",
	.driver_name = "venc",
	.type = OMAP_DISPLAY_TYPE_VENC,
	.phy.venc.type = OMAP_DSS_VENC_TYPE_SVIDEO,
};

static struct omap_dss_device *panther_dss_devices[] = {
#ifdef CONFIG_PANEL_INNOLUX_AT070TN83
	&panther_lcd_device,   // lcd�豸
#endif
	&panther_dvi_device,
	&panther_tv_device,
};

// panther dss�豸�ṹ��
static struct omap_dss_board_info panther_dss_data = {
	.num_devices = ARRAY_SIZE(panther_dss_devices),  // dss�豸����
	.devices = panther_dss_devices,  // dss�豸�ṹ��
	.default_device = &panther_dvi_device,  // Ĭ���豸
};

// omapdss�豸
// Core.c (drivers\video\omap2\dss):
static struct platform_device panther_dss_device = {
	.name          = "omapdss",
	.id            = -1,
	.dev            = {
		.platform_data = &panther_dss_data,
	},
};

static struct regulator_consumer_supply panther_vdac_supply =
	REGULATOR_SUPPLY("vdda_dac", "omapdss");  // vdda_dac�Ǹ�omapdssʹ�õ�

static struct regulator_consumer_supply panther_vdvi_supply =
	REGULATOR_SUPPLY("vdds_dsi", "omapdss");  // vdds_dsi�Ǹ�omapdssʹ�õ�

// ��ʾ����ʼ��
static void __init panther_display_init(void)
{
	int r;

	/* DVI */
	panther_dvi_device.reset_gpio = 129;
	// �趨Ϊreset����Ϊ���빦��
	omap_mux_init_gpio(panther_dvi_device.reset_gpio, OMAP_PIN_INPUT);

	r = gpio_request(panther_dvi_device.reset_gpio, "DVI reset");
	if (r < 0) {
		printk(KERN_ERR "Unable to get DVI reset GPIO\n");
		return;
	}
	// Ȼ���趨Ϊ���
	gpio_direction_output(panther_dvi_device.reset_gpio, 0);
}

#include "sdram-micron-mt46h32m32lf-6.h"

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,   // mmc���
		.caps		= MMC_CAP_4_BIT_DATA,  // 4bit
		.gpio_wp	= -EINVAL,
	},
#ifdef CONFIG_WL12XX_PLATFORM_DATA
	{
	.name           = "wl1271",
	.mmc            = 2,   // mmc���
	.caps           = MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD,
	.gpio_wp        = -EINVAL,
	.gpio_cd        = -EINVAL,
	.nonremovable   = true,
	},
#endif
	{}	/* Terminator */
};

static struct regulator_consumer_supply panther_vmmc1_supply = {
	.supply			= "vmmc",
};

#ifdef CONFIG_TI_ST

#define PANTHER_BTEN_GPIO       15

int plat_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
	/* TODO: wait for HCI-LL sleep */
    pr_info(" plat_kim_suspend \n");
	return 0;
}

int plat_kim_resume(struct platform_device *pdev)
{
    pr_info(" plat_kim_resume \n");
	return 0;
}

// wilinkƽ̨����  TI��оƬ����BT FM GPS����
/* TI BT, FM, GPS connectivity chip */
struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = PANTHER_BTEN_GPIO,
	.dev_name = "/dev/ttyO1",
	.flow_cntrl = 1,
	.baud_rate = 3000000,
	.suspend = plat_kim_suspend,
	.resume = plat_kim_resume,
};

static struct platform_device wl12xx_device = {
	.name           = "kim",
	.id             = -1,
	.dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};
#endif


#ifdef CONFIG_WL12XX_PLATFORM_DATA

#define PANTHER_WLAN_PMENA_GPIO       (16)
#define PANTHER_WLAN_IRQ_GPIO         (112)

static struct regulator_consumer_supply panther_vmmc2_supply =
	REGULATOR_SUPPLY("vmmc", "mmci-omap-hs.1");

/* VMMC2 for driving the  module */
// vmmc2��������wl12xxģ��WL12xx
static struct regulator_init_data panther_vmmc2 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,  // ���Խ�ֹ��ʹ��
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies = &panther_vmmc2_supply,  // �õ���
};

// �̶��ĵ�ѹ����
static struct fixed_voltage_config panther_vwlan = {
	.supply_name            = "vwl1271",
	.microvolts             = 1800000, /* 1.80V */
	.gpio                   = PANTHER_WLAN_PMENA_GPIO,
	.startup_delay          = 70000, /* 70ms */
	.enable_high            = 1,
	.enabled_at_boot        = 0,
	.init_data              = &panther_vmmc2,
};

static struct platform_device panther_wlan_regulator = {
	.name           = "reg-fixed-voltage",
	.id             = 1,
	.dev = {
		.platform_data  = &panther_vwlan,
	},
};
struct wl12xx_platform_data panther_wlan_data __initdata = {
	.irq = OMAP_GPIO_IRQ(PANTHER_WLAN_IRQ_GPIO),
	.board_ref_clock = WL12XX_REFCLOCK_38_XTAL, /* 38.4 MHz */
};
#endif

// This regulator has been enaled in u-boot. The following code is removed since we don't need to control it in kernel.
#if 0
static struct regulator_consumer_supply panther_vsim_supply = {
	.supply			= "vmmc_aux",
};
#endif

static struct regulator_consumer_supply panther_vaux3_supply = {
	.supply         = "cam_digital",  // ������
};

static struct regulator_consumer_supply panther_vaux4_supply = {
	.supply         = "cam_io",  // ������
};

static struct gpio_led gpio_leds[];

// twl��gpio����
static int panther_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	mmc[0].gpio_cd = gpio + 0;   
	omap2_hsmmc_init(mmc);  // ��ʼ��hsmmc

	/* link regulators to MMC adapters */
	panther_vmmc1_supply.dev = mmc[0].dev;
	// VSIM isn't a part of MMC1's system.
	//panther_vsim_supply.dev = mmc[0].dev;

	/* TWL4030_GPIO_MAX + 0 == ledA, EHCI nEN_USB_PWR (out, active low) */
	// ����gpio�ӿ�
	gpio_request(gpio + TWL4030_GPIO_MAX, "nEN_USB_PWR");
	gpio_direction_output(gpio + TWL4030_GPIO_MAX, 1);

	/* gpio + 1 on Panther controls the TFP410's enable line (active low) */
	gpio_request(gpio + 1, "nDVI_PWR_EN");
	gpio_direction_output(gpio + 1, 0);

	return 0;
}

static struct twl4030_gpio_platform_data panther_gpio_data = {
	.gpio_base	= OMAP_MAX_GPIO_LINES,
	.irq_base	= TWL4030_GPIO_IRQ_BASE,
	.irq_end	= TWL4030_GPIO_IRQ_END,
	.use_leds	= true,
	.pullups	= BIT(1),
	.pulldowns	= BIT(2) | BIT(6) | BIT(7) | BIT(8) | BIT(13)
				| BIT(15) | BIT(16) | BIT(17),
	.setup		= panther_twl_gpio_setup,
};

/* VMMC1 for MMC1 pins CMD, CLK, DAT0..DAT3 (20 mA, plus card == max 220 mA) */
// ���mmc1���ŵ� 
static struct regulator_init_data panther_vmmc1 = {
	.constraints = { // ����������
		.min_uV			= 1850000,  // ��͵�ѹ
		.max_uV			= 3150000,  // ��ߵ�ѹ
		.valid_modes_mask	= REGULATOR_MODE_NORMAL   // ����ģʽ
					| REGULATOR_MODE_STANDBY,  // ֧������
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE  // ��ѹ�ɸı�
					| REGULATOR_CHANGE_MODE  // ģʽ�ɸı�
					| REGULATOR_CHANGE_STATUS,  // ���������Ա���ֹ��ʹ��
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &panther_vmmc1_supply,
};

// VSIM is not used by pantherboard right now.
#if 0
/* VSIM for GPIO_126,GPIO_127 & GPIO_129 */
static struct regulator_init_data panther_vsim = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 3000000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &panther_vsim_supply,
};
#endif

/* VDAC for DSS driving S-Video (8 mA unloaded, max 65 mA) */
// vdac ����dss����s-video��ʱ��
static struct regulator_init_data panther_vdac = {
	.constraints = {
		.min_uV			= 1800000,  // ��ѹΪ1.8v
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL  // ����ģʽ
					| REGULATOR_MODE_STANDBY,  // ����֧������
		.valid_ops_mask		= REGULATOR_CHANGE_MODE  // ֧��ģʽ�ı�
					| REGULATOR_CHANGE_STATUS,  // ���������Ա���ֹ��ʹ��
	},
	.num_consumer_supplies	= 1, // �õ���
	.consumer_supplies	= &panther_vdac_supply,  // �õ��߲���
};

/* VPLL2 for digital video outputs */
// vpll2����������Ƶ���
static struct regulator_init_data panther_vpll2 = {
	.constraints = {
		.name			= "VDVI",  // ����
		.min_uV			= 1800000, // ��ѹ1.8v
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL  // ����
					| REGULATOR_MODE_STANDBY, // ֧������
		.valid_ops_mask		= REGULATOR_CHANGE_MODE // ֧���޸�ģʽ
					| REGULATOR_CHANGE_STATUS,// �������ܹ�����ֹ��ʹ��
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &panther_vdvi_supply,  // ��������Ϣ
};

/* VAUX3 for CAM_DIGITAL */
// vaux3��������cam�ӿ�
static struct regulator_init_data panther_vaux3 = {
	.constraints = {
		/* CAM_DIGITAL(DVDD) may vary from 1.5v to 1.8v */
		// ��������ͷ������1.5 �� 1.8v����
		.min_uV                 = 1500000,
		.max_uV                 = 1800000,
		.apply_uV               = true,  // 
		.valid_modes_mask       = REGULATOR_MODE_NORMAL // ��ͨģʽ
					| REGULATOR_MODE_STANDBY, // ���Խ�������
		/* Add REGULATOR_CHANGE_VOLTAGE flag to control voltage */
		.valid_ops_mask         = REGULATOR_CHANGE_VOLTAGE  // ��ѹ�ɵ�
					| REGULATOR_CHANGE_MODE  // ģʽ�ɵ�
					| REGULATOR_CHANGE_STATUS,  // ����ʹ�ܺͽ�ֹ
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &panther_vaux3_supply,  // �õ���
};

 /* VAUX4 for CAM_IO */
 // vaux4 ����cam��io�ӿ�
static struct regulator_init_data panther_vaux4 = {
	.constraints = {
		// ��ѹ�ɱ� ��1.8v��2.8v
		/* CAM_IO(DOVDD) may vary from 1.8v to 2.8v */
		.min_uV                 = 1800000,
		.max_uV                 = 2800000,
		.apply_uV               = true,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL  // ����ģʽ
			| REGULATOR_MODE_STANDBY, // ������
		/* Add REGULATOR_CHANGE_VOLTAGE flag to control voltage */
		.valid_ops_mask         = REGULATOR_CHANGE_VOLTAGE  // ��ѹ�ɵ�
			| REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &panther_vaux4_supply,  // ������
};

// twl4030 usb���ݽṹ��
static struct twl4030_usb_data panther_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,  
};

/**
 ** Macro to configure resources
 **/
 // �궨������������Դ
#define TWL4030_RESCONFIG(res,grp,typ1,typ2,state)  \
{                       \
    .resource   = res,          \
    .devgroup   = grp,          \
    .type       = typ1,         \
    .type2      = typ2,         \
    .remap_sleep    = state         \
}
// ��Ϊ���¼�����Դ��
// #define DEV_GRP_P1		0x1	/* P1: all OMAP devices ����omap�豸*/
// #define DEV_GRP_P2		0x2	/* P2: all Modem devices ����modem�豸*/
// #define DEV_GRP_P3		0x4	/* P3: all peripheral devices ���������豸*/
// #define DEV_GRP_ALL		0x7	/* P1/P2/P3: all devices ���е��豸*/

static struct twl4030_resconfig  __initdata board_twl4030_rconfig[] = {
	// ��Դ��RES_VPLL1������DEV_GRP_P1�飬type=3 type2=1 ����ӳ������״̬�ر�
    TWL4030_RESCONFIG(RES_VPLL1, DEV_GRP_P1, 3, 1, RES_STATE_OFF),      /* ? */
    TWL4030_RESCONFIG(RES_VINTANA1, DEV_GRP_ALL, 1, 2, RES_STATE_SLEEP),
    TWL4030_RESCONFIG(RES_VINTANA2, DEV_GRP_ALL, 0, 2, RES_STATE_SLEEP),
    TWL4030_RESCONFIG(RES_VINTDIG, DEV_GRP_ALL, 1, 2, RES_STATE_SLEEP),
    TWL4030_RESCONFIG(RES_VIO, DEV_GRP_ALL, 2, 2, RES_STATE_SLEEP),
    TWL4030_RESCONFIG(RES_VDD1, DEV_GRP_P1, 4, 1, RES_STATE_OFF),       /* ? */
    TWL4030_RESCONFIG(RES_VDD2, DEV_GRP_P1, 3, 1, RES_STATE_OFF),       /* ? */
    TWL4030_RESCONFIG(RES_REGEN, DEV_GRP_ALL, 2, 1, RES_STATE_SLEEP),
    TWL4030_RESCONFIG(RES_NRES_PWRON, DEV_GRP_ALL, 0, 1, RES_STATE_SLEEP),
    TWL4030_RESCONFIG(RES_CLKEN, DEV_GRP_ALL, 3, 2, RES_STATE_SLEEP),
    TWL4030_RESCONFIG(RES_SYSEN, DEV_GRP_ALL, 6, 1, RES_STATE_SLEEP),
    TWL4030_RESCONFIG(RES_HFCLKOUT, DEV_GRP_P3, 0, 2, RES_STATE_SLEEP), /* ? */
    TWL4030_RESCONFIG(0, 0, 0, 0, 0),
};

/**
 ** Optimized 'Active to Sleep' sequence
 **/
 // �Ż�����  ������� ����
static struct twl4030_ins panther_sleep_seq[] __initdata = {
    { MSG_SINGULAR(DEV_GRP_NULL, RES_HFCLKOUT, RES_STATE_SLEEP), 20},
    { MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1, RES_STATE_SLEEP), 2 },
    { MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_SLEEP), 2 },
};

// ���߽ű�
static struct twl4030_script panther_sleep_script __initdata = {
    .script = panther_sleep_seq,  // �ű�����
    .size   = ARRAY_SIZE(panther_sleep_seq),
    .flags  = TWL4030_SLEEP_SCRIPT,
};

/**
 ** Optimized 'Sleep to Active (P12)' sequence
 **/
 // �Ż����� ���ߵ�����p12 ����
static struct twl4030_ins panther_wake_p12_seq[] __initdata = {
    { MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1, RES_STATE_ACTIVE), 2 }
};

// ����p12�ű�
static struct twl4030_script panther_wake_p12_script __initdata = {
    .script = panther_wake_p12_seq,
    .size   = ARRAY_SIZE(panther_wake_p12_seq),
    .flags  = TWL4030_WAKEUP12_SCRIPT,
}; 

/**
 ** Optimized 'Sleep to Active' (P3) sequence
 **/
 // ˯�ߵ�����p3����
static struct twl4030_ins panther_wake_p3_seq[] __initdata = {
    { MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_ACTIVE), 2 }
};

static struct twl4030_script panther_wake_p3_script __initdata = {
    .script = panther_wake_p3_seq,
    .size   = ARRAY_SIZE(panther_wake_p3_seq),
    .flags  = TWL4030_WAKEUP3_SCRIPT,
};

/**
 ** Optimized warm reset sequence (for less power surge)
 **/
 // �Ż������ȸ�λ����  ��Թ���Ҫ�󲻸ߵ�ʱ��
static struct twl4030_ins panther_wrst_seq[] __initdata = {
    { MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_OFF), 0x2 },
    { MSG_SINGULAR(DEV_GRP_NULL, RES_MAIN_REF, RES_STATE_WRST), 2 },
    { MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_WRST), 0x2},
    { MSG_SINGULAR(DEV_GRP_NULL, RES_VUSB_3V1, RES_STATE_WRST), 0x2 },
    { MSG_SINGULAR(DEV_GRP_NULL, RES_VPLL1, RES_STATE_WRST), 0x2 },
    { MSG_SINGULAR(DEV_GRP_NULL, RES_VDD2, RES_STATE_WRST), 0x7 },
    { MSG_SINGULAR(DEV_GRP_NULL, RES_VDD1, RES_STATE_WRST), 0x25 },
    { MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_RC, RES_TYPE_ALL, RES_TYPE2_R0, RES_STATE_WRST), 0x2 },
    { MSG_SINGULAR(DEV_GRP_NULL, RES_RESET, RES_STATE_ACTIVE), 0x2 },

};

static struct twl4030_script panther_wrst_script __initdata = {
    .script = panther_wrst_seq,
    .size   = ARRAY_SIZE(panther_wrst_seq),
    .flags  = TWL4030_WRST_SCRIPT,
};

// ����Ľű�
static struct twl4030_script __initdata *board_twl4030_scripts[] = {
    &panther_wake_p12_script,
    &panther_wake_p3_script,
    &panther_sleep_script,
    &panther_wrst_script
};

// pather board �ű����ݽṹ��
static struct twl4030_power_data __initdata panther_script_data = {
    .scripts        = board_twl4030_scripts,
    .num            = ARRAY_SIZE(board_twl4030_scripts),
    .resource_config    = board_twl4030_rconfig,  // ��Դ����
};

// ��Ƶcodec����
static struct twl4030_codec_audio_data panther_audio_data = {
	.audio_mclk = 26000000,
	.digimic_delay = 1,
	.ramp_delay_value = 1,
	.offset_cncl_path = 1,
	.check_defaults = false,
	.reset_registers = false,
	.reset_registers = false,
};

static struct twl4030_codec_data panther_codec_data = {
	.audio_mclk = 26000000,
	.audio = &panther_audio_data,
};

// twl4030ƽ̨����
static struct twl4030_platform_data panther_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,  // �ж�
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.usb		= &panther_usb_data,
	.gpio		= &panther_gpio_data,
	.codec		= &panther_codec_data,
// The following table shows the relationship of pantherboard's regulators.
//
//	[LDO NAME]	[SCHEMATIC SYMBOLE]	[CONNECTED DEVICE (Outer/Inner)]
//	VPLL2		VDD_MIC		None / DSI, CSIPHY2
//	VMMC1		VDD_MMC1		Micro SD / None  sd��
//	VMMC2		VMMC2			None / Nnoe
//	VAUX1		CAM_ANA		None / None
//	VAUX2		EXP_VDD		USB PHY / None  usb����оƬ
//	VAUX3		CAM_DIGITAL		Camera Module / None
//	VAUX4		CAM_IO			Camera Module / None
// ========== Listed below are the regulators which used inside AP module ==========
//	VDAC		None			None / Video Buffer, DAC(S-Video & CVBS)
//	VPLL1		None			None / DPLL, DLL
//	VMIC1		None			None / Unknown
//	VMIC2		None			None / Unknown
//	VSIM		None			None / GPIO_126, GPIO127, GPIO_129
	.vmmc1		= &panther_vmmc1,
// VSIM is not used by pantherboard right now.
#if 0
	.vsim		= &panther_vsim,
#endif
	.vdac		= &panther_vdac,
	.vpll2		= &panther_vpll2,
	.vaux3		= &panther_vaux3,
	.vaux4		= &panther_vaux4,
    .power      = &panther_script_data,
};

// i2c�Ϲ��ص��豸  tps65950оƬ
static struct i2c_board_info __initdata panther_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &panther_twldata,
	},
};

// ������eepromоƬ
static struct i2c_board_info __initdata panther_i2c_eeprom[] = {
       {
               I2C_BOARD_INFO("eeprom", 0x50),
       },
};

// ��ʼ��i2c�ӿ�
static int __init panther_i2c_init(void)
{
	// iic1 ��ʼ��  ����panther�����豸
	omap_register_i2c_bus(1, 2600, panther_i2c_boardinfo,
			ARRAY_SIZE(panther_i2c_boardinfo));

	/* Bus 2 is used for Camera/Sensor interface */
	// ����camera�ӿںʹ������ӿ�
	omap_register_i2c_bus(2, 400, NULL, 0);

	/* Bus 3 is attached to the DVI port where devices like the pico DLP
	 * projector don't work reliably with 400kHz */
	 // ����3 ��������dvi�ӿڣ���Ϊ�ù�����400khz��ʱ���ȶ�������趨Ϊ100khz
	omap_register_i2c_bus(3, 100, panther_i2c_eeprom, ARRAY_SIZE(panther_i2c_eeprom));

	return 0;
}

// gpio led�ӿ�
static struct gpio_led gpio_leds[] = {
// Pantherboard's leds aren't driven by GPIOs (except for D701(USB Active)).
// pantherboard��ledû��ʹ��gpio����
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,  // led�ṹ��
	.num_leds	= ARRAY_SIZE(gpio_leds), // led����
};

// led��gpio�豸
static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,  // gpio��led��Ϣ
	},
};

// gpio�İ���
static struct gpio_keys_button gpio_buttons[] = {
// ����chipsee����չ��
// Jack_20110907: defined the buttons on Chipsee's expansion board
//	To see the definitions of SCANCODE and KEYCODE, please refer to
//		-- <ROWBOAT_ANDROID>/kernel/include/linux/input.h
//		-- <ROWBOAT_ANDROID>/sdk/emulators/keymaps/qwerty.kl
// �����Ҫ�鿴���壬��ο����ϵ��ļ�
// ����ads7846��������
#ifdef CONFIG_TOUCHSCREEN_ADS7846
	{
		.code			= KEY_MENU, // �˵���
		.gpio			= 137,
		.desc			= "s1",
		.active_low		= true,
		.wakeup			= 1,
	},
	{
		.code			= KEY_BACK,  // ���ؼ�
		.gpio			= 138,
		.desc			= "s2",
		.active_low		= true,
		.wakeup			= 1,
	},
#endif
};

// gpio�İ�����Ϣ
static struct gpio_keys_platform_data gpio_key_info = {
	.buttons	= gpio_buttons,   // �����ṹ��
	.nbuttons	= ARRAY_SIZE(gpio_buttons),  // ��������
};
// ʹ�õ�������gpio-keys
static struct platform_device keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_key_info,
	},
};
// ��ʼ���ж�
static void __init panther_init_irq(void)
{
	omap2_init_common_infrastructure();
	// ��ʼ��common�豸  sdrc����
	omap2_init_common_devices(mt46h32m32lf6_sdrc_params,
				  mt46h32m32lf6_sdrc_params);
	// ��ʼ���ж�
	omap_init_irq();
	// gpmc��ʼ��
	gpmc_init();
#ifdef CONFIG_OMAP_32K_TIMER
    // �趨clock�¼� �趨��ʱ��
	omap2_gp_clockevent_set_gptimer(1);
#endif
}

// panther�豸�б� ��ƽ̨ע��
static struct platform_device *panther_devices[] __initdata = {
	&leds_gpio,  // led gpio����
	&keys_gpio,  // key gpio����
	&panther_dss_device,  // dss��ʾ�豸
	&usb_mass_storage_device,  // usb�������洢��
#ifdef CONFIG_TI_ST
    &wl12xx_device,  // wifiģ���豸
    &btwilink_device,  // bt�豸
#endif
};
// flash��ʼ��
static void __init panther_flash_init(void)
{
	u8 cs = 0;
	u8 nandcs = GPMC_CS_NUM + 1;

	/* find out the chip-select on which NAND exists */
	while (cs < GPMC_CS_NUM) {
		u32 ret = 0;
		ret = gpmc_cs_read_reg(cs, GPMC_CS_CONFIG1);

		if ((ret & 0xC00) == 0x800) {
			printk(KERN_INFO "Found NAND on CS%d\n", cs);
			if (nandcs > GPMC_CS_NUM)
				nandcs = cs;
		}
		cs++;
	}

	if (nandcs > GPMC_CS_NUM) {
		printk(KERN_INFO "NAND: Unable to find configuration "
				 "in GPMC\n ");
		return;
	}

	if (nandcs < GPMC_CS_NUM) {
		printk(KERN_INFO "Registering NAND on CS%d\n", nandcs);
		board_nand_init(panther_nand_partitions,
			ARRAY_SIZE(panther_nand_partitions),
			nandcs, NAND_BUSWIDTH_16);
	}
}

// usb�ӿڳ�ʼ��
static const struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {

	.port_mode[0] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	.reset_gpio_port[0]  = -EINVAL,
	.reset_gpio_port[1]  = 39,
	.reset_gpio_port[2]  = -EINVAL
};

#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
#ifdef CONFIG_TOUCHSCREEN_ADS7846
	OMAP3_MUX(MCBSP1_FSR, OMAP_MUX_MODE4 | OMAP_PIN_INPUT_PULLDOWN),
#endif
#ifdef CONFIG_WL12XX_PLATFORM_DATA
	/* WLAN IRQ - GPIO 112 */
	OMAP3_MUX(CSI2_DX0, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),

	/* WLAN POWER ENABLE - GPIO 16 */
	OMAP3_MUX(ETK_D2, OMAP_MUX_MODE4 | OMAP_PIN_OUTPUT),

	/* MMC2 SDIO pin muxes for WL12xx */
	OMAP3_MUX(SDMMC2_CLK, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(SDMMC2_CMD, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(SDMMC2_DAT0, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(SDMMC2_DAT1, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(SDMMC2_DAT2, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
	OMAP3_MUX(SDMMC2_DAT3, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP),
#endif
    OMAP3_MUX(SYS_NIRQ, OMAP_MUX_MODE0 | OMAP_PIN_INPUT_PULLUP |
            OMAP_PIN_OFF_INPUT_PULLUP | OMAP_PIN_OFF_OUTPUT_LOW |
            OMAP_PIN_OFF_WAKEUPENABLE),
        OMAP3_MUX(CAM_D0,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
        OMAP3_MUX(CAM_D1,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
        OMAP3_MUX(CAM_D2,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
        OMAP3_MUX(CAM_D3,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
        OMAP3_MUX(CAM_D4,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
        OMAP3_MUX(CAM_D5,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
        OMAP3_MUX(CAM_D6,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
        OMAP3_MUX(CAM_D7,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),

        OMAP3_MUX(CAM_D8,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
        OMAP3_MUX(CAM_D9,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
        OMAP3_MUX(CAM_D10,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
        OMAP3_MUX(CAM_D11,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
 

        OMAP3_MUX(DSS_DATA0,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
        OMAP3_MUX(DSS_DATA1,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
        OMAP3_MUX(DSS_DATA2,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
        OMAP3_MUX(DSS_DATA3,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
        OMAP3_MUX(DSS_DATA4,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
        OMAP3_MUX(DSS_DATA5,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
        
//	OMAP3_MUX(UART2_RX,OMAP_MUX_MODE1|OMAP_PIN_OFF_INPUT_PULLDOWN),
	OMAP3_MUX(CAM_XCLKB,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),	
	
	OMAP3_MUX(CSI2_DY0,OMAP_MUX_MODE4|OMAP_PIN_OFF_INPUT_PULLDOWN),
	OMAP3_MUX(CSI2_DX1,OMAP_MUX_MODE4|OMAP_PIN_OFF_INPUT_PULLDOWN),
	OMAP3_MUX(CSI2_DY1,OMAP_MUX_MODE4|OMAP_PIN_OFF_INPUT_PULLDOWN),

	OMAP3_MUX(MCBSP1_CLKX,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
	OMAP3_MUX(MCBSP1_DX,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
	OMAP3_MUX(MCBSP4_CLKX,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
	OMAP3_MUX(MCBSP4_DR,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
	
	OMAP3_MUX(MCBSP4_DX,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
	OMAP3_MUX(MCBSP4_FSX,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
	OMAP3_MUX(MCBSP_CLKS,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
	OMAP3_MUX(ETK_D9,OMAP_MUX_MODE2|OMAP_PIN_OFF_INPUT_PULLDOWN),
	OMAP3_MUX(MCBSP3_DX,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
	OMAP3_MUX(MCBSP3_DR,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
	OMAP3_MUX(MCBSP3_CLKX,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
	OMAP3_MUX(MCSPI1_SIMO,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
	
	OMAP3_MUX(MCSPI1_SOMI,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
	OMAP3_MUX(ETK_D4,OMAP_MUX_MODE2|OMAP_PIN_OFF_INPUT_PULLDOWN),
	OMAP3_MUX(ETK_D5,OMAP_MUX_MODE2|OMAP_PIN_OFF_INPUT_PULLDOWN),
	OMAP3_MUX(ETK_D6,OMAP_MUX_MODE2|OMAP_PIN_OFF_INPUT_PULLDOWN),
	OMAP3_MUX(ETK_D7,OMAP_MUX_MODE2|OMAP_PIN_OFF_INPUT_PULLDOWN),

    { .reg_offset = OMAP_MUX_TERMINATOR },
};
#endif

static struct omap_musb_board_data musb_board_data = {
	.interface_type		= MUSB_INTERFACE_ULPI,
	.mode			= MUSB_OTG,
	.power			= 100,
};

// panter��ʼ��
static void __init panther_init(void)
{
	// ��ʼ��cbp��װ����
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBP);  // ap moduleʹ��cbp��װ
	// ��ʼ��iic
	panther_i2c_init();
	// ע��ƽ̨���豸������panther�豸
	platform_add_devices(panther_devices,
			ARRAY_SIZE(panther_devices));
	// ���ڳ�ʼ��
	omap_serial_init();

	// ��usb��ʼ��
	usb_musb_init(&musb_board_data);
	// usb ehci��ʼ��
	usb_ehci_init(&ehci_pdata);
	// flash��ʼ��
	panther_flash_init();
	// ���������ads7846��Ŵ�����оƬ
#ifdef CONFIG_TOUCHSCREEN_ADS7846
	// ����mux��ʼ��
	panther_config_mcspi4_mux();
	// �ж�gpio��
	panther_spi_board_info[0].irq = gpio_to_irq(PANTHER_TS_GPIO);
	// ע��spi�ӿڵ�оƬ����
	spi_register_board_info(panther_spi_board_info, ARRAY_SIZE(panther_spi_board_info));
	// ��ʼ�����豸
	ads7846_dev_init();
#endif

	/* Ensure SDRC pins are mux'd for self-refresh */
	// ȷ��sdrc������mux��Ϊ��ˢ��ģʽ
	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);

	// panther��ʾ��ʼ��
	panther_display_init();
#ifdef CONFIG_USB_ANDROID
	// ��ʼ��android��usb
	panther_android_gadget_init();
#endif

	// ��ʼ��wl12xxƽ̨����
#ifdef CONFIG_WL12XX_PLATFORM_DATA
	/* WL12xx WLAN Init */
	// wl12xx wifi��ʼ�����������趨ƽ̨����
	if (wl12xx_set_platform_data(&panther_wlan_data))
		pr_err("error setting wl12xx data\n");
	// ע��wlan ������
	platform_device_register(&panther_wlan_regulator);
#endif

#ifdef CONFIG_TI_ST
    /* Config GPIO to output for BT */
	// ����gpio��� ����bt
	omap_mux_init_gpio(PANTHER_BTEN_GPIO, OMAP_PIN_OUTPUT);
#endif
	// beagle�ĵ�Դ�����ʼ��
    omap3_beagle_pm_init();
}

MACHINE_START(PANTHER, "OMAP3 Panther Board")
	/* Maintainer: Syed Mohammed Khasim - http://beagleboard.org */
	.boot_params	= 0x80000100,
	.map_io		= omap3_map_io,
	.reserve	= omap_reserve,
	.init_irq	= panther_init_irq,
	.init_machine	= panther_init,
	.timer		= &omap_timer,
MACHINE_END
