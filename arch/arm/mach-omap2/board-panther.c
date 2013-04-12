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

#ifdef CONFIG_TOUCHSCREEN_ADS7846
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

#ifdef CONFIG_WL12XX_PLATFORM_DATA
#include <linux/wl12xx.h>
#include <linux/regulator/fixed.h>
#endif

#ifdef CONFIG_TI_ST
#include <linux/ti_wilink_st.h>
#endif

#include <linux/interrupt.h>
#include <linux/dm9000.h>
#include <linux/i2c/tsc2007.h> // Modify by nmy
#include <plat/gpmc-16c554.h>

#include "mux.h"
#include "hsmmc.h"
#include "timer-gp.h"
#include "board-flash.h"
#include "pm34xx.h"

#define NAND_BLOCK_SIZE		SZ_128K

#ifdef CONFIG_USB_ANDROID

#define GOOGLE_VENDOR_ID		0x18d1
#define GOOGLE_PRODUCT_ID		0x9018
#define GOOGLE_ADB_PRODUCT_ID		0x9015

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

static struct android_usb_product usb_products[] = {
	{
		.product_id	= GOOGLE_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_adb),
		.functions	= usb_functions_adb,
	},
	{
		.product_id	= GOOGLE_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_mass_storage),
		.functions	= usb_functions_mass_storage,
	},
	{
		.product_id	= GOOGLE_PRODUCT_ID,
		.num_functions	= ARRAY_SIZE(usb_functions_ums_adb),
		.functions	= usb_functions_ums_adb,
	},
};

static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= 1,
	.vendor		= "rowboat",
	.product	= "rowboat gadget",
	.release	= 0x100,
};

static struct platform_device usb_mass_storage_device = {
	.name	= "usb_mass_storage",
	.id	= -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

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

static struct platform_device androidusb_device = {
	.name	= "android_usb",
	.id	= -1,
	.dev	= {
		.platform_data = &android_usb_pdata,
	},
};

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
static void __init omap3_beagle_pm_init(void)
{
    /* Use sys_offmode signal */
    omap_pm_sys_offmode_select(1);

    /* sys_clkreq - active high */
    omap_pm_sys_clkreq_pol(1);

    /* sys_offmode - active low */
    omap_pm_sys_offmode_pol(0);

    /* Automatically send OFF command */
    omap_pm_auto_off(1);

    /* Automatically send RET command */
    omap_pm_auto_ret(1);

    /*initialize sleep relational register for PM components*/
    omap3_pm_prm_polctrl_set(1,0);
    omap3_pm_prm_voltctrl_set(1,1,1);
}



#ifdef CONFIG_TOUCHSCREEN_ADS7846

#define PANTHER_TS_GPIO       157

#include <plat/mcspi.h>
#include <linux/spi/spi.h>

static struct omap2_mcspi_device_config ads7846_mcspi_config = {
        .turbo_mode     = 0,
        .single_channel = 1,    /* 0: slave, 1: master */
};

static int ads7846_get_pendown_state(void)
{
        return !gpio_get_value(PANTHER_TS_GPIO);
}

struct ads7846_platform_data ads7846_config = {
        .x_max                  = 0x0fff,
        .y_max                  = 0x0fff,
        .x_plate_ohms           = 180,
        .pressure_max           = 255,
        .debounce_max           = 10,
        .debounce_tol           = 3,
        .debounce_rep           = 1,
        .get_pendown_state      = ads7846_get_pendown_state,
        .keep_vref_on           = 1,
        .settle_delay_usecs     = 150,
        .wakeup                         = true,
};

static void __init panther_config_mcspi4_mux(void)
{
        omap_mux_init_signal("mcbsp1_clkr.mcspi4_clk", OMAP_PIN_INPUT);
        omap_mux_init_signal("mcbsp1_fsx.mcspi4_cs0", OMAP_PIN_OUTPUT);
        omap_mux_init_signal("mcbsp1_dx.mcspi4_simo", OMAP_PIN_OUTPUT);
        omap_mux_init_signal("mcbsp1_dr.mcspi4_somi", OMAP_PIN_INPUT_PULLUP);
}

struct spi_board_info panther_spi_board_info[] = {
        [0] = {
                .modalias               = "ads7846",
                .bus_num                = 4,
                .chip_select            = 0,
                .max_speed_hz           = 1500000,
                .controller_data        = &ads7846_mcspi_config,
                .irq                    = OMAP_GPIO_IRQ(PANTHER_TS_GPIO),
                .platform_data          = &ads7846_config,
        },
};

static void ads7846_dev_init(void)
{
	printk("Initialize ads7846 touch screen controller\n");

	if (gpio_request(PANTHER_TS_GPIO, "ADS7846 pendown") < 0)
		printk(KERN_ERR "can't get ads7846 pen down GPIO\n");

	gpio_direction_input(PANTHER_TS_GPIO);
	gpio_set_debounce(PANTHER_TS_GPIO, 0xa);
}
#endif

static struct mtd_partition panther_nand_partitions[] = {
	/* All the partition sizes are listed in terms of NAND block size */
	{
		.name		= "X-Loader",
		.offset		= 0,
		.size		= 4 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x80000 */
		.size		= 10 * NAND_BLOCK_SIZE,
		.mask_flags	= MTD_WRITEABLE,	/* force read-only */
	},
	{
		.name		= "U-Boot Env",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x1c0000 */
		.size		= 6 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "Kernel",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x280000 */
		.size		= 40 * NAND_BLOCK_SIZE,
	},
	{
		.name		= "File System",
		.offset		= MTDPART_OFS_APPEND,	/* Offset = 0x780000 */
		.size		= MTDPART_SIZ_FULL,
	},
};

/* DSS */

#ifdef CONFIG_PANEL_INNOLUX_AT070TN83
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

static int panther_enable_lcd(struct omap_dss_device *dssdev)
{
#define TWL_GPBR1	0xc
#define TWL_PWM0OFF	0x1
#define TWL_PMBR1	0xd
	twl_i2c_write_u8(TWL4030_MODULE_INTBR, 0x04, TWL_PMBR1);	// set GPIO.6 to PWM0
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
	&panther_lcd_device,
#endif
	&panther_dvi_device,
	&panther_tv_device,
};

static struct omap_dss_board_info panther_dss_data = {
	.num_devices = ARRAY_SIZE(panther_dss_devices),
	.devices = panther_dss_devices,
	.default_device = &panther_dvi_device,
};

static struct platform_device panther_dss_device = {
	.name          = "omapdss",
	.id            = -1,
	.dev            = {
		.platform_data = &panther_dss_data,
	},
};

static struct regulator_consumer_supply panther_vdac_supply =
	REGULATOR_SUPPLY("vdda_dac", "omapdss");

static struct regulator_consumer_supply panther_vdvi_supply =
	REGULATOR_SUPPLY("vdds_dsi", "omapdss");

static void __init panther_display_init(void)
{
	int r;

	/* DVI */
	panther_dvi_device.reset_gpio = 129;
	omap_mux_init_gpio(panther_dvi_device.reset_gpio, OMAP_PIN_INPUT);

	r = gpio_request(panther_dvi_device.reset_gpio, "DVI reset");
	if (r < 0) {
		printk(KERN_ERR "Unable to get DVI reset GPIO\n");
		return;
	}

	gpio_direction_output(panther_dvi_device.reset_gpio, 0);
}

#include "sdram-micron-mt46h32m32lf-6.h"

static struct omap2_hsmmc_info mmc[] = {
	{
		.mmc		= 1,
		.caps		= MMC_CAP_4_BIT_DATA,
		.gpio_wp	= -EINVAL,
	},
#ifdef CONFIG_WL12XX_PLATFORM_DATA
	{
	.name           = "wl1271",
	.mmc            = 2,
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

//#define PANTHER_BTEN_GPIO       15
#define PANTHER_BTEN_GPIO       138

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

/* VMMC2 for driving the WL12xx module */
static struct regulator_init_data panther_vmmc2 = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies = &panther_vmmc2_supply,
};

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
	.supply         = "cam_digital",
};

static struct regulator_consumer_supply panther_vaux4_supply = {
	.supply         = "cam_io",
};

static struct gpio_led gpio_leds[];

static int panther_twl_gpio_setup(struct device *dev,
		unsigned gpio, unsigned ngpio)
{
	/* gpio + 0 is "mmc0_cd" (input/IRQ) */
	mmc[0].gpio_cd = gpio + 0;
	omap2_hsmmc_init(mmc);

	/* link regulators to MMC adapters */
	panther_vmmc1_supply.dev = mmc[0].dev;
	// VSIM isn't a part of MMC1's system.
	//panther_vsim_supply.dev = mmc[0].dev;

	/* TWL4030_GPIO_MAX + 0 == ledA, EHCI nEN_USB_PWR (out, active low) */
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
static struct regulator_init_data panther_vmmc1 = {
	.constraints = {
		.min_uV			= 1850000,
		.max_uV			= 3150000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
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
static struct regulator_init_data panther_vdac = {
	.constraints = {
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &panther_vdac_supply,
};

/* VPLL2 for digital video outputs */
static struct regulator_init_data panther_vpll2 = {
	.constraints = {
		.name			= "VDVI",
		.min_uV			= 1800000,
		.max_uV			= 1800000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		.valid_ops_mask		= REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &panther_vdvi_supply,
};

/* VAUX3 for CAM_DIGITAL */
static struct regulator_init_data panther_vaux3 = {
	.constraints = {
		/* CAM_DIGITAL(DVDD) may vary from 1.5v to 1.8v */
		.min_uV                 = 1500000,
		.max_uV                 = 1800000,
		.apply_uV               = true,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
					| REGULATOR_MODE_STANDBY,
		/* Add REGULATOR_CHANGE_VOLTAGE flag to control voltage */
		.valid_ops_mask         = REGULATOR_CHANGE_VOLTAGE
					| REGULATOR_CHANGE_MODE
					| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &panther_vaux3_supply,
};

 /* VAUX4 for CAM_IO */
static struct regulator_init_data panther_vaux4 = {
	.constraints = {
		/* CAM_IO(DOVDD) may vary from 1.8v to 2.8v */
		.min_uV                 = 1800000,
		.max_uV                 = 2800000,
		.apply_uV               = true,
		.valid_modes_mask       = REGULATOR_MODE_NORMAL
			| REGULATOR_MODE_STANDBY,
		/* Add REGULATOR_CHANGE_VOLTAGE flag to control voltage */
		.valid_ops_mask         = REGULATOR_CHANGE_VOLTAGE
			| REGULATOR_CHANGE_MODE
			| REGULATOR_CHANGE_STATUS,
	},
	.num_consumer_supplies  = 1,
	.consumer_supplies      = &panther_vaux4_supply,
};

static struct twl4030_usb_data panther_usb_data = {
	.usb_mode	= T2_USB_MODE_ULPI,
};

/**
 ** Macro to configure resources
 **/
#define TWL4030_RESCONFIG(res,grp,typ1,typ2,state)  \
{                       \
    .resource   = res,          \
    .devgroup   = grp,          \
    .type       = typ1,         \
    .type2      = typ2,         \
    .remap_sleep    = state         \
}

static struct twl4030_resconfig  __initdata board_twl4030_rconfig[] = {
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
static struct twl4030_ins panther_sleep_seq[] __initdata = {
    { MSG_SINGULAR(DEV_GRP_NULL, RES_HFCLKOUT, RES_STATE_SLEEP), 20},
    { MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1, RES_STATE_SLEEP), 2 },
    { MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R2, RES_STATE_SLEEP), 2 },
};

static struct twl4030_script panther_sleep_script __initdata = {
    .script = panther_sleep_seq,
    .size   = ARRAY_SIZE(panther_sleep_seq),
    .flags  = TWL4030_SLEEP_SCRIPT,
};

/**
 ** Optimized 'Sleep to Active (P12)' sequence
 **/
static struct twl4030_ins panther_wake_p12_seq[] __initdata = {
    { MSG_BROADCAST(DEV_GRP_NULL, RES_GRP_ALL, RES_TYPE_R0, RES_TYPE2_R1, RES_STATE_ACTIVE), 2 }
};

static struct twl4030_script panther_wake_p12_script __initdata = {
    .script = panther_wake_p12_seq,
    .size   = ARRAY_SIZE(panther_wake_p12_seq),
    .flags  = TWL4030_WAKEUP12_SCRIPT,
}; 

/**
 ** Optimized 'Sleep to Active' (P3) sequence
 **/
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

static struct twl4030_script __initdata *board_twl4030_scripts[] = {
    &panther_wake_p12_script,
    &panther_wake_p3_script,
    &panther_sleep_script,
    &panther_wrst_script
};

static struct twl4030_power_data __initdata panther_script_data = {
    .scripts        = board_twl4030_scripts,
    .num            = ARRAY_SIZE(board_twl4030_scripts),
    .resource_config    = board_twl4030_rconfig,
};

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

static struct twl4030_platform_data panther_twldata = {
	.irq_base	= TWL4030_IRQ_BASE,
	.irq_end	= TWL4030_IRQ_END,

	/* platform_data for children goes here */
	.usb		= &panther_usb_data,
	.gpio		= &panther_gpio_data,
	.codec		= &panther_codec_data,
// The following table shows the relationship of pantherboard's regulators.
//
//	[LDO NAME]	[SCHEMATIC SYMBOLE]	[CONNECTED DEVICE (Outer/Inner)]
//	VPLL2		VDD_MIC		None / DSI, CSIPHY2
//	VMMC1		VDD_MMC1		Micro SD / None
//	VMMC2		VMMC2			None / Nnoe
//	VAUX1		CAM_ANA		None / None
//	VAUX2		EXP_VDD		USB PHY / None
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

static struct i2c_board_info __initdata panther_i2c_boardinfo[] = {
	{
		I2C_BOARD_INFO("twl4030", 0x48),
		.flags = I2C_CLIENT_WAKE,
		.irq = INT_34XX_SYS_NIRQ,
		.platform_data = &panther_twldata,
	},
};


//----------------------------------------------------------------------------//
//  nmy add tsc2007 code   start  2010-12-10  14:00
//----------------------------------------------------------------------------//
/* * TSC 2007 Support */
#define TSC2007_GPIO_IRQ_PIN_TMP	38
#define TSC2007_GPIO_IRQ_PIN	162
static int tsc2007_init_irq(void)
{	
		int ret = 0;        
		//pr_warning("%s: lierda_tcs2007_init_irq %d\n", __func__, ret);
		#if 1	
		omap_mux_init_gpio(TSC2007_GPIO_IRQ_PIN_TMP, OMAP_PIN_INPUT_PULLUP);	
		ret = gpio_request(TSC2007_GPIO_IRQ_PIN_TMP, "tsc2007-irq-tmp");	
		if (ret < 0) 
		{		
				printk("%s: failed to TSC2007 IRQ GPIO: %d\n", __func__, ret);		
				return ret;	
		}	
		else	
		{		
				printk("%s: ok to TSC2007 IRQ GPIO: %d\n", __func__, ret);	
		}	
		gpio_direction_input(TSC2007_GPIO_IRQ_PIN_TMP);	
		omap_mux_init_gpio(TSC2007_GPIO_IRQ_PIN, OMAP_PIN_INPUT_PULLUP);	
		ret = gpio_request(TSC2007_GPIO_IRQ_PIN, "tsc2007-irq");	
		if (ret < 0) 
		{		
				printk("%s: failed to TSC2007 IRQ GPIO: %d\n", __func__, ret);		
				return ret;	
		}	
		else	
		{		
				printk("%s: ok to TSC2007 IRQ GPIO: %d\n", __func__, ret);	
		}	
	  	gpio_direction_input(TSC2007_GPIO_IRQ_PIN);
		#endif
		
		#if 0	
		omap_mux_init_gpio(TSC2007_GPIO_IRQ_PIN, OMAP_PIN_OUTPUT);	
		//omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);	
		gpio_request(TSC2007_GPIO_IRQ_PIN, "tsc2007");	
		gpio_direction_output(TSC2007_GPIO_IRQ_PIN,1);	
		gpio_set_value(TSC2007_GPIO_IRQ_PIN,1);
		#endif	
		return ret;
}

static void tsc2007_exit_irq(void)
{	
		gpio_free(TSC2007_GPIO_IRQ_PIN);
}

static int tsc2007_get_irq_level(void)
{	
		//pr_warning("%s: lierda_tsc2007_get_irq_level %d\n", __func__, 0);	
		//lsd_ts_dbg(LSD_DBG,"enter tsc2007_get_irq_level\n");	
		return gpio_get_value(TSC2007_GPIO_IRQ_PIN) ? 0 : 1;
}

struct tsc2007_platform_data da850evm_tsc2007data = 
{	
		.model = 2007,	
		.x_plate_ohms = 180,	
		.get_pendown_state = tsc2007_get_irq_level,	
		.init_platform_hw = tsc2007_init_irq,	
		.exit_platform_hw = tsc2007_exit_irq,
};
//----------------------------------------------------------------------------//
//  nmy add tsc2007 code   end  2010-12-10  14:00
//----------------------------------------------------------------------------//

static struct i2c_board_info __initdata panther_i2c_eeprom[] = {
       {
               I2C_BOARD_INFO("eeprom", 0x50),
       },
	{	       
		I2C_BOARD_INFO("tsc2007", 0x48),	       
		.platform_data = &da850evm_tsc2007data,    
		.irq = OMAP_GPIO_IRQ(TSC2007_GPIO_IRQ_PIN),    
	},
};



static int __init panther_i2c_init(void)
{
	omap_register_i2c_bus(1, 2600, panther_i2c_boardinfo,
			ARRAY_SIZE(panther_i2c_boardinfo));

	/* Bus 2 is used for Camera/Sensor interface */
	omap_register_i2c_bus(2, 100, panther_i2c_eeprom, ARRAY_SIZE(panther_i2c_eeprom));
	
	/* Bus 3 is attached to the DVI port where devices like the pico DLP
	 * projector don't work reliably with 400kHz */
	omap_register_i2c_bus(3, 400, NULL, 0);

	return 0;
}

static struct gpio_led gpio_leds[] = {
// Pantherboard's leds aren't driven by GPIOs (except for D701(USB Active)).
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_led_info,
	},
};

static struct gpio_keys_button gpio_buttons[] = {
// Jack_20110907: defined the buttons on Chipsee's expansion board
//	To see the definitions of SCANCODE and KEYCODE, please refer to
//		-- <ROWBOAT_ANDROID>/kernel/include/linux/input.h
//		-- <ROWBOAT_ANDROID>/sdk/emulators/keymaps/qwerty.kl
//#ifdef CONFIG_TOUCHSCREEN_ADS7846

	{
		.code			= KEY_BACK,
		.gpio			= 157,
		.desc			= "157",
		.active_low		= true,
		.type                   = EV_KEY,
		.wakeup			= 1,
	},
//#endif
};

static struct gpio_keys_platform_data gpio_key_info = {
	.buttons	= gpio_buttons,
	.nbuttons	= ARRAY_SIZE(gpio_buttons),
};

static struct platform_device keys_gpio = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data	= &gpio_key_info,
	},
};

static void __init panther_init_irq(void)
{
	omap2_init_common_infrastructure();
	omap2_init_common_devices(mt46h32m32lf6_sdrc_params,
				  mt46h32m32lf6_sdrc_params);
	omap_init_irq();
	gpmc_init();
#ifdef CONFIG_OMAP_32K_TIMER
	omap2_gp_clockevent_set_gptimer(1);
#endif
}


//--------------------------------------------------------------//                             
                                                                                               
                                                                                               
#define OMAP_DM9000_BASE        0x2c000000                                                     
#define OMAP_DM9000_GPIO_IRQ    182                                                            
static struct resource omap3sbc8100_plus_dm9000_resources[] = {                                
        [0] =   {                                                                              
                .start  = OMAP_DM9000_BASE,                                                    
                .end    = (OMAP_DM9000_BASE + 0x4 - 1),                                        
                .flags  = IORESOURCE_MEM,                                                      
        },                                                                                     
        [1] =   {                                                                              
                .start  = (OMAP_DM9000_BASE + 0x10),                                           
                .end    = (OMAP_DM9000_BASE + 0x10 + 0x4 - 1),                                 
                .flags  = IORESOURCE_MEM,                                                      
        },                                                                                     
        [2] =   {                                                                              
                .start  = OMAP_GPIO_IRQ(OMAP_DM9000_GPIO_IRQ),                                 
                .end    = OMAP_GPIO_IRQ(OMAP_DM9000_GPIO_IRQ),                                 
                .flags  = IORESOURCE_IRQ | IRQF_TRIGGER_LOW,                                   
        },                                                                                     
};                                                                                             
                                                                                               
static struct dm9000_plat_data omap_dm9000_platdata = {                                        
        .flags = DM9000_PLATF_8BITONLY | DM9000_PLATF_NO_EEPROM,                              
	.dev_addr[0] =  0x5C,                                                                         
	.dev_addr[1] =  0x26,                                                                         
	.dev_addr[2] =  0x0A,                                                                         
	.dev_addr[3] =  0x17,                                                                         
	.dev_addr[4] =  0xD1,                                                                         
	.dev_addr[5] =  0x88,                                                                         
};                                                                                             
                                                                                               
static struct platform_device omap3sbc8100_plus_dm9000_device = {                              
        .name           = "dm9000",                                                            
        .id             = -1,                                                                  
        .num_resources  = ARRAY_SIZE(omap3sbc8100_plus_dm9000_resources),                      
        .resource       = omap3sbc8100_plus_dm9000_resources,                                  
        .dev            = {                                                                    
                .platform_data = &omap_dm9000_platdata,                                        
        },                                                                                     
};                                                                                             
                                                                                               
                                                                                               
#define NET_GPMC_CONFIG1	0x00001000                                                            
#define NET_GPMC_CONFIG2	0x001e1e00                                                            
#define NET_GPMC_CONFIG3	0x00080300                                                            
#define NET_GPMC_CONFIG4	0x1c091c09                                                            
#define NET_GPMC_CONFIG5	0x04181f1f                                                            
#define NET_GPMC_CONFIG6	0x00000FCF                                                            
#define NET_GPMC_CONFIG7	0x00000f6c                                                            
                                                                                               
#define GPMC_CS3 3                                                                             
                                                                                               
static const u32 gpmc_nor2[7] = {                                                              
	 NET_GPMC_CONFIG1,                                                                            
	 NET_GPMC_CONFIG2,                                                                            
	 NET_GPMC_CONFIG3 ,                                                                           
	 NET_GPMC_CONFIG4,                                                                            
	 NET_GPMC_CONFIG5,                                                                            
	 NET_GPMC_CONFIG6,                                                                            
	 NET_GPMC_CONFIG7                                                                             
};                                                                                             
                                                                                               
                                                                                               
static void __init omap3sbc8100_plus_init_dm9000(void)                                         
{                                                                                              
                                                                                               
	// init gpmc mux                                                                              
	omap_mux_init_signal("gpmc_ncs3", OMAP_PIN_OUTPUT);                                           
	omap_mux_init_signal("gpmc_ncs6", OMAP_PIN_OUTPUT);                                           
	omap_mux_init_signal("gpmc_ncs7", OMAP_PIN_OUTPUT);                                           
	omap_mux_init_signal("gpmc_a4", OMAP_PIN_OUTPUT);                                             
                                                                                               
	omap_mux_init_gpio(OMAP_DM9000_GPIO_IRQ, OMAP_PIN_OUTPUT);                                    
	//omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);                                         
	gpio_request(OMAP_DM9000_GPIO_IRQ, "dm9000");                                                 
	gpio_direction_output(OMAP_DM9000_GPIO_IRQ,1);                                                
	gpio_set_value(OMAP_DM9000_GPIO_IRQ,1);                                                       
                                                                                               
	omap_mux_init_gpio(OMAP_DM9000_GPIO_IRQ, OMAP_PIN_INPUT_PULLUP);                              
#if 0                                                                                          
        if (gpio_request(OMAP_DM9000_GPIO_IRQ, "dm9000 irq") < 0) {                            
                printk(KERN_ERR "Failed to request GPIO%d for dm9000 IRQ\n",                   
                        OMAP_DM9000_GPIO_IRQ);                                                 
		lsd_eth_dbg(LSD_ERR,"gpio_request OMAP_DM9000_GPIO_IRQ error\n");                           
                return;                                                                        
        }                                                                                      
	else                                                                                          
	{                                                                                             
		lsd_eth_dbg(LSD_OK,"gpio_request OMAP_DM9000_GPIO_IRQ ok\n");                               
	}                                                                                             
#endif                                                                                         
                                                                                               
        gpio_direction_input(OMAP_DM9000_GPIO_IRQ);                                            
                                                                                               
                                                                                               
	gpmc_cs_write_reg(GPMC_CS3, GPMC_CS_CONFIG1, gpmc_nor2[0]);                                   
                                                                                               
	gpmc_cs_write_reg(GPMC_CS3, GPMC_CS_CONFIG2, gpmc_nor2[1]);                                   
                                                                                               
	gpmc_cs_write_reg(GPMC_CS3, GPMC_CS_CONFIG3, gpmc_nor2[2]);                                   
                                                                                               
	gpmc_cs_write_reg(GPMC_CS3, GPMC_CS_CONFIG4, gpmc_nor2[3]);                                   
                                                                                               
	gpmc_cs_write_reg(GPMC_CS3, GPMC_CS_CONFIG5, gpmc_nor2[4]);                                   
                                                                                               
	gpmc_cs_write_reg(GPMC_CS3, GPMC_CS_CONFIG6, gpmc_nor2[5]);                                   
	                                                                                              
	//val = gpmc_cs_read_reg(GPMC_CS, GPMC_CS_CONFIG7);                                           
	//val |= (1 << 6);                                                                            
	gpmc_cs_write_reg(GPMC_CS3, GPMC_CS_CONFIG7, gpmc_nor2[6]);                                   
}                                                                                              



static struct platform_device *panther_devices[] __initdata = {
	&leds_gpio,
	&keys_gpio,
	&panther_dss_device,
	&usb_mass_storage_device,
#ifdef CONFIG_TI_ST
    &wl12xx_device,
    &btwilink_device,
#endif
	&omap3sbc8100_plus_dm9000_device,
	&st16c554_device,
};

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

static const struct ehci_hcd_omap_platform_data ehci_pdata __initconst = {

	//.port_mode[0] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[0] = EHCI_HCD_OMAP_MODE_PHY,
	//.port_mode[1] = EHCI_HCD_OMAP_MODE_PHY,
	.port_mode[1] = EHCI_HCD_OMAP_MODE_UNKNOWN,
	.port_mode[2] = EHCI_HCD_OMAP_MODE_UNKNOWN,

	.phy_reset  = true,
	//.reset_gpio_port[0]  = -EINVAL,
        .reset_gpio_port[0]  = 39,
	//.reset_gpio_port[1]  = 39,
	.reset_gpio_port[1]  = -EINVAL,
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
 

        //OMAP3_MUX(DSS_DATA0,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
        //OMAP3_MUX(DSS_DATA1,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
        //OMAP3_MUX(DSS_DATA2,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
        //OMAP3_MUX(DSS_DATA3,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
        //OMAP3_MUX(DSS_DATA4,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),
        //OMAP3_MUX(DSS_DATA5,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),

	OMAP3_MUX(DSS_DATA0,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
        OMAP3_MUX(DSS_DATA1,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
        OMAP3_MUX(DSS_DATA2,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
        OMAP3_MUX(DSS_DATA3,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
        OMAP3_MUX(DSS_DATA4,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
        OMAP3_MUX(DSS_DATA5,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA6,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
        OMAP3_MUX(DSS_DATA7,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
        OMAP3_MUX(DSS_DATA8,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
        OMAP3_MUX(DSS_DATA9,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
        OMAP3_MUX(DSS_DATA10,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
        OMAP3_MUX(DSS_DATA11,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA12,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
        OMAP3_MUX(DSS_DATA13,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
        OMAP3_MUX(DSS_DATA14,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
        OMAP3_MUX(DSS_DATA15,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
        OMAP3_MUX(DSS_DATA16,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
        OMAP3_MUX(DSS_DATA17,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
	OMAP3_MUX(DSS_DATA18,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
        OMAP3_MUX(DSS_DATA19,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
        OMAP3_MUX(DSS_DATA20,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
        OMAP3_MUX(DSS_DATA21,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
        OMAP3_MUX(DSS_DATA22,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
        OMAP3_MUX(DSS_DATA23,OMAP_MUX_MODE0|OMAP_PIN_OUTPUT),
        
//	OMAP3_MUX(UART2_RX,OMAP_MUX_MODE1|OMAP_PIN_OFF_INPUT_PULLDOWN),
	OMAP3_MUX(CAM_XCLKB,OMAP_MUX_MODE0|OMAP_PIN_OFF_INPUT_PULLDOWN),	
	
	OMAP3_MUX(CSI2_DY0,OMAP_MUX_MODE4|OMAP_PIN_OFF_INPUT_PULLDOWN),
	OMAP3_MUX(CSI2_DX1,OMAP_MUX_MODE4|OMAP_PIN_OFF_INPUT_PULLDOWN),
	OMAP3_MUX(CSI2_DY1,OMAP_MUX_MODE4|OMAP_PIN_OFF_INPUT_PULLDOWN),

	OMAP3_MUX(MCBSP1_CLKX, OMAP_MUX_MODE4 | OMAP_PIN_INPUT),
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


// nmy add for st16c554
static struct omap_16c554_platform_data board_16c554_data = {
	.cs		= 3,
	//.gpio_irq	= 149,
	//.flags		= GPMC_MUX_ADD_DATA | IORESOURCE_IRQ_LOWLEVEL,
	.flags		= IORESOURCE_IRQ_LOWLEVEL,
};

static void __init panther_init(void)
{
	omap3_mux_init(board_mux, OMAP_PACKAGE_CBP);
	panther_i2c_init();
	
	omap3sbc8100_plus_init_dm9000();

	platform_add_devices(panther_devices,
			ARRAY_SIZE(panther_devices));
	omap_serial_init();

	omap_mux_init_gpio(25, OMAP_PIN_OUTPUT);

	// 10.4inch demo for audio en
	omap_mux_init_gpio(167, OMAP_PIN_OUTPUT);
	gpio_request(167, "audio_en");
	gpio_direction_output(167, 1);

	omap_mux_init_gpio(24, OMAP_PIN_OUTPUT);
	gpio_request(24, "power");
	gpio_direction_output(24, 1);

	omap_mux_init_gpio(172, OMAP_PIN_OUTPUT);
	gpio_request(172, "power2");
	gpio_direction_output(172, 1);

	// 10.4inch demo for usb phy rst
	//omap_mux_init_gpio(159, OMAP_PIN_OUTPUT);

	// for 16c554 uart interrupt
	omap_mux_init_gpio(159, OMAP_PIN_INPUT);  // UARTA INT   mcbsp1_dr
	omap_mux_init_gpio(160, OMAP_PIN_INPUT);  // UARTB INT  cam_shutter
	omap_mux_init_gpio(184, OMAP_PIN_INPUT);  // UARTC INT  i2c3_scl

	// 16c554 gpmc bus pin init
	omap_mux_init_signal("gpmc_ncs4", OMAP_PIN_OUTPUT);                                           
	omap_mux_init_signal("gpmc_ncs5", OMAP_PIN_OUTPUT);                                           
	omap_mux_init_signal("gpmc_ncs6", OMAP_PIN_OUTPUT);   
	omap_mux_init_signal("gpmc_a2", OMAP_PIN_OUTPUT); 
	omap_mux_init_signal("gpmc_a3", OMAP_PIN_OUTPUT);                                         
	omap_mux_init_signal("gpmc_a4", OMAP_PIN_OUTPUT);
	


	usb_musb_init(&musb_board_data);
	usb_ehci_init(&ehci_pdata);
	panther_flash_init();

	// 16c554 init
	gpmc_16c554_init(&board_16c554_data);

#ifdef CONFIG_TOUCHSCREEN_ADS7846
	panther_config_mcspi4_mux();
	panther_spi_board_info[0].irq = gpio_to_irq(PANTHER_TS_GPIO);
	spi_register_board_info(panther_spi_board_info, ARRAY_SIZE(panther_spi_board_info));
	ads7846_dev_init();
#endif

	/* Ensure SDRC pins are mux'd for self-refresh */
	omap_mux_init_signal("sdrc_cke0", OMAP_PIN_OUTPUT);
	omap_mux_init_signal("sdrc_cke1", OMAP_PIN_OUTPUT);

	panther_display_init();
#ifdef CONFIG_USB_ANDROID
	panther_android_gadget_init();
#endif

#ifdef CONFIG_WL12XX_PLATFORM_DATA
	/* WL12xx WLAN Init */
	if (wl12xx_set_platform_data(&panther_wlan_data))
		pr_err("error setting wl12xx data\n");
	platform_device_register(&panther_wlan_regulator);
#endif

#ifdef CONFIG_TI_ST
    /* Config GPIO to output for BT */
	omap_mux_init_gpio(PANTHER_BTEN_GPIO, OMAP_PIN_OUTPUT);
#endif
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
