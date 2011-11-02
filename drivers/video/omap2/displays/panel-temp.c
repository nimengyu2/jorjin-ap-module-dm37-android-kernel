/*
 * Generic panel support
 *
 * Copyright (C) 2008 Nokia Corporation
 * Author: Tomi Valkeinen <tomi.valkeinen@nokia.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/err.h>
#include <linux/slab.h>

#include <plat/display.h>

struct temp_data {
	struct backlight_device *bl;
};

static struct omap_video_timings temp_panel_timings = {
	/* 7 inch LCD */
	.x_res		= 800,
	.y_res		= 480,
	.pixel_clock	= 36000,
	.hfp		= 1,
	.hsw		= 48,
	.hbp		= 46,
	.vfp		= 12,
	.vsw		= 3,
	.vbp		= 23,
};

static int temp_panel_bl_update_status(struct backlight_device *bl)
{
	struct omap_dss_device *dssdev = dev_get_drvdata(&bl->dev);
	int level;

	if (!dssdev->set_backlight)
		return -EINVAL;

	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		level = bl->props.brightness;
	else
		level = 0;

	return dssdev->set_backlight(dssdev, level);
}

static int temp_panel_bl_get_brightness(struct backlight_device *bl)
{
	if (bl->props.fb_blank == FB_BLANK_UNBLANK &&
			bl->props.power == FB_BLANK_UNBLANK)
		return bl->props.brightness;

	return 0;
}

static const struct backlight_ops temp_panel_bl_ops = {
	.get_brightness = temp_panel_bl_get_brightness,
	.update_status  = temp_panel_bl_update_status,
};

static int temp_panel_probe(struct omap_dss_device *dssdev)
{
	struct backlight_properties props;
	struct backlight_device *bl;
	struct temp_data *sd;
	int r;

	dssdev->panel.config |= OMAP_DSS_LCD_TFT;
	//dssdev->panel.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS;
	//dssdev->panel.acb = 0x28;
	dssdev->panel.timings = temp_panel_timings;	

	sd = kzalloc(sizeof(*sd), GFP_KERNEL);
	if (!sd)
		return -ENOMEM;

	dev_set_drvdata(&dssdev->dev, sd);

	memset(&props, 0, sizeof(struct backlight_properties));
	props.max_brightness = dssdev->max_backlight_level;

	bl = backlight_device_register("temp-panel", &dssdev->dev, dssdev,
			&temp_panel_bl_ops, &props);
	if (IS_ERR(bl)) {
		r = PTR_ERR(bl);
		kfree(sd);
		return r;
	}
	sd->bl = bl;

	bl->props.fb_blank = FB_BLANK_UNBLANK;
	bl->props.power = FB_BLANK_UNBLANK;
	bl->props.brightness = dssdev->max_backlight_level;
	r = temp_panel_bl_update_status(bl);
	if (r < 0)
		dev_err(&dssdev->dev, "failed to set lcd brightness\n");

	return 0;
}

static void temp_panel_remove(struct omap_dss_device *dssdev)
{
	struct temp_data *sd = dev_get_drvdata(&dssdev->dev);
	struct backlight_device *bl = sd->bl;

	bl->props.power = FB_BLANK_POWERDOWN;
	temp_panel_bl_update_status(bl);
	backlight_device_unregister(bl);

	kfree(sd);
}

static int temp_panel_power_on(struct omap_dss_device *dssdev)
{
	int r;

	if (dssdev->state == OMAP_DSS_DISPLAY_ACTIVE)
		return 0;

	r = omapdss_dpi_display_enable(dssdev);
	if (r)
		goto err0;

	if (dssdev->platform_enable) {
		r = dssdev->platform_enable(dssdev);
		if (r)
			goto err1;
	}

	return 0;
err1:
	omapdss_dpi_display_disable(dssdev);
err0:
	return r;
}

static void temp_panel_power_off(struct omap_dss_device *dssdev)
{
	if (dssdev->state != OMAP_DSS_DISPLAY_ACTIVE)
		return;

	if (dssdev->platform_disable)
		dssdev->platform_disable(dssdev);

	omapdss_dpi_display_disable(dssdev);
}

static int temp_panel_enable(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = temp_panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

static void temp_panel_disable(struct omap_dss_device *dssdev)
{
	temp_panel_power_off(dssdev);

	dssdev->state = OMAP_DSS_DISPLAY_DISABLED;
}

static int temp_panel_suspend(struct omap_dss_device *dssdev)
{
	temp_panel_power_off(dssdev);
	dssdev->state = OMAP_DSS_DISPLAY_SUSPENDED;
	return 0;
}

static int temp_panel_resume(struct omap_dss_device *dssdev)
{
	int r = 0;

	r = temp_panel_power_on(dssdev);
	if (r)
		return r;

	dssdev->state = OMAP_DSS_DISPLAY_ACTIVE;

	return 0;
}

/*static void temp_panel_set_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	dpi_set_timings(dssdev, timings);
}

static void temp_panel_get_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	*timings = dssdev->panel.timings;
}

static int temp_panel_check_timings(struct omap_dss_device *dssdev,
		struct omap_video_timings *timings)
{
	return dpi_check_timings(dssdev, timings);
}*/

static struct omap_dss_driver temp_driver = {
	.probe		= temp_panel_probe,
	.remove		= temp_panel_remove,

	.enable		= temp_panel_enable,
	.disable	= temp_panel_disable,
	.suspend	= temp_panel_suspend,
	.resume		= temp_panel_resume,

	//.set_timings	= temp_panel_set_timings,
	//.get_timings	= temp_panel_get_timings,
	//.check_timings	= temp_panel_check_timings,

	.driver         = {
		.name   = "temp_panel",
		.owner  = THIS_MODULE,
	},
};

static int __init temp_panel_drv_init(void)
{
	return omap_dss_register_driver(&temp_driver);
}

static void __exit temp_panel_drv_exit(void)
{
	omap_dss_unregister_driver(&temp_driver);
}

module_init(temp_panel_drv_init);
module_exit(temp_panel_drv_exit);
MODULE_LICENSE("GPL");
