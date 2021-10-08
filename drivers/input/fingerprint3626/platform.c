/*
 * platform indepent driver interface
 * Copyright (C) 2016 Goodix
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/timer.h>
#include <linux/err.h>
#include <drm/drm_panel.h>
#include <linux/of.h>

#include "gf_spi.h"

#if defined(USE_SPI_BUS)
#include <linux/spi/spi.h>
#include <linux/spi/spidev.h>
#elif defined(USE_PLATFORM_BUS)
#include <linux/platform_device.h>
#endif

int parse_dt_panel(struct gf_dev *gf_dev)
{
	int i;
	int count;
    struct device_node *panel_node = NULL;
	struct drm_panel *panel;

    struct device *dev = &gf_dev->spi->dev;
	struct device_node *node = dev->of_node;
	pr_err("gf:parse_dt_panel start\n");
	count = of_count_phandle_with_args(node, "panel", NULL);
	pr_err("gf:parse_dt_panel , count = %d \n" , count);
	if (count <= 0)
		return -1;

	for (i = 0; i < count; i++) {
		panel_node = of_parse_phandle(node, "panel", i);
		if( panel_node == NULL ){
			pr_err("gf:parse_dt_panel ,panel_node == NULL\n");
		}
		panel = of_drm_find_panel(panel_node);
		if( panel == NULL ){
			pr_err("gf:parse_dt_panel ,panel == NULL\n");
		}
		of_node_put(panel_node);
		pr_err("gf:parse_dt_panel ---64---\n");
		if (!IS_ERR(panel)) {
			gf_dev->active_panel_asus = panel;
			pr_err("gf:parse_dt_panel , return 0\n");
			return 0;
		}else{
			pr_err("gf:parse_dt_panel , %d\n",PTR_ERR(panel));
		}
	}
    pr_err("gf:parse_dt_panel end\n");
	return 0;
}


int gf_parse_dts(struct gf_dev *gf_dev)
{
	int rc = 0;
	struct device *dev = &gf_dev->spi->dev;
	struct device_node *np = dev->of_node;

	gf_dev->irq_gpio = of_get_named_gpio(np, "goodix,gpio_irq", 0);
	if (gf_dev->irq_gpio < 0) {
		pr_err("falied to get irq gpio!\n");
		return gf_dev->irq_gpio;
	}
	rc = devm_gpio_request(dev, gf_dev->irq_gpio, "goodix_irq");
	if (rc) {
		pr_err("failed to request irq gpio, rc = %d\n", rc);
		goto err_irq;
	}
	gpio_direction_input(gf_dev->irq_gpio);
	pr_info("gf_parse_dts,gf_dev->irq_gpio = %d\n", gf_dev->irq_gpio);
	
	
	gf_dev->reset_gpio = of_get_named_gpio(np, "goodix,gpio_reset", 0);
	if (gf_dev->reset_gpio < 0) {
		pr_err("falied to get reset gpio!\n");
		return gf_dev->reset_gpio;
	}
	rc = devm_gpio_request(dev, gf_dev->reset_gpio, "goodix_reset");
	if (rc) {
		pr_err("failed to request reset gpio, rc = %d\n", rc);
		goto err_irq;
	}
	gpio_direction_output(gf_dev->reset_gpio,0);
	pr_info("gf_parse_dts,gf_dev->reset_gpio = %d\n", gf_dev->reset_gpio);


    gf_dev->vendorId_gpio = of_get_named_gpio(np, "goodix,gpio_vendor_id", 0);
	if (gf_dev->vendorId_gpio < 0) {
		pr_err("falied to get vendorId gpio!\n");
		return gf_dev->vendorId_gpio;
	}
	rc = devm_gpio_request(dev, gf_dev->vendorId_gpio, "goodix_vendor_id");
	if (rc) {
		pr_err("failed to request vendorId gpio, rc = %d\n", rc);
		goto err_irq;
	}
	gpio_direction_input(gf_dev->vendorId_gpio);
	pr_info("gf_parse_dts,gf_dev->vendorId_gpio = %d\n", gf_dev->vendorId_gpio);


	gf_dev->pwr_gpio = of_get_named_gpio(np, "goodix,gpio_ldo", 0);
	if (gf_dev->pwr_gpio < 0) {
		pr_err("falied to get power gpio!\n");
		return gf_dev->pwr_gpio;
	}

	rc = devm_gpio_request(dev, gf_dev->pwr_gpio, "goodix_ldo");
	if (rc) {
		pr_err("failed to request power gpio, rc = %d\n", rc);
	}

	pr_err("gf_parse_dts end! , rc = %d\n" ,rc);
	return rc;
err_irq:
	devm_gpio_free(dev, gf_dev->irq_gpio);

	return rc;
}

void gf_cleanup(struct gf_dev *gf_dev)
{
	pr_info("[info] %s\n", __func__);

	if (gpio_is_valid(gf_dev->irq_gpio)) {
		gpio_free(gf_dev->irq_gpio);
		pr_info("remove irq_gpio success\n");
	}
	if (gpio_is_valid(gf_dev->reset_gpio)) {
		gpio_free(gf_dev->reset_gpio);
		pr_info("remove reset_gpio success\n");
	}
}

int gf_power_on(struct gf_dev *gf_dev)
{
	int rc = 0;

	gpio_direction_output(gf_dev->pwr_gpio,1);
    
    //mdelay(5);
    msleep(5);
    gpio_set_value(gf_dev->reset_gpio, 1);
    
    if (!gf_dev->power_enabled) {
		gf_dev->power_enabled = 1;
		/* TODO: add your power control here */
	}
	

	pr_err("gf_power_on end, vendor is %d\n", gpio_get_value(gf_dev->vendorId_gpio));
	
	return rc;
}

int gf_power_off(struct gf_dev *gf_dev)
{
	int rc = 0;
	if (!gf_dev) {
		pr_err("Input buff is NULL.\n");
		return -ENODEV;
	}

    gpio_direction_output(gf_dev->pwr_gpio,0);
    
	if (gf_dev->power_enabled) {
		gf_dev->power_enabled = 0;
		/* TODO: add your power control here */
	}
	return rc;
}

int gf_hw_reset(struct gf_dev *gf_dev, unsigned int delay_ms)
{
	if (!gf_dev) {
		pr_err("Input buff is NULL.\n");
		return -ENODEV;
	}
	
	gpio_direction_output(gf_dev->reset_gpio, 1);
	gpio_set_value(gf_dev->reset_gpio, 0);
	mdelay(3);
	gpio_set_value(gf_dev->reset_gpio, 1);
	mdelay(delay_ms);
	return 0;
}

int gf_irq_num(struct gf_dev *gf_dev)
{
	if (!gf_dev) {
		pr_err("Input buff is NULL.\n");
		return -ENODEV;
	} else {
		return gpio_to_irq(gf_dev->irq_gpio);
	}
}

