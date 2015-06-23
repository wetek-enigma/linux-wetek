/*
 * Wetek NIMs/DVB detection
 *
 * Copyright (C) 2014 Sasa Savic <sasa.savic.sr@gmail.com>
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License along
 *    with this program; if not, write to the Free Software Foundation, Inc.,
 *    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */



#include <linux/amlogic/aml_gpio_consumer.h>
#include <linux/usb.h>
#include "nimdetect.h"

#include "cxd2837.h"
#include "mxl603.h"
#include "avl6211.h"
#include "mn88436.h"


#define TOTAL_I2C		 	2
#define TOTAL_DEMODS 		2
#define TOTAL_AML_INPUTS 	3


static struct wetek_nims weteknims;

static struct cxd2837_cfg cxd2837cfg = {
		.adr = 0x6C,
		.if_agc_polarity = 1,
		.rfain_monitoring = 0,
		.ts_error_polarity = 0,
		.clock_polarity = 1,
		.ifagc_adc_range = 0,
		.spec_inv = 0,
		.xtal = XTAL_20500KHz,
		.ts_clock = SERIAL_TS_CLK_MID_FULL,
};

static struct mxl603_config mxl603cfg = {
		.xtal_freq_hz = MXL603_XTAL_24MHz,
		.if_freq_hz = MXL603_IF_5MHz,
		.agc_type = MXL603_AGC_SELF,
		.xtal_cap = 16,
		.gain_level = 11,
		.if_out_gain_level = 11,
		.agc_set_point = 66,
		.agc_invert_pol = 0,
		.invert_if = 1,
		.loop_thru_enable = 0,
		.clk_out_enable = 1,
		.clk_out_div = 0,
		.clk_out_ext = 0,
		.xtal_sharing_mode = 0,
		.single_supply_3_3V = 1,
};

static struct mxl603_config mxl603cfg_atsc = {
		.xtal_freq_hz = MXL603_XTAL_24MHz,
		.if_freq_hz = MXL603_IF_5MHz,
		.agc_type = MXL603_AGC_EXTERNAL,
		.xtal_cap = 31,
		.gain_level = 11,
		.if_out_gain_level = 11,
		.agc_set_point = 66,
		.agc_invert_pol = 0,
		.invert_if = 0,
		.loop_thru_enable = 0,
		.clk_out_enable = 1,
		.clk_out_div = 0,
		.clk_out_ext = 0,
		.xtal_sharing_mode = 0,
		.single_supply_3_3V = 1,
};

static struct avl6211_config avl6211cfg[] = {
	{
		.tuner_address = 0xC2,
		.tuner_i2c_clock = 200,	
		.demod_address = 0x0C,
		.mpeg_pol = 1,
		.mpeg_mode = 0,
		.mpeg_format = 0,
		.demod_refclk = 9,
		.mpeg_pin = 0,
		.tuner_rfagc = 1,
		.tuner_spectrum = 0,
		.use_lnb_pin59 = 1,
		.use_lnb_pin60 = 0,
		.set_external_vol_gpio = set_external_vol_gpio,
	},
	{
		.tuner_address = 0xC2,
		.tuner_i2c_clock = 200,		
		.demod_address = 0x0C,
		.mpeg_pol = 1,
		.mpeg_mode = 0,
		.mpeg_format = 0,
		.demod_refclk = 9,
		.mpeg_pin = 0,
		.tuner_rfagc = 1,
		.tuner_spectrum = 0,
		.use_lnb_pin59 = 1,
		.use_lnb_pin60 = 0,
		.set_external_vol_gpio = set_external_vol_gpio,
	}
};

extern struct list_head usb_bus_list;
extern struct mutex usb_bus_list_lock;
extern struct usb_device *usb_hub_find_child(struct usb_device *hdev, int port1);
extern struct usb_device *usb_get_dev(struct usb_device *dev);
extern int usb_control_msg(struct usb_device *dev, unsigned int pipe, __u8 request,
                     __u8 requesttype, __u16 value, __u16 index, void *data,
                     __u16 size, int timeout);
extern const unsigned long cpu_bit_bitmap[BITS_PER_LONG+1][BITS_TO_LONGS(NR_CPUS)];
extern int __irq_set_affinity(unsigned int irq, const struct cpumask *mask, bool force);

EXPORT_SYMBOL(usb_bus_list);
EXPORT_SYMBOL(usb_bus_list_lock);
EXPORT_SYMBOL(usb_hub_find_child);
EXPORT_SYMBOL(usb_get_dev);
EXPORT_SYMBOL(usb_control_msg);
EXPORT_SYMBOL(get_nims_infos);
EXPORT_SYMBOL(__irq_set_affinity);



const struct cpumask *aml_get_cpu_mask(unsigned int cpu)
{
	const unsigned long *p = cpu_bit_bitmap[1 + cpu % BITS_PER_LONG];
	p -= cpu / BITS_PER_LONG;
	return to_cpumask(p);
}
EXPORT_SYMBOL(aml_get_cpu_mask);

void get_nims_infos(struct wetek_nims *p)
{
	memcpy(p, &weteknims, sizeof(struct wetek_nims));
}

int set_external_vol_gpio(int *demod_id, int on)
{
	if (on) {
		if (*demod_id == 0 ) 		
			amlogic_gpio_direction_output(GPIOAO_8, 1, "nimdetect");
        else if (*demod_id == 1)
			amlogic_gpio_direction_output(GPIOAO_9, 1, "nimdetect");
	} else if (!on) {
		if (*demod_id == 0 ) 
			amlogic_gpio_direction_output(GPIOAO_8, 0, "nimdetect");
        else if (*demod_id == 1)		
			amlogic_gpio_direction_output(GPIOAO_9, 0, "nimdetect");               
    }
	return 0;
}

static void nim_dvb_pinctrl_put(struct wetek_nims *p)
{
	if (p->ts[0].pinctrl) {
		devm_pinctrl_put(p->ts[0].pinctrl);
		p->ts[0].pinctrl = NULL;
	}
}
static struct pinctrl * __must_check nim_dvb_pinctrl_get_select(
		struct device *dev, struct wetek_nims *p, const char *name)
{
	/*all dvb pinctrls share the ts[0]'s pinctrl.*/
	struct pinctrl *pctl = p->ts[0].pinctrl;
	
	struct pinctrl_state *s;
	int ret;

	if (!pctl) {
		pctl = devm_pinctrl_get(dev);
		if (IS_ERR(pctl))
			return pctl;
			
		p->ts[0].pinctrl = pctl;
	}

	s = pinctrl_lookup_state(pctl, name);
	if (IS_ERR(s)) {
		devm_pinctrl_put(pctl);
		return ERR_CAST(s);
	}

	ret = pinctrl_select_state(pctl, s);
	if (ret < 0) {
		devm_pinctrl_put(pctl);
		return ERR_PTR(ret);
	}

	return pctl;
}
static int nim_dvb_probe(struct platform_device *pdev)
{
	int i;
	int ret = 0;

	weteknims.pdev = pdev;
	weteknims.dev  = &pdev->dev;

	for (i = 0; i < TOTAL_I2C; i++) {
		weteknims.i2c[i] = i2c_get_adapter(i + 1);
		if (weteknims.i2c[i] != NULL)
			dev_info(&pdev->dev, "Found Wetek i2c-%d adapter ...\n", i + 1);
		else {
			dev_info(&pdev->dev, "Failed to acquire Wetek i2c-%d adapter ...\n", i + 1);	
			return 0;
		}
	
	}
	if (pdev->dev.of_node) {
		for (i = 0; i <  TOTAL_AML_INPUTS; i++) {
			char buf[32];
			const char *str;

			snprintf(buf, sizeof(buf), "ts%d", i);
			ret = of_property_read_string(pdev->dev.of_node, buf, &str);
			if (!ret) {
				if (!strcmp(str, "parallel")) {
					dev_info(&pdev->dev, "%s: parallel\n", buf);
					snprintf(buf, sizeof(buf), "p_ts%d", i);
					weteknims.ts[i].mode    = 1;
					weteknims.ts[i].pinctrl = nim_dvb_pinctrl_get_select(&pdev->dev, &weteknims, buf);
				}
			}
		}
	}

	/* FEC_RESET */
	amlogic_gpio_request(GPIOD_8, "nimdetect"); 
	
	/* INPUT1 POWER CTRL */
	amlogic_gpio_request(GPIOAO_8, "nimdetect");
	
	/* INPUT2 POWER CTRL */
	amlogic_gpio_request(GPIOAO_9, "nimdetect");

	amlogic_gpio_direction_output(GPIOAO_8, 0, "nimdetect"); //SWITCH OFF INPUT1 POWER		
	amlogic_gpio_direction_output(GPIOAO_9, 0, "nimdetect"); //SWITCH OFF INPUT2 POWER		

	/* RESET DEMOD(s) */
	amlogic_gpio_direction_output(GPIOD_8, 0, "nimdetect");
	msleep(600);
	amlogic_gpio_direction_output(GPIOD_8, 1, "nimdetect");
	msleep(200);


	dev_info(&pdev->dev, "Wetek NIM(s) detection in progress ...\n");

	for (i = 0; i < TOTAL_DEMODS; i++) {		
		if (i == 0) {		
			dev_info(&pdev->dev, "Checking for Sony CXD2837 DVB-C/T/T2 demod ...\n");	
			
			weteknims.fe[i] =  dvb_attach(cxd2837_attach, weteknims.i2c[i], &cxd2837cfg);				
			
			if (weteknims.fe[i] != NULL) {			
															
				if (dvb_attach(mxl603_attach, weteknims.fe[i], weteknims.i2c[i], 0x60, &mxl603cfg) == NULL) {
					dev_info(&pdev->dev, "Failed to find MxL603 tuner!\n");
					dev_info(&pdev->dev, "Detaching Sony CXD2837 DVB-C/T/T2 frontend!\n");
					dvb_frontend_detach(weteknims.fe[i]);
					return 0;
				}
				
				weteknims.total_nims++;
				dev_info(&pdev->dev, "Total Wetek NIM(s) found: %d\n", weteknims.total_nims);
				return 0;
			}
			
			dev_info(&pdev->dev, "Checking for Panasonic MN88436 ATSC demod ...\n");	
			
			weteknims.fe[i] =  dvb_attach(mn88436_attach, weteknims.i2c[i], 0);				
			
			if (weteknims.fe[i] != NULL) {			
												
				if (dvb_attach(mxl603_attach, weteknims.fe[i], weteknims.i2c[i], 0x60, &mxl603cfg_atsc) == NULL) {
					dev_info(&pdev->dev, "Failed to find MxL603 tuner!\n");
					dev_info(&pdev->dev, "Detaching Panasonic MN88436 ATSC frontend!\n");
					dvb_frontend_detach(weteknims.fe[i]);
					return 0;
				}
				
				weteknims.total_nims++;
				dev_info(&pdev->dev, "Total Wetek NIM(s) found: %d\n", weteknims.total_nims);
				return 0;
			}			
		}
		dev_info(&pdev->dev, "Checking for AVL6211 DVB-S/S2 demod ...\n");
		weteknims.fe[i] = dvb_attach(avl6211_attach, weteknims.i2c[i], &avl6211cfg[i], i);
		if (i == 0 && weteknims.fe[i] == NULL) {
			dev_info(&pdev->dev, "No available NIM(s) found ...\n");
			return 0;			
		}
		if (weteknims.fe[i] != NULL)
			weteknims.total_nims++;		
	}
	
	if (weteknims.total_nims > 0)
		dev_info(&pdev->dev, "Total Wetek NIM(s) found: %d\n", weteknims.total_nims);
	
	return 0;
}
static int nim_dvb_remove(struct platform_device *pdev)
{
	int i;
	
	for (i = 0; i < TOTAL_DEMODS; i++) {
		if (weteknims.fe[i] != NULL) 
			dvb_frontend_detach(weteknims.fe[i]);
	}

	for (i = 0; i < TOTAL_I2C; i++) {
		if (weteknims.i2c[i] != NULL)
			i2c_put_adapter(weteknims.i2c[i]);
	}
	amlogic_gpio_free(GPIOD_8, "nimdetect");
	amlogic_gpio_free(GPIOAO_8, "nimdetect");
	amlogic_gpio_free(GPIOAO_9, "nimdetect");
	nim_dvb_pinctrl_put(&weteknims);
	return 0;
}
static int wetekcard_probe(struct platform_device *pdev)
{
	if (pdev->dev.of_node)
		weteknims.card_pinctrl = devm_pinctrl_get_select_default(&pdev->dev);

	return 0;
}
static int wetekcard_remove(struct platform_device *pdev)
{
	if(weteknims.card_pinctrl)
		devm_pinctrl_put(weteknims.card_pinctrl);
	return 0;
}
static const struct of_device_id nim_dvb_dt_match[] = {
	{
		.compatible = "amlogic,dvb",
	},
	{},
};
static struct platform_driver nim_dvb_detection = {
	.probe		= nim_dvb_probe,
	.remove		= nim_dvb_remove,
	.driver		= {
		.name	= "wetek-dvb",
		.owner	= THIS_MODULE,
		.of_match_table = nim_dvb_dt_match,
	}
};

static const struct of_device_id wetekcard_dt_match[]={
	{	.compatible = "amlogic,smartcard",
	},
	{},
};

static struct platform_driver wetekcard_driver = {
	.probe		= wetekcard_probe,
    .remove		= wetekcard_remove,
	.driver		= {
		.name	= "wetek-card",
		.owner	= THIS_MODULE,
		.of_match_table = wetekcard_dt_match,
	}
};

int __init nim_dvb_init(void)
{
	int ret;
	
	memset(&weteknims, 0, sizeof(struct wetek_nims));
	
	ret = platform_driver_register(&nim_dvb_detection);
	if (!ret)
		return platform_driver_register(&wetekcard_driver);
		
	return ret;
}
void __exit nim_dvb_exit(void)
{
	platform_driver_unregister(&nim_dvb_detection);
	platform_driver_unregister(&wetekcard_driver);
}

module_init(nim_dvb_init);
module_exit(nim_dvb_exit);

MODULE_DESCRIPTION("Wetek NIMs DVB detection");
MODULE_AUTHOR("Sasa Savic <sasa.savic.sr@gmail.com>");
MODULE_LICENSE("GPL");
