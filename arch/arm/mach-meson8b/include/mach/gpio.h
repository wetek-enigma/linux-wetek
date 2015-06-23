#ifndef __ARCH_MACH_GPIO_H__
#define __ARCH_MACH_GPIO_H__
typedef enum {
	GPIOAO_0=0,
	GPIOAO_1=1,
	GPIOAO_2=2,
	GPIOAO_3=3,
	GPIOAO_4=4,
	GPIOAO_5=5,
	GPIOAO_6=6,
	GPIOAO_7=7,
	GPIOAO_8=8,
	GPIOAO_9=9,
	GPIOAO_10=10,
	GPIOAO_11=11,
	GPIOAO_12=12,
	GPIOAO_13=13,
	GPIOH_0=14,
	GPIOH_1=15,
	GPIOH_2=16,
	GPIOH_3=17,
	GPIOH_4=18,
	GPIOH_5=19,
	GPIOH_6=20,
	GPIOH_7=21,
	GPIOH_8=22,
	GPIOH_9=23,
	BOOT_0=24,
	BOOT_1=25,
	BOOT_2=26,
	BOOT_3=27,
	BOOT_4=28,
	BOOT_5=29,
	BOOT_6=30,
	BOOT_7=31,
	BOOT_8=32,
	BOOT_9=33,
	BOOT_10=34,
	BOOT_11=35,
	BOOT_12=36,
	BOOT_13=37,
	BOOT_14=38,
	BOOT_15=39,
	BOOT_16=40,
	BOOT_17=41,
	BOOT_18=42,
	CARD_0=43,
	CARD_1=44,
	CARD_2=45,
	CARD_3=46,
	CARD_4=47,
	CARD_5=48,
	CARD_6=49,
	GPIODV_0=50,
	GPIODV_1=51,
	GPIODV_2=52,
	GPIODV_3=53,
	GPIODV_4=54,
	GPIODV_5=55,
	GPIODV_6=56,
	GPIODV_7=57,
	GPIODV_8=58,
	GPIODV_9=59,
	GPIODV_10=60,
	GPIODV_11=61,
	GPIODV_12=62,
	GPIODV_13=63,
	GPIODV_14=64,
	GPIODV_15=65,
	GPIODV_16=66,
	GPIODV_17=67,
	GPIODV_18=68,
	GPIODV_19=69,
	GPIODV_20=70,
	GPIODV_21=71,
	GPIODV_22=72,
	GPIODV_23=73,
	GPIODV_24=74,
	GPIODV_25=75,
	GPIODV_26=76,
	GPIODV_27=77,
	GPIODV_28=78,
	GPIODV_29=79,
	GPIOY_0=80,
	GPIOY_1=81,
	GPIOY_2=82,
	GPIOY_3=83,
	GPIOY_4=84,
	GPIOY_5=85,
	GPIOY_6=86,
	GPIOY_7=87,
	GPIOY_8=88,
	GPIOY_9=89,
	GPIOY_10=90,
	GPIOY_11=91,
	GPIOY_12=92,
	GPIOY_13=93,
	GPIOY_14=94,
	GPIOY_15=95,
	GPIOY_16=96,
	GPIOX_0=97,
	GPIOX_1=98,
	GPIOX_2=99,
	GPIOX_3=100,
	GPIOX_4=101,
	GPIOX_5=102,
	GPIOX_6=103,
	GPIOX_7=104,
	GPIOX_8=105,
	GPIOX_9=106,
	GPIOX_10=107,
	GPIOX_11=108,
	GPIOX_12=109,
	GPIOX_13=110,
	GPIOX_14=111,
	GPIOX_15=112,
	GPIOX_16=113,
	GPIOX_17=114,
	GPIOX_18=115,
	GPIOX_19=116,
	GPIOX_20=117,
	GPIOX_21=118,
	DIF_TTL_0_P=119,
	DIF_TTL_0_N=120,
	DIF_TTL_1_P=121,
	DIF_TTL_1_N=122,
	DIF_TTL_2_P=123,
	DIF_TTL_2_N=124,
	DIF_TTL_3_P=125,
	DIF_TTL_3_N=126,
	DIF_TTL_4_P=127,
	DIF_TTL_4_N=128,
	HDMI_TTL_0_P=129,
	HDMI_TTL_0_N=130,
	HDMI_TTL_1_P=131,
	HDMI_TTL_1_N=132,
	HDMI_TTL_2_P=133,
	HDMI_TTL_2_N=134,
	HDMI_TTL_CK_P=135,
	HDMI_TTL_CK_N=136,
	GPIO_BSD_EN=137,
	GPIO_TEST_N=138,
	GPIO_MAX=139,
}gpio_t;

enum {
	GPIO_IRQ0=0,
	GPIO_IRQ1,
	GPIO_IRQ2,
	GPIO_IRQ3,
	GPIO_IRQ4,
	GPIO_IRQ5,
	GPIO_IRQ6,
	GPIO_IRQ7,
};

enum {
	GPIO_IRQ_HIGH=0,
	GPIO_IRQ_LOW,
	GPIO_IRQ_RISING,
	GPIO_IRQ_FALLING,
};

enum {
	FILTER_NUM0=0,
	FILTER_NUM1,
	FILTER_NUM2,
	FILTER_NUM3,
	FILTER_NUM4,
	FILTER_NUM5,
	FILTER_NUM6,
	FILTER_NUM7,
};
#endif
