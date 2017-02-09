/*
 * Code for Variscite AM335X SOM.
 *
 * Copyright (C) 2012 Variscite, Ltd. - http://www.variscite.com/
 *
 * Copyright (C) 2011 Texas Instruments, Inc. - http://www.ti.com/
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation version 2.
 *
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/phy.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/nand.h>
#include <linux/micrel_phy.h>
#include <linux/mtd/partitions.h>
#include <linux/platform_device.h>
#include <linux/platform_data/ti_adc.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/wl12xx.h>
#include <linux/ethtool.h>
#include <linux/mfd/tps65910.h>
#include <linux/pwm_backlight.h>
#include <linux/reboot.h>
#include <linux/pwm/pwm.h>
#include <linux/ti_wilink_st.h>
#include <linux/input/ft5x06.h>
#include <linux/mfd/ti_tscadc.h>

#include <video/da8xx-fb.h>
#include <mach/hardware.h>
#include <mach/board-am335xevm.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/hardware/asp.h>

#include <plat/irqs.h>
#include <plat/board.h>
#include <plat/common.h>
#include <plat/lcdc.h>
#include <plat/usb.h>
#include <plat/mmc.h>
#include <plat/emif.h>
#include <plat/nand.h>
#include <plat/omap-pm.h>
#include <plat/omap_device.h>

#include "common.h"
#include "board-flash.h"
#include "cpuidle33xx.h"
#include "mux.h"
#include "devices.h"
#include "hsmmc.h"


/* module pin mux structure */
struct pinmux_config {
	const char *string_name; /* signal name format */
	int val; /* Options for the mux register value */
};

/* Convert GPIO signal to GPIO pin number */
#define GPIO_TO_PIN(bank, gpio) (32 * (bank) + (gpio))

#define NO_OF_MAC_ADDR		3

/* VAR-SOM-AM33 module specific gpios */
#define VAR_SOM_GMII1_RST_GPIO          GPIO_TO_PIN(2, 19)
#define VAR_SOM_REV_BIT0                GPIO_TO_PIN(2, 13)
#define VAR_SOM_REV_BIT1                GPIO_TO_PIN(2, 22)
#define VAR_SOM_REV_BIT2                GPIO_TO_PIN(2, 11)
#define VAR_SOM_WLAN_PMENA_GPIO         GPIO_TO_PIN(3, 21)
#define VAR_SOM_WLAN_IRQ_GPIO           GPIO_TO_PIN(3, 20)
#define VAR_SOM_BT_PMENA_GPIO           GPIO_TO_PIN(3, 9)

/* Board specific gpios */
#define BOARD_LCD_BACKLIGHT_GPIO        GPIO_TO_PIN(1, 21)
#define BOARD_FT5X06_IRQ_GPIO           GPIO_TO_PIN(1, 17)
#define BOARD_FT5X06_RESET_GPIO         GPIO_TO_PIN(2, 0)
#define BOARD_A9_NOTIFY                 GPIO_TO_PIN(0, 3)

static struct spi_board_info spidevices[] = {
    {
        .modalias = "spidev",
        .chip_select = 1,
        .max_speed_hz = 1 * 1000 * 1000,
        .bus_num = 1,
    },
    {
        .modalias = "spidev",
        .chip_select = 0,
        .max_speed_hz = 1 * 1000 * 1000,
        .bus_num = 2,
    },
};


static char am335x_mac_addr[NO_OF_MAC_ADDR][ETH_ALEN];

/* LCD */
static const struct display_panel disp_panel = {
	WVGA,
	32,
	16,
	COLOR_PASSIVE,
};

static struct lcd_ctrl_config lcd_cfg = {
	&disp_panel,
	.ac_bias		= 255,
	.ac_bias_intrpt		= 0,
	.dma_burst_sz		= 16,
	.bpp			= 32,
	.fdd			= 0x80,
	.tft_alt_mode		= 0,
	.stn_565_mode		= 1,
	.mono_8bit_mode		= 0,
	.invert_line_clock	= 0,
	.invert_frm_clock	= 0,
	.sync_edge		= 1,
	.sync_ctrl		= 1,
	.raster_order		= 0,
};

struct da8xx_lcdc_platform_data newhaven_lcd_pdata = {
	.manu_name		= "Newhaven",
	.controller_data	= &lcd_cfg,
	.type			= "NHD-5.0-800480TF-ATXL",
};

static void sgx_init(void);

/* Audio */
static u8 am335x_iis_serializer_direction1[] = {
	INACTIVE_MODE,	INACTIVE_MODE,	RX_MODE,	TX_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,	INACTIVE_MODE,
};

static struct snd_platform_data var_am335x_som_snd_data0 = {
	.tx_dma_offset	= 0x46000000,	/* McASP0 */
	.rx_dma_offset	= 0x46000000,
	.op_mode	= DAVINCI_MCASP_IIS_MODE,
	.num_serializer	= ARRAY_SIZE(am335x_iis_serializer_direction1),
	.tdm_slots	= 2,
	.serial_dir	= am335x_iis_serializer_direction1,
	.asp_chan_q	= EVENTQ_2,
	.version	= MCASP_VERSION_3,
	.txnumevt	= 32,
	.rxnumevt	= 32,
	.get_context_loss_count	=
			omap_pm_get_dev_context_loss_count,
};

/* MMC */
static struct omap2_hsmmc_info am335x_mmc[] __initdata = {
	{
		.mmc            = 1,
		.caps           = MMC_CAP_4_BIT_DATA,
		.gpio_cd        = -EINVAL,
		.gpio_wp        = -EINVAL,
		.ocr_mask       = MMC_VDD_32_33 | MMC_VDD_33_34, /* 3V3 */
	},
	{
		.mmc            = 0,	/* will be set at runtime */
	},
	{
		.mmc            = 0,	/* will be set at runtime */
	},
	{}      /* Terminator */
};


#ifdef CONFIG_OMAP_MUX
static struct omap_board_mux board_mux[] __initdata = {
	AM33XX_MUX(I2C0_SDA, OMAP_MUX_MODE7 | AM33XX_INPUT_EN),
	AM33XX_MUX(I2C0_SCL, OMAP_MUX_MODE7 |AM33XX_INPUT_EN ),
	AM33XX_MUX(SPI0_SCLK, OMAP_MUX_MODE7 | AM33XX_SLEWCTRL_SLOW |
				AM33XX_INPUT_EN | AM33XX_PIN_OUTPUT),
	AM33XX_MUX(EMU1, OMAP_MUX_MODE7 |
				AM33XX_INPUT_EN | AM33XX_PULL_UP),

	{ .reg_offset = OMAP_MUX_TERMINATOR },
};
#else
#define	board_mux	NULL
#endif

/* SOM revision GPIO */
static struct pinmux_config am33_var_som_rev_pin_mux[] = {
	{"lcd_data7.gpio2_13",	OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLDOWN},
	{"lcd_data5.gpio2_11",	OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLDOWN},
	{"lcd_vsync.gpio2_22",	OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLDOWN},

	{NULL, 0},
};

/* UART0 */
static struct pinmux_config uart0_pin_mux[] = {
	{"uart0_rxd.uart0_rxd", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"uart0_txd.uart0_txd", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL},
	{NULL, 0},
};

/* LCD Backlight GPIO */
static struct pinmux_config lcd_backlight_pin_mux[] = {
    {"gpmc_a5.gpio1_21", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* CDI BUS */
static struct pinmux_config cdi_bus_pin_mux[] = {
    {"spi0_d0.gpio0_3", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
    {"mcasp0_ahclkr.spi1_cs0", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
    {"uart0_ctsn.spi1_d0", OMAP_MUX_MODE4 | AM33XX_PIN_INPUT_PULLDOWN},
    {"uart0_rtsn.spi1_d1", OMAP_MUX_MODE4 | AM33XX_PIN_INPUT_PULLDOWN},
    {"ecap0_in_pwm0_out.spi1_sclk", OMAP_MUX_MODE4 | AM33XX_PIN_OUTPUT},
    {NULL, 0}
};

/* Module pin mux for LCDC */
static struct pinmux_config lcdc_pin_mux[] = {
	{"lcd_data0.lcd_data0",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data1.lcd_data1",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data2.lcd_data2",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data3.lcd_data3",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data4.lcd_data4",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data5.lcd_data5",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data6.lcd_data6",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data7.lcd_data7",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data8.lcd_data8",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data9.lcd_data9",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data10.lcd_data10",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data11.lcd_data11",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data12.lcd_data12",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data13.lcd_data13",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data14.lcd_data14",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_data15.lcd_data15",	OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT
						       | AM33XX_PULL_DISA},
	{"lcd_vsync.lcd_vsync",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_hsync.lcd_hsync",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_pclk.lcd_pclk",		OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"lcd_ac_bias_en.lcd_ac_bias_en", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* Pin mux for nand flash module */
static struct pinmux_config nand_pin_mux[] = {
	{"gpmc_ad0.gpmc_ad0",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad1.gpmc_ad1",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad2.gpmc_ad2",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad3.gpmc_ad3",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad4.gpmc_ad4",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad5.gpmc_ad5",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad6.gpmc_ad6",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad7.gpmc_ad7",	  OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wait0.gpmc_wait0", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_wpn.gpmc_wpn",	  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn0.gpmc_csn0",	  OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_advn_ale.gpmc_advn_ale",  OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_oen_ren.gpmc_oen_ren",	 OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_wen.gpmc_wen",     OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{"gpmc_ben0_cle.gpmc_ben0_cle",	 OMAP_MUX_MODE0 | AM33XX_PULL_DISA},
	{NULL, 0},
};

/* Module pin mux for rmii1 */
static struct pinmux_config rmii1_pin_mux[] = {
	{"mii1_crs.rmii1_crs_dv", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxerr.mii1_rxerr", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_txen.mii1_txen", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_txd1.mii1_txd1", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_txd0.mii1_txd0", OMAP_MUX_MODE1 | AM33XX_PIN_OUTPUT},
	{"mii1_rxd1.mii1_rxd1", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mii1_rxd0.mii1_rxd0", OMAP_MUX_MODE1 | AM33XX_PIN_INPUT_PULLDOWN},
	{"rmii1_refclk.rmii1_refclk", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mdio_data.mdio_data", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mdio_clk.mdio_clk", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT_PULLUP},
	{NULL, 0},
};

/* I2C1 */
static struct pinmux_config i2c1_pin_mux[] = {
	{"spi0_d1.i2c1_sda",    OMAP_MUX_MODE2 | AM33XX_SLEWCTRL_SLOW |
					AM33XX_PULL_ENBL | AM33XX_INPUT_EN},
	{"spi0_cs0.i2c1_scl",   OMAP_MUX_MODE2 | AM33XX_SLEWCTRL_SLOW |
					AM33XX_PULL_ENBL | AM33XX_INPUT_EN},
	{NULL, 0},
};

/* Module pin mux for mcasp0 */
static struct pinmux_config mcasp0_pin_mux[] = {
	{"mcasp0_aclkx.mcasp0_aclkx", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mcasp0_fsx.mcasp0_fsx", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mcasp0_aclkr.mcasp0_axr2", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLDOWN},
	{"mcasp0_fsr.mcasp0_axr3", OMAP_MUX_MODE2 |	AM33XX_PIN_INPUT_PULLDOWN},
	{NULL, 0},
};

/* Module pin mux for mmc0 */
static struct pinmux_config mmc0_pin_mux[] = {
	{"mmc0_dat3.mmc0_dat3",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat2.mmc0_dat2",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat1.mmc0_dat1",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_dat0.mmc0_dat0",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_clk.mmc0_clk",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"mmc0_cmd.mmc0_cmd",	OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	/*{"mcasp0_aclkr.mmc0_sdwp", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},*/
	{"gpmc_ben1.gpio1_28",  OMAP_MUX_MODE7 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

/* Module pin mux for wlan and bluetooth */
static struct pinmux_config mmc1_wl12xx_pin_mux[] = {
	{"gpmc_ad8.mmc1_dat0", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad9.mmc1_dat1", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad10.mmc1_dat2", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_ad11.mmc1_dat3", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn2.mmc1_cmd", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{"gpmc_csn1.mmc1_clk", OMAP_MUX_MODE2 | AM33XX_PIN_INPUT_PULLUP},
	{NULL, 0},
};

static struct pinmux_config uart1_wl12xx_pin_mux[] = {
	{"uart1_ctsn.uart1_ctsn", OMAP_MUX_MODE0 | AM33XX_PIN_OUTPUT},
	{"uart1_rtsn.uart1_rtsn", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"uart1_rxd.uart1_rxd", OMAP_MUX_MODE0 | AM33XX_PIN_INPUT_PULLUP},
	{"uart1_txd.uart1_txd", OMAP_MUX_MODE0 | AM33XX_PULL_ENBL},
	{NULL, 0},
};

static struct pinmux_config wl12xx_pin_mux_var_som[] = {
	{"mcasp0_axr1.gpio3_20", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
	{"mcasp0_ahclkx.gpio3_21", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{"mii1_txclk.gpio3_9", OMAP_MUX_MODE7 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* Enable clkout1 */
static struct pinmux_config clkout1_pin_mux[] = {
	{"xdma_event_intr0.clkout1", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
	{NULL, 0},
};

/* FT5x06 touchscreen GPIO pins */
static struct pinmux_config ft5x06_gpio_touchscreen_mux[] = {
    {"gpmc_csn3.gpio2_0", OMAP_MUX_MODE3 | AM33XX_PIN_OUTPUT},
    {"gpmc_a1.gpio1_17", OMAP_MUX_MODE7 | AM33XX_PIN_INPUT},
    {NULL, 0}
};

static struct gpio som_rev_gpios[] __initdata = {
	{VAR_SOM_REV_BIT0, GPIOF_IN,	"som_rev_bit_0"},
	{VAR_SOM_REV_BIT1, GPIOF_IN,	"som_rev_bit_1"},
	{VAR_SOM_REV_BIT2, GPIOF_IN,	"som_rev_bit_2"},

};

static struct ft5x06_platform_data  board_ft5x06_platform_data =
{
    .irq_pin = BOARD_FT5X06_IRQ_GPIO,
    .reset_pin = BOARD_FT5X06_RESET_GPIO,
    .abs_x = 800,
    .abs_y = 480
};

/*
* @pin_mux - single module pin-mux structure which defines pin-mux
*			details for all its pins.
*/
static void setup_pin_mux(struct pinmux_config *pin_mux)
{
	int i;

	for (i = 0; pin_mux->string_name != NULL; pin_mux++)
		omap_mux_init_signal(pin_mux->string_name, pin_mux->val);
}

int get_var_am33_som_rev(void)
{
	static int som_rev = -1;
	int subversion;

	if (som_rev == (-1)) {
		int status;

		setup_pin_mux(am33_var_som_rev_pin_mux);

		status = gpio_request_array(som_rev_gpios,
				ARRAY_SIZE(som_rev_gpios));
		if (status) {
			pr_err("Error requesting som rev gpio: %d\n", status);
		}
		subversion = gpio_get_value(VAR_SOM_REV_BIT2) ? 0 : 1;
		som_rev = (gpio_get_value(VAR_SOM_REV_BIT0) |
				(gpio_get_value(VAR_SOM_REV_BIT1) << 1)) +
			subversion;

		gpio_free_array(som_rev_gpios,
				ARRAY_SIZE(som_rev_gpios));

	}

	return som_rev;
}

const char *get_var_am33_som_rev_str(void)
{
	static char som_rev_str[32];

	sprintf(som_rev_str, "1.%d", get_var_am33_som_rev());

	return som_rev_str;
}

static struct wl12xx_platform_data var_som_am33_wlan_data = {
	.irq = OMAP_GPIO_IRQ(VAR_SOM_WLAN_IRQ_GPIO),
	.wlan_enable_gpio = VAR_SOM_WLAN_PMENA_GPIO,
	.bt_enable_gpio = VAR_SOM_BT_PMENA_GPIO,
	.board_ref_clock = WL12XX_REFCLOCK_26, /* 26Mhz */
};


static int __init backlight_init(void)
{
	int status;

	setup_pin_mux(lcd_backlight_pin_mux);

	status = gpio_request_one(BOARD_LCD_BACKLIGHT_GPIO,
		GPIOF_OUT_INIT_LOW, "backlight");
	
    if (status < 0)
		pr_err("Failed to request gpio for backlight");

	gpio_export(BOARD_LCD_BACKLIGHT_GPIO, 0);

	return 0;
}
late_initcall(backlight_init);

static int __init conf_disp_pll(int rate)
{
	struct clk *disp_pll;
	int ret = -EINVAL;

	disp_pll = clk_get(NULL, "dpll_disp_ck");
	if (IS_ERR(disp_pll)) {
		pr_err("Cannot clk_get disp_pll\n");
		goto out;
	}

	ret = clk_set_rate(disp_pll, rate);
	clk_put(disp_pll);
out:
	return ret;
}

static void lcdc_init(void)
{
	struct da8xx_lcdc_platform_data* plcdc_data;

	plcdc_data = &newhaven_lcd_pdata;

	setup_pin_mux(lcdc_pin_mux);

	if (conf_disp_pll(300000000)) {
		pr_info("Failed configure display PLL, not attempting to"
				"register LCDC\n");
		return;
	}

	plcdc_data->get_context_loss_count = omap_pm_get_dev_context_loss_count;

	if (am33xx_register_lcdc(plcdc_data))
		pr_info("Failed to register LCDC device\n");

	return;
}

static void uart_init(void)
{
	setup_pin_mux(uart0_pin_mux);

	omap_serial_init();
}


static void rmii1_init(void)
{
	int status;

	omap_mux_init_gpio(VAR_SOM_GMII1_RST_GPIO, OMAP_PIN_OUTPUT);

	status = gpio_request_one(VAR_SOM_GMII1_RST_GPIO,
		GPIOF_OUT_INIT_HIGH, "rmii1_rst");
	if (status) {
		pr_err("Error requesting rmii1 reset gpio: %d\n", status);
	}

	/* Reset RMII1 */
	mdelay(10);
	gpio_set_value(VAR_SOM_GMII1_RST_GPIO, 0);
	mdelay(10);
	gpio_set_value(VAR_SOM_GMII1_RST_GPIO, 1);

	gpio_export(VAR_SOM_GMII1_RST_GPIO, 0);

	setup_pin_mux(rmii1_pin_mux);

	return;
}

/* NAND partition information
 */
static struct mtd_partition am335x_nand_partitions[] = {
/* All the partition sizes are listed in terms of NAND block size */
	{
		.name           = "SPL",
		.offset         = 0,			/* Offset = 0x0 */
		.size           = SZ_128K,
	},
	{
		.name           = "SPL.backup1",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x20000 */
		.size           = SZ_128K,
	},
	{
		.name           = "SPL.backup2",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x40000 */
		.size           = SZ_128K,
	},
	{
		.name           = "SPL.backup3",
		.offset         = MTDPART_OFS_APPEND,	/* Offset = 0x60000 */
		.size           = SZ_128K,
	},
	{
		.name           = "U-Boot",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x80000 */
		.size           = 15 * SZ_128K,
	},
	{
		.name           = "U-Boot Env",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x260000 */
		.size           = 1 * SZ_128K,
	},
	{
		.name           = "Kernel",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x280000 */
		.size           = 40 * SZ_128K,
	},
	{
		.name           = "File System",
		.offset         = MTDPART_OFS_APPEND,   /* Offset = 0x780000 */
		.size           = MTDPART_SIZ_FULL,
	},
};

static struct gpmc_timings am335x_nand_timings = {
	.sync_clk = 0,

	.cs_on = 0,
	.cs_rd_off = 44,
	.cs_wr_off = 44,

	.adv_on = 6,
	.adv_rd_off = 34,
	.adv_wr_off = 44,
	.we_off = 40,
	.oe_off = 54,

	.access = 64,
	.rd_cycle = 82,
	.wr_cycle = 82,

	.wr_access = 40,
	.wr_data_mux_bus = 0,
};

static void som_nand_init(void)
{
	struct omap_nand_platform_data *pdata;
	struct gpmc_devices_info gpmc_device[2] = {
		{ NULL, 0 },
		{ NULL, 0 },
	};

	setup_pin_mux(nand_pin_mux);
	pdata = omap_nand_init(am335x_nand_partitions,
		ARRAY_SIZE(am335x_nand_partitions), 0, 0,
		&am335x_nand_timings);
	if (!pdata)
		return;
	pdata->ecc_opt =OMAP_ECC_BCH8_CODE_HW;
	pdata->elm_used = true;
	gpmc_device[0].pdata = pdata;
	gpmc_device[0].flag = GPMC_DEVICE_NAND;

	omap_init_gpmc(gpmc_device, sizeof(gpmc_device));
	omap_init_elm();
}

/* Setup McASP 0 */
static void mcasp0_init(void)
{
	/* Configure McASP */
	setup_pin_mux(mcasp0_pin_mux);
	am335x_register_mcasp(&var_am335x_som_snd_data0, 0);
	return;
}

/* WLAN */
static void mmc1_wl12xx_init(void)
{
	setup_pin_mux(mmc1_wl12xx_pin_mux);

	am335x_mmc[1].mmc = 2;
	am335x_mmc[1].name = "wl1271";
	am335x_mmc[1].caps = MMC_CAP_4_BIT_DATA | MMC_CAP_POWER_OFF_CARD
				| MMC_PM_KEEP_POWER;
	am335x_mmc[1].nonremovable = true;
	am335x_mmc[1].gpio_cd = -EINVAL;
	am335x_mmc[1].gpio_wp = -EINVAL;
	am335x_mmc[1].ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34; /* 3V3 */

	/* mmc will be initialized when mmc0_init is called */
	return;
}

static void uart1_wl12xx_init(void)
{
	setup_pin_mux(uart1_wl12xx_pin_mux);
}

#ifdef CONFIG_TI_ST
/* TI-ST for WL12xx BT */

static int plat_kim_suspend(struct platform_device *pdev, pm_message_t state)
{
	return 0;
}

static int plat_kim_resume(struct platform_device *pdev)
{
	return 0;
}

static int plat_kim_chip_enable(struct kim_data_s *kim_data)
{
	gpio_direction_output(kim_data->nshutdown, 0);
	msleep(1);
	gpio_direction_output(kim_data->nshutdown, 1);

	return 0;
}

static int plat_kim_chip_disable(struct kim_data_s *kim_data)
{
	gpio_direction_output(kim_data->nshutdown, 0);

	return 0;
}

static struct ti_st_plat_data wilink_pdata = {
	.nshutdown_gpio = VAR_SOM_BT_PMENA_GPIO,
	.dev_name = "/dev/ttyO1",
	.flow_cntrl = 1,
	.baud_rate = 3000000,
	.suspend = plat_kim_suspend,
	.resume = plat_kim_resume,
	.chip_enable = plat_kim_chip_enable,
	.chip_disable = plat_kim_chip_disable,
};

static struct platform_device wl12xx_device = {
	.name       = "kim",
	.id     = -1,
	.dev.platform_data = &wilink_pdata,
};

static struct platform_device btwilink_device = {
	.name = "btwilink",
	.id = -1,
};

static inline void __init var_som_init_btwilink(void)
{
	pr_info("var_som_am33: bt init\n");

	platform_device_register(&wl12xx_device);
	platform_device_register(&btwilink_device);
}
#endif

static void wl12xx_bluetooth_enable(void)
{
#ifndef CONFIG_TI_ST
	int status = gpio_request(var_som_am33_wlan_data.bt_enable_gpio,
			"bt_en\n");
	if (status < 0)
		pr_err("Failed to request gpio for bt_enable");

	pr_info("Configure Bluetooth Enable pin...\n");
	gpio_direction_output(var_som_am33_wlan_data.bt_enable_gpio, 0);
#else
	var_som_init_btwilink();
#endif
}

static int wl12xx_set_power(struct device *dev, int slot, int on, int vdd)
{
	if (on) {
		gpio_set_value(var_som_am33_wlan_data.wlan_enable_gpio, 1);
		mdelay(70);
	}
	else
		gpio_set_value(var_som_am33_wlan_data.wlan_enable_gpio, 0);

	return 0;
}

static void wl12xx_init(void)
{
	struct device *dev;
	struct omap_mmc_platform_data *pdata;
	int ret;

	setup_pin_mux(wl12xx_pin_mux_var_som);

	if (get_var_am33_som_rev() >= 3) {
		var_som_am33_wlan_data.board_ref_clock = WL12XX_REFCLOCK_38;
		var_som_am33_wlan_data.board_tcxo_clock = WL12XX_TCXOCLOCK_38_4;
	}

	/* Register WLAN enable pin */
	if (wl12xx_set_platform_data(&var_som_am33_wlan_data))
		pr_err("error setting wl12xx data\n");

	dev = am335x_mmc[1].dev;
	if (!dev) {
		pr_err("wl12xx mmc device initialization failed\n");
		goto out;
	}

	pdata = dev->platform_data;
	if (!pdata) {
		pr_err("Platfrom data of wl12xx device not set\n");
		goto out;
	}

	wl12xx_bluetooth_enable();

	ret = gpio_request_one(var_som_am33_wlan_data.wlan_enable_gpio,
		GPIOF_OUT_INIT_LOW, "wlan_en");
	if (ret) {
		pr_err("Error requesting wlan enable gpio: %d\n", ret);
		goto out;
	}

	pdata->slots[0].set_power = wl12xx_set_power;
out:
	return;
}

static void mmc0_init(void)
{
	setup_pin_mux(mmc0_pin_mux);

	omap2_hsmmc_init(am335x_mmc);
	return;
}

#define VAR_SOM_KS8051_PHY_ID       0x00221556
#define VAR_SOM_KS8051_PHY_MASK     0xffffffff

#define VAR_SOM_KS8081_PHY_ID       0x00221560
#define VAR_SOM_KS8081_PHY_MASK     0xffffffff

#define VAR_SOM_KSZ9021_PHY_ID      0x00221611
#define VAR_SOM_KSZ9021_PHY_MASK    0xffffffff

static int var_som_ks8051_phy_fixup(struct phy_device *phydev)
{
	/* override strap options, set RMII mode */
	phy_write(phydev, 0x16, 0x2);

	return 0;
}

static int var_som_ks8081_phy_fixup(struct phy_device *phydev)
{
	/* override strap options, set RMII mode */
	phy_write(phydev, 0x16, 0x2);

	return 0;
}

static int var_som_ksz9021_phy_fixup(struct phy_device *phydev)
{
	/* Fine-tune clock and control pad skew */
	phy_write(phydev, 0xb, 0x8104);
	phy_write(phydev, 0xc, 0xA097);

	/* Fine-tune RX data pad skew */
	phy_write(phydev, 0xb, 0x8105);
	phy_write(phydev, 0xc, 0);

	return 0;
}

static void ethernet_init(void)
{
	/* Setup fallback MAC addresses, used only if eFuse MACID is invalid.
	 */
	am335x_mac_addr[0][0]=0xF8;
	am335x_mac_addr[0][1]=0xdc;
	am335x_mac_addr[0][2]=0x7a;
	am335x_mac_addr[0][3]=0x00;
	am335x_mac_addr[0][4]=0x11;
	am335x_mac_addr[0][5]=0x22;

	am335x_mac_addr[1][0]=0xF8;
	am335x_mac_addr[1][1]=0xdc;
	am335x_mac_addr[1][2]=0x7a;
	am335x_mac_addr[1][3]=0x00;
	am335x_mac_addr[1][4]=0x11;
	am335x_mac_addr[1][5]=0x23;
	am33xx_cpsw_macidfillup(am335x_mac_addr[0],	am335x_mac_addr[1]);

	phy_register_fixup_for_uid(VAR_SOM_KS8051_PHY_ID, VAR_SOM_KS8051_PHY_MASK,
				   var_som_ks8051_phy_fixup);

	phy_register_fixup_for_uid(VAR_SOM_KS8081_PHY_ID, VAR_SOM_KS8081_PHY_MASK,
				   var_som_ks8081_phy_fixup);

	phy_register_fixup_for_uid(VAR_SOM_KSZ9021_PHY_ID, VAR_SOM_KSZ9021_PHY_MASK,
				   var_som_ksz9021_phy_fixup);

	am33xx_cpsw_init(AM33XX_CPSW_MODE_VAR, "0:01", "0:07");
}

static struct regulator_init_data am335x_dummy = {
	.constraints.always_on	= true,
};

static struct regulator_consumer_supply am335x_vdd1_supply[] = {
	REGULATOR_SUPPLY("vdd_mpu", NULL),
};

static struct regulator_init_data am335x_vdd1 = {
	.constraints = {
		.min_uV			= 600000,
		.max_uV			= 1500000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(am335x_vdd1_supply),
	.consumer_supplies	= am335x_vdd1_supply,
};

static struct regulator_consumer_supply am335x_vdd2_supply[] = {
	REGULATOR_SUPPLY("vdd_core", NULL),
};

static struct regulator_init_data am335x_vdd2 = {
	.constraints = {
		.min_uV			= 600000,
		.max_uV			= 1500000,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE,
		.always_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(am335x_vdd2_supply),
	.consumer_supplies	= am335x_vdd2_supply,
};

static struct tps65910_board am335x_tps65910_info = {
	.tps65910_pmic_init_data[TPS65910_REG_VRTC]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VIO]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDD1]	= &am335x_vdd1,
	.tps65910_pmic_init_data[TPS65910_REG_VDD2]	= &am335x_vdd2,
	.tps65910_pmic_init_data[TPS65910_REG_VDD3]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDIG1]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDIG2]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VPLL]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VDAC]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX1]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX2]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VAUX33]	= &am335x_dummy,
	.tps65910_pmic_init_data[TPS65910_REG_VMMC]	= &am335x_dummy,
};

#define AM33XX_VDD_CORE_OPP50_UV		1100000
#define AM33XX_OPP120_FREQ		600000000
#define AM33XX_OPPTURBO_FREQ		720000000

#define AM33XX_ES2_0_VDD_CORE_OPP50_UV	950000
#define AM33XX_ES2_0_OPP120_FREQ	720000000
#define AM33XX_ES2_0_OPPTURBO_FREQ	800000000
#define AM33XX_ES2_0_OPPNITRO_FREQ	1000000000

#define AM33XX_ES2_1_VDD_CORE_OPP50_UV	950000
#define AM33XX_ES2_1_OPP120_FREQ	720000000
#define AM33XX_ES2_1_OPPTURBO_FREQ	800000000
#define AM33XX_ES2_1_OPPNITRO_FREQ	1000000000

static int am335x_opp_update(void)
{
	u32 rev;
	int voltage_uv = 0;
	struct device *core_dev, *mpu_dev;
	struct regulator *core_reg;

	core_dev = omap_device_get_by_hwmod_name("l3_main");
	mpu_dev = omap_device_get_by_hwmod_name("mpu");

	if (!mpu_dev || !core_dev) {
		pr_err("%s: Aiee.. no mpu/core devices? %p %p\n", __func__,
		       mpu_dev, core_dev);
		return -1;
	}

	core_reg = regulator_get(core_dev, "vdd_core");
	if (IS_ERR(core_reg)) {
		pr_err("%s: unable to get core regulator\n", __func__);
		return -1;
	}

	/*
	 * Ensure physical regulator is present.
	 * (e.g. could be dummy regulator.)
	 */
	voltage_uv = regulator_get_voltage(core_reg);
	if (voltage_uv < 0) {
		pr_err("%s: physical regulator not present for core" \
		       "(%d)\n", __func__, voltage_uv);
		regulator_put(core_reg);
		return -1;
	}

	pr_debug("%s: core regulator value %d\n", __func__, voltage_uv);
	if (voltage_uv > 0) {
		rev = omap_rev();
		switch (rev) {
		case AM335X_REV_ES1_0:
			if (voltage_uv <= AM33XX_VDD_CORE_OPP50_UV) {
				/*
				 * disable the higher freqs - we dont care about
				 * the results
				 */
				opp_disable(mpu_dev, AM33XX_OPP120_FREQ);
				opp_disable(mpu_dev, AM33XX_OPPTURBO_FREQ);
			}
			break;
		case AM335X_REV_ES2_0:
			if (voltage_uv <= AM33XX_ES2_0_VDD_CORE_OPP50_UV) {
				/*
				 * disable the higher freqs - we dont care about
				 * the results
				 */
				opp_disable(mpu_dev,
					    AM33XX_ES2_0_OPP120_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_0_OPPTURBO_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_0_OPPNITRO_FREQ);
			}
			break;
		case AM335X_REV_ES2_1:
		/* FALLTHROUGH */
		default:
			if (voltage_uv <= AM33XX_ES2_1_VDD_CORE_OPP50_UV) {
				/*
				 * disable the higher freqs - we dont care about
				 * the results
				 */
				opp_disable(mpu_dev,
					    AM33XX_ES2_1_OPP120_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_1_OPPTURBO_FREQ);
				opp_disable(mpu_dev,
					    AM33XX_ES2_1_OPPNITRO_FREQ);
			}
			break;
		}
	}

	return 0;
}
late_initcall(am335x_opp_update);


static struct omap_musb_board_data musb_board_data = {
	.interface_type	= MUSB_INTERFACE_ULPI,
	/*
	 * mode[0:3] = USB0PORT's mode
	 * mode[4:7] = USB1PORT's mode
	 */
	.mode           = (MUSB_OTG << 4) | MUSB_HOST,
	.power		= 500,
	.instances	= 1,
};

#define VAR_SOM_TSC_CTW_IRQ_GPIO 	GPIO_TO_PIN(0, 3)

static struct i2c_board_info __initdata var_som_i2c1_boardinfo[] = {
	{
		I2C_BOARD_INFO("tps65910", TPS65910_I2C_ID1),
		.platform_data  = &am335x_tps65910_info,
	},
	{
		I2C_BOARD_INFO("tlv320aic3x", 0x1b),
	},
    {
        I2C_BOARD_INFO("nh_ft5x06", 0x38),
        .platform_data = &board_ft5x06_platform_data,
        .flags = I2C_CLIENT_WAKE,
        .irq = OMAP_GPIO_IRQ(BOARD_FT5X06_IRQ_GPIO)
    }
};

static void i2c1_init(void)
{
	setup_pin_mux(i2c1_pin_mux);

	omap_register_i2c_bus(2, 100, var_som_i2c1_boardinfo,
			ARRAY_SIZE(var_som_i2c1_boardinfo));
	return;
}

static void __init clkout1_enable(void)
{
	setup_pin_mux(clkout1_pin_mux);
}

static struct resource am33xx_cpuidle_resources[] = {
	{
		.start		= AM33XX_EMIF0_BASE,
		.end		= AM33XX_EMIF0_BASE + SZ_32K - 1,
		.flags		= IORESOURCE_MEM,
	},
};

/* AM33XX devices support DDR2 power down */
static struct am33xx_cpuidle_config am33xx_cpuidle_pdata = {
	.ddr2_pdown	= 1,
};

static struct platform_device am33xx_cpuidle_device = {
	.name			= "cpuidle-am33xx",
	.num_resources		= ARRAY_SIZE(am33xx_cpuidle_resources),
	.resource		= am33xx_cpuidle_resources,
	.dev = {
		.platform_data	= &am33xx_cpuidle_pdata,
	},
};

static void __init am33xx_cpuidle_init(void)
{
	extern void __iomem * __init am33xx_get_mem_ctlr(void);
	int ret;

	am33xx_cpuidle_pdata.emif_base = am33xx_get_mem_ctlr();

	ret = platform_device_register(&am33xx_cpuidle_device);

	if (ret)
		pr_warning("AM33XX cpuidle registration failed\n");
}

#ifdef CONFIG_ANDROID
static void sgx_init(void)
{
	if (omap3_has_sgx()) {
		am33xx_gpu_init();
	}
}
#endif

static void touchscreen_init(void)
{
    setup_pin_mux(ft5x06_gpio_touchscreen_mux);
}

static void __init var_am335x_som_init(void)
{
	am33xx_cpuidle_init();
	am33xx_mux_init(board_mux);

	uart_init();
	printk ("Variscite AM33 SOM revision %s detected\n",
			get_var_am33_som_rev_str());

	if (get_var_am33_som_rev() < 2)
		pr_err("CRITICAL: SOM REVISION %s IS NOT SUPPORTED !!!\n", 
				get_var_am33_som_rev_str());

	clkout1_enable(); /* Required by Audio codec */
	mmc1_wl12xx_init();
	mmc0_init();
	uart1_wl12xx_init();
	wl12xx_init();
	som_nand_init();
	lcdc_init();
	mcasp0_init();
	rmii1_init();
	ethernet_init();
    touchscreen_init();
	i2c1_init();
	sgx_init();
	usb_musb_init(&musb_board_data);

    setup_pin_mux(cdi_bus_pin_mux);
    spi_register_board_info(spidevices, ARRAY_SIZE(spidevices));

	/* Create an alias for icss clock */
	if (clk_add_alias("pruss", NULL, "pruss_uart_gclk", NULL))
		pr_warn("failed to create an alias: icss_uart_gclk --> pruss\n");
	/* Create an alias for gfx/sgx clock */
	if (clk_add_alias("sgx_ck", NULL, "gfx_fclk", NULL))
		pr_warn("failed to create an alias: gfx_fclk --> sgx_ck\n");
}

static void __init var_am335x_som_map_io(void)
{
	omap2_set_globals_am33xx();
	omapam33xx_map_common_io();
}

MACHINE_START(VAR_SOM_AM33, "VAR-SOM-AM33")
	/* Maintainer: Uri Yosef <uri.y@variscite.com> */
	.atag_offset	= 0x100,
	.map_io		= var_am335x_som_map_io,
	.init_early	= am33xx_init_early,
	.init_irq	= ti81xx_init_irq,
	.handle_irq     = omap3_intc_handle_irq,
	.timer		= &omap3_am33xx_timer,
	.init_machine	= var_am335x_som_init,
MACHINE_END
