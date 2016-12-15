/*
 * rtc-tps65910.c -- TPS65910 Real Time Clock interface
 *
 * Copyright (C) 2012 Variscite, Ltd
 * Author: Uri Yosef <uri.y@variscite.com>
 *
 * Based on TI driver trc-twl.c
 *   Copyright (C) 2007 MontaVista Software, Inc
 *   Author: Alexandre Rusev <source@mvista.com>
 *
 * Based on original TI driver twl4030-rtc.c
 *   Copyright (C) 2006 Texas Instruments, Inc.
 *
 * Based on rtc-omap.c
 *   Copyright (C) 2003 MontaVista Software, Inc.
 *   Author: George G. Davis <gdavis@mvista.com> or <source@mvista.com>
 *   Copyright (C) 2006 David Brownell
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/rtc.h>
#include <linux/bcd.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>

#include <linux/mfd/tps65910.h>

static struct tps65910 *tps65910;

/* RTC_CTRL_REG bitfields */
#define BIT_RTC_CTRL_REG_STOP_RTC_M              0x01
#define BIT_RTC_CTRL_REG_ROUND_30S_M             0x02
#define BIT_RTC_CTRL_REG_AUTO_COMP_M             0x04
#define BIT_RTC_CTRL_REG_MODE_12_24_M            0x08
#define BIT_RTC_CTRL_REG_TEST_MODE_M             0x10
#define BIT_RTC_CTRL_REG_SET_32_COUNTER_M        0x20
#define BIT_RTC_CTRL_REG_GET_TIME_M              0x40
#define BIT_RTC_CTRL_REG_RTC_V_OPT_M             0x80

/* RTC_STATUS_REG bitfields */
#define BIT_RTC_STATUS_REG_RUN_M                 0x02
#define BIT_RTC_STATUS_REG_1S_EVENT_M            0x04
#define BIT_RTC_STATUS_REG_1M_EVENT_M            0x08
#define BIT_RTC_STATUS_REG_1H_EVENT_M            0x10
#define BIT_RTC_STATUS_REG_1D_EVENT_M            0x20
#define BIT_RTC_STATUS_REG_ALARM_M               0x40
#define BIT_RTC_STATUS_REG_POWER_UP_M            0x80

/* RTC_INTERRUPTS_REG bitfields */
#define BIT_RTC_INTERRUPTS_REG_EVERY_M           0x03
#define BIT_RTC_INTERRUPTS_REG_IT_TIMER_M        0x04
#define BIT_RTC_INTERRUPTS_REG_IT_ALARM_M        0x08

/* DEVCTRL bitfields */
#define BIT_RTC_PWDN                             0x40


/* TPS65910_SECONDS through TPS65910_YEARS is how many registers? */
#define ALL_TIME_REGS		6

static int tps65910_rtc_read(u8 *data, int cnt, u8 reg)
{
	int ret;

	ret = tps65910->read(tps65910, reg, cnt, data);
	if (ret < 0)
		pr_err("tps65910-rtc: Could not read TWL"
		       "register %X - error %d\n", reg, ret);
	return ret;
}

static int tps65910_rtc_write(u8 *data, int cnt, u8 reg)
{
	int ret;

	ret = tps65910->write(tps65910, reg, cnt, data);
	if (ret < 0)
		pr_err("tps65910-rtc: Could not write TWL"
		       "register %X - error %d\n", reg, ret);
	return ret;
}

/*
 * Cache the value for timer/alarm interrupts register; this is
 * only changed by callers holding rtc ops lock (or resume).
 */
static unsigned char rtc_irq_bits;

/*
 * Enable 1/second update and/or alarm interrupts.
 */
static int set_rtc_irq_bit(unsigned char bit)
{
	unsigned char val;
	int ret;

	val = rtc_irq_bits | bit;
	val &= ~BIT_RTC_INTERRUPTS_REG_EVERY_M;
	ret = tps65910_rtc_write(&val, 1, TPS65910_RTC_INTERRUPTS);
	if (ret == 0)
		rtc_irq_bits = val;

	return ret;
}

/*
 * Disable update and/or alarm interrupts.
 */
static int mask_rtc_irq_bit(unsigned char bit)
{
	unsigned char val;
	int ret;

	val = rtc_irq_bits & ~bit;
	ret = tps65910_rtc_write(&val, 1, TPS65910_RTC_INTERRUPTS);
	if (ret == 0)
		rtc_irq_bits = val;

	return ret;
}

static int tps65910_rtc_alarm_irq_enable(struct device *dev, unsigned enabled)
{
	int ret;

	if (enabled)
		ret = set_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_ALARM_M);
	else
		ret = mask_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_ALARM_M);

	return ret;
}

/*
 * Gets current TWL RTC time and date parameters.
 *
 * The RTC's time/alarm representation is not what gmtime(3) requires
 * Linux to use:
 *
 *  - Months are 1..12 vs Linux 0-11
 *  - Years are 0..99 vs Linux 1900..N (we assume 21st century)
 */
static int tps65910_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	unsigned char rtc_data[ALL_TIME_REGS + 1];
	int ret;
	u8 save_control;

	ret = tps65910_rtc_read(&save_control, 1, TPS65910_RTC_CTRL);
	if (ret < 0)
		return ret;

	save_control |= BIT_RTC_CTRL_REG_GET_TIME_M;

	ret = tps65910_rtc_write(&save_control, 1, TPS65910_RTC_CTRL);
	if (ret < 0)
		return ret;

	ret = tps65910_rtc_read(rtc_data, ALL_TIME_REGS,
			TPS65910_SECONDS);

	if (ret < 0) {
		dev_err(dev, "rtc_read_time error %d\n", ret);
		return ret;
	}

	tm->tm_sec = bcd2bin(rtc_data[0]);
	tm->tm_min = bcd2bin(rtc_data[1]);
	tm->tm_hour = bcd2bin(rtc_data[2]);
	tm->tm_mday = bcd2bin(rtc_data[3]);
	tm->tm_mon = bcd2bin(rtc_data[4]) - 1;
	tm->tm_year = bcd2bin(rtc_data[5]) + 100;

	return ret;
}

static int tps65910_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	unsigned char save_control;
	unsigned char rtc_data[ALL_TIME_REGS + 1];
	int ret;

	rtc_data[0] = bin2bcd(tm->tm_sec);
	rtc_data[1] = bin2bcd(tm->tm_min);
	rtc_data[2] = bin2bcd(tm->tm_hour);
	rtc_data[3] = bin2bcd(tm->tm_mday);
	rtc_data[4] = bin2bcd(tm->tm_mon + 1);
	rtc_data[5] = bin2bcd(tm->tm_year - 100);

	/* Stop RTC while updating the TC registers */
	ret = tps65910_rtc_read(&save_control, 1, TPS65910_RTC_CTRL);
	if (ret < 0)
		goto out;

	save_control &= ~BIT_RTC_CTRL_REG_STOP_RTC_M;
	ret = tps65910_rtc_write(&save_control, 1, TPS65910_RTC_CTRL);
	if (ret < 0)
		goto out;

	/* update all the time registers in one shot */
	ret = tps65910_rtc_write(rtc_data, ALL_TIME_REGS,
		TPS65910_SECONDS);
	if (ret < 0) {
		dev_err(dev, "rtc_set_time error %d\n", ret);
		goto out;
	}

	/* Start back RTC */
	save_control |= BIT_RTC_CTRL_REG_STOP_RTC_M;
	ret = tps65910_rtc_write(&save_control, 1, TPS65910_RTC_CTRL);

out:
	return ret;
}

/*
 * Gets current TWL RTC alarm time.
 */
static int tps65910_rtc_read_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	unsigned char rtc_data[ALL_TIME_REGS + 1];
	int ret;

	ret = tps65910_rtc_read(rtc_data, ALL_TIME_REGS,
			TPS65910_ALARM_SECONDS);
	if (ret < 0) {
		dev_err(dev, "rtc_read_alarm error %d\n", ret);
		return ret;
	}

	/* some of these fields may be wildcard/"match all" */
	alm->time.tm_sec = bcd2bin(rtc_data[0]);
	alm->time.tm_min = bcd2bin(rtc_data[1]);
	alm->time.tm_hour = bcd2bin(rtc_data[2]);
	alm->time.tm_mday = bcd2bin(rtc_data[3]);
	alm->time.tm_mon = bcd2bin(rtc_data[4]) - 1;
	alm->time.tm_year = bcd2bin(rtc_data[5]) + 100;

	/* report cached alarm enable state */
	if (rtc_irq_bits & BIT_RTC_INTERRUPTS_REG_IT_ALARM_M)
		alm->enabled = 1;

	return ret;
}

static int tps65910_rtc_set_alarm(struct device *dev, struct rtc_wkalrm *alm)
{
	unsigned char alarm_data[ALL_TIME_REGS + 1];
	int ret;

	ret = tps65910_rtc_alarm_irq_enable(dev, 0);
	if (ret)
		goto out;

	alarm_data[1] = bin2bcd(alm->time.tm_sec);
	alarm_data[2] = bin2bcd(alm->time.tm_min);
	alarm_data[3] = bin2bcd(alm->time.tm_hour);
	alarm_data[4] = bin2bcd(alm->time.tm_mday);
	alarm_data[5] = bin2bcd(alm->time.tm_mon + 1);
	alarm_data[6] = bin2bcd(alm->time.tm_year - 100);

	/* update all the alarm registers in one shot */
	ret = tps65910_rtc_write(alarm_data, ALL_TIME_REGS,
		TPS65910_ALARM_SECONDS);
	if (ret) {
		dev_err(dev, "rtc_set_alarm error %d\n", ret);
		goto out;
	}

	if (alm->enabled)
		ret = tps65910_rtc_alarm_irq_enable(dev, 1);
out:
	return ret;
}

#ifdef TPS65910_IRQ_SUPPORTED
static irqreturn_t tps65910_rtc_interrupt(int irq, void *rtc)
{
	unsigned long events = 0;
	int ret = IRQ_NONE;
	int res;
	u8 rd_reg;

	res = tps65910_rtc_read(&rd_reg, 1, TPS65910_RTC_STATUS);
	if (res)
		goto out;
	/*
	 * Figure out source of interrupt: ALARM or TIMER in RTC_STATUS_REG.
	 * only one (ALARM or RTC) interrupt source may be enabled
	 * at time, we also could check our results
	 * by reading RTS_INTERRUPTS_REGISTER[IT_TIMER,IT_ALARM]
	 */
	if (rd_reg & BIT_RTC_STATUS_REG_ALARM_M)
		events |= RTC_IRQF | RTC_AF;
	else
		events |= RTC_IRQF | RTC_UF;

	rd_reg = rd_reg | BIT_RTC_STATUS_REG_ALARM_M;
	res = tps65910_rtc_write(&rd_reg, 1,
				   TPS65910_RTC_STATUS);
	if (res)
		goto out;

	/* Notify RTC core on event */
	rtc_update_irq(rtc, 1, events);

	ret = IRQ_HANDLED;
out:
	return ret;
}
#endif

static struct rtc_class_ops tps65910_rtc_ops = {
	.read_time	= tps65910_rtc_read_time,
	.set_time	= tps65910_rtc_set_time,
	.read_alarm	= tps65910_rtc_read_alarm,
	.set_alarm	= tps65910_rtc_set_alarm,
	.alarm_irq_enable = tps65910_rtc_alarm_irq_enable,
};

static int __devinit tps65910_rtc_probe(struct platform_device *pdev)
{
	struct rtc_device *rtc;
	int ret = -EINVAL;
#ifdef TPS65910_IRQ_SUPPORTED
	int irq = platform_get_irq(pdev, 0);
#endif
	u8 rd_reg;

	tps65910 = dev_get_drvdata(pdev->dev.parent);

#ifdef TPS65910_IRQ_SUPPORTED
	if (irq <= 0)
		goto out1;
#endif

	/* Take rtc out of reset */
	tps65910_rtc_read(&rd_reg, 1, TPS65910_DEVCTRL);
	rd_reg &= ~BIT_RTC_PWDN;
	ret = tps65910_rtc_write(&rd_reg, 1, TPS65910_DEVCTRL);
	if (ret < 0)
		goto out1;

	/* Dummy read to ensure that the register gets updated.
	 * Please refer tps65910 TRM table:25 for details
	 */
	tps65910_rtc_read(&rd_reg, 1, TPS65910_RTC_STATUS);

	ret = tps65910_rtc_read(&rd_reg, 1, TPS65910_RTC_STATUS);
	if (ret < 0)
		goto out1;

	if (rd_reg & BIT_RTC_STATUS_REG_POWER_UP_M)
		dev_warn(&pdev->dev, "Power up reset detected.\n");

	if (rd_reg & BIT_RTC_STATUS_REG_ALARM_M)
		dev_warn(&pdev->dev, "Pending Alarm interrupt detected.\n");

	/* Clear RTC Power up reset and pending alarm interrupts */
	ret = tps65910_rtc_write(&rd_reg, 1, TPS65910_RTC_STATUS);
	if (ret < 0)
		goto out1;

	ret = tps65910_rtc_read(&rd_reg, 1, TPS65910_INT_STS);
	if (ret < 0) {
		 dev_err(&pdev->dev, "failed to read init sys register\n");
		 goto out1;
	}

	if (rd_reg & 0x40) {
		dev_info(&pdev->dev, "pending alarm interrupt! clearing.");
		tps65910_rtc_write(&rd_reg, 1, TPS65910_INT_STS);
	}

#ifdef TPS65910_IRQ_SUPPORTED
	twl6030_interrupt_unmask(TWL6030_RTC_INT_MASK, REG_INT_MSK_LINE_A);
#endif

	/* Check RTC module status, Enable if it is off */
	ret = tps65910_rtc_read(&rd_reg, 1, TPS65910_RTC_CTRL);
	if (ret < 0)
		goto out1;

	if (!(rd_reg & BIT_RTC_CTRL_REG_STOP_RTC_M)) {
		dev_info(&pdev->dev, "Enabling TPS65910 RTC.\n");
		rd_reg = BIT_RTC_CTRL_REG_STOP_RTC_M;
		ret = tps65910_rtc_write(&rd_reg, 1, TPS65910_RTC_CTRL);
		if (ret < 0)
			goto out1;
	}

	/* init cached IRQ enable bits */
	ret = tps65910_rtc_read(&rtc_irq_bits, 1, TPS65910_RTC_INTERRUPTS);
	if (ret < 0)
		goto out1;

	rtc = rtc_device_register(pdev->name,
				  &pdev->dev, &tps65910_rtc_ops, THIS_MODULE);
	if (IS_ERR(rtc)) {
		ret = PTR_ERR(rtc);
		dev_err(&pdev->dev, "can't register RTC device, err %ld\n",
			PTR_ERR(rtc));
		goto out1;
	}

#ifdef TPS65910_IRQ_SUPPORTED
	ret = request_threaded_irq(irq, NULL, tps65910_rtc_interrupt,
				   IRQF_TRIGGER_RISING,
				   dev_name(&rtc->dev), rtc);
	if (ret < 0) {
		dev_err(&pdev->dev, "IRQ is not free.\n");
		goto out2;
	}
#endif

	platform_set_drvdata(pdev, rtc);
	return 0;

#ifdef TPS65910_IRQ_SUPPORTED
out2:
	rtc_device_unregister(rtc);
#endif
out1:
	return ret;
}

/*
 * Disable all TPS65910 RTC module interrupts.
 * Sets status flag to free.
 */
static int __devexit tps65910_rtc_remove(struct platform_device *pdev)
{
	/* leave rtc running, but disable irqs */
	struct rtc_device *rtc = platform_get_drvdata(pdev);
#ifdef TPS65910_IRQ_SUPPORTED
	int irq = platform_get_irq(pdev, 0);
#endif

	mask_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_ALARM_M);
	mask_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_TIMER_M);
#ifdef TPS65910_IRQ_SUPPORTED
	twl6030_interrupt_mask(TWL6030_RTC_INT_MASK, REG_INT_MSK_STS_A);
#endif

#ifdef TPS65910_IRQ_SUPPORTED
	free_irq(irq, rtc);
#endif

	rtc_device_unregister(rtc);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static void tps65910_rtc_shutdown(struct platform_device *pdev)
{
	/* mask timer interrupts, but leave alarm interrupts on to enable
	   power-on when alarm is triggered */
	mask_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_TIMER_M);
}

#ifdef CONFIG_PM

static unsigned char irqstat;

static int tps65910_rtc_suspend(struct platform_device *pdev, pm_message_t state)
{
	irqstat = rtc_irq_bits;

	mask_rtc_irq_bit(BIT_RTC_INTERRUPTS_REG_IT_TIMER_M);
	return 0;
}

static int tps65910_rtc_resume(struct platform_device *pdev)
{
	set_rtc_irq_bit(irqstat);
	return 0;
}

#else
#define tps65910_rtc_suspend NULL
#define tps65910_rtc_resume  NULL
#endif

MODULE_ALIAS("platform:tps65910-rtc");

static struct platform_driver tps65910_rtc_driver = {
	.probe		= tps65910_rtc_probe,
	.remove		= __devexit_p(tps65910_rtc_remove),
	.shutdown	= tps65910_rtc_shutdown,
	.suspend	= tps65910_rtc_suspend,
	.resume		= tps65910_rtc_resume,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "tps65910-rtc",
	},
};

static int __init tps65910_rtc_init(void)
{
	return platform_driver_register(&tps65910_rtc_driver);
}
module_init(tps65910_rtc_init);

static void __exit tps65910_rtc_exit(void)
{
	platform_driver_unregister(&tps65910_rtc_driver);
}
module_exit(tps65910_rtc_exit);

MODULE_AUTHOR("Uri Yosef <uri.y@variscite.com>, Variscite");
MODULE_LICENSE("GPL");
