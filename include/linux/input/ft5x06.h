#ifndef _FT5X06_H
#define _FT5X06_H

/*
 * Copyright (c) 2016 Nenad Radulovic, <nenad.b.radulovic@gmail.com>
 */

struct ft5x06_platform_data {
	int irq_pin;
	int reset_pin;
    int abs_x;
    int abs_y;
};

#endif /* _FT5X06_H */
