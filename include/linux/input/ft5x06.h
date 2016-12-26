#ifndef _FT5X06_H
#define _FT5X06_H

/*
 * Copyright (c) 2012 Simon Budig, <simon.budig@kernelconcepts.de>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

struct ft5x06_platform_data {
	int irq_pin;
	int reset_pin;
        
        int num_x;
        int num_y;
};

#endif /* _FT5X06_H */
