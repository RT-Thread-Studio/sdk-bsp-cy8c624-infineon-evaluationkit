/*
 * Copyright (c) 2006-2024, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 */

#include <RTduino.h>

static void _senfu_setup(void)
{
    /* put your setup code here, to run once: */
}

static void _senfu_loop(void)
{
    /* put your main code here, to run repeatedly: */

    delay(50);
}
RTDUINO_SKETCH_LOADER("senfu", _senfu_setup, _senfu_loop);