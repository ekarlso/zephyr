/*
 * Copyright (c) 2018 Diego Sueiro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <init.h>

static int efr32mg_sltb004a_init(struct device *dev)
{
	ARG_UNUSED(dev);
	return 0;
}

SYS_INIT(efr32mg_sltb004a_init, PRE_KERNEL_1, CONFIG_BOARD_INIT_PRIORITY);
