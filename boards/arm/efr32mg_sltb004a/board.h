/*
 * Copyright (c) 2018 Christian Taedcke, Diego Sueiro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __INC_BOARD_H
#define __INC_BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef _ASMLANGUAGE

#include <soc.h>
#include "soc_pinmap.h"

#endif /* !_ASMLANGUAGE */

#ifdef __cplusplus
}
#endif


/* Push button PB0 (SW0) */
#define PB0_GPIO_NAME	CONFIG_GPIO_GECKO_PORTD_NAME
#define PB0_GPIO_PIN	14
#define SW0_GPIO_NAME	PB0_GPIO_NAME
#define SW0_GPIO_PIN	PB0_GPIO_PIN

/* Push button PB1 */
#define PB1_GPIO_NAME	CONFIG_GPIO_GECKO_PORTD_NAME
#define PB1_GPIO_PIN	15

/* LED 0 (RED) */
#define LED0_GPIO_NAME	CONFIG_GPIO_GECKO_PORTD_NAME
#define LED0_GPIO_PORT  LED0_GPIO_NAME
#define LED0_GPIO_PIN	8

/* LED 1 (GREEN) */
#define LED1_GPIO_NAME	CONFIG_GPIO_GECKO_PORTD_NAME
#define LED1_GPIO_PORT  LED1_GPIO_NAME
#define LED1_GPIO_PIN	9

/* Environment Sensors Enable Pin */
#define ENV_SENSE_ENABLE_NAME	CONFIG_GPIO_GECKO_PORTF_NAME
#define ENV_SENSE_ENABLE_PIN	9

#endif /* __INC_BOARD_H */
