/*
 * Copyright (c) 2018 Kokoon Technology Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>

#include <adc.h>
#include <device.h>
#include <kernel.h>
#include <init.h>

#define ADC_CONTEXT_USES_KERNEL_TIMER
#include "adc_context.h"

#define LOG_LEVEL CONFIG_ADC_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(adc_stm32);

#include <clock_control/stm32_clock_control.h>

#ifdef CONFIG_SOC_SERIES_STM32L0X
#include <stm32l0xx.h>
#include <stm32l0xx_hal.h>
#include <stm32l0xx_hal_adc.h>
#include <stm32l0xx_hal_adc_ex.h>
#include <stm32l0xx_hal_cortex.h>
#elif defined(CONFIG_SOC_SERIES_STM32F4X)
#include <stm32f4xx.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_adc.h>
#include <stm32f4xx_hal_adc_ex.h>
#include <stm32f4xx_hal_cortex.h>
#endif


typedef void (*cfg_func_t)(struct device *dev);

struct adc_stm32_data {
	struct adc_context ctx;
	struct device *dev;
	u16_t *buffer;
	u16_t *repeat_buffer;

	const struct adc_sequence *entries;
	u8_t seq_size;
	u8_t resolution;
	
	/* Bit mask of the channels to be sampled. */
	u32_t channels;

	/* Index of the channel being sampled. */
	u8_t channel_id;

	volatile ADC_TypeDef *Instance;
};

struct adc_stm32_cfg {
	cfg_func_t cfg_func;
	u8_t dev_nr; 
};

static int check_buffer_size(const struct adc_sequence *sequence,
			     u8_t active_channels)
{
	size_t needed_buffer_size;
	needed_buffer_size = active_channels * sizeof(u16_t);
	if (sequence->options) {
		needed_buffer_size *= (1 + sequence->options->extra_samplings);
	}
	if (sequence->buffer_size < needed_buffer_size) {
		LOG_ERR("Provided buffer is too small (%u/%u)",
				sequence->buffer_size, needed_buffer_size);
		return -ENOMEM;
	}
	return 0;
}

static void adc_stm32_start_conversion(struct device *dev)
{
	struct adc_stm32_data *data = dev->driver_data;
	const struct adc_stm32_cfg *config = dev->config->config_info;
	// const struct adc_sequence *entry = data->ctx.sequence;
	LOG_INF("Starting conversion");
	// u32_t interval_us = 0;

	// if (entry->options) {
	// 	interval_us = entry->options->interval_us;
	// }

	// /* Setup sequence */

	// if (ADC_IS_CONVERSION_ONGOING_REGULAR(data) == RESET) {
	// 	data->Instance->CR |= ADC_CR_ADSTART;
	// }
	data->Instance->CR |= ADC_CR_ADSTART;

	LOG_INF("Convert done!");
}

static int start_read(struct device *dev, const struct adc_sequence *sequence)
{
	struct adc_stm32_data *data = dev->driver_data;
	const struct adc_stm32_cfg *config = dev->config->config_info;
	
	int err;

	u32_t channels = sequence->channels;

	data->channels = 0U;
	
	/* Ensure resolution is valid */
	// data->channels = seq_tbl->channels & data->active_channels;
	// if (data->channels != seq_tbl->channels) {
	// 	return -EINVAL;
	// }

	switch (sequence->resolution) {
	case 6:
		break;
	case 8:
		break;
	case 10:
		break;
	case 12:
		break;
	default:
		LOG_ERR("Invalid resolution");
		return -EINVAL;
	}

	data->buffer = sequence->buffer;

	u8_t num_active_channels = 0U;
	u8_t channel = 0U;

	while (channels > 0) {
		if (channels & 1) {
			++num_active_channels;
		}

		channels >>= 1;
		++channel;
	}

	err = check_buffer_size(sequence, num_active_channels);
	if (err) {
		return err;
	}

	data->Instance->CFGR1 &= -ADC_CFGR1_RES;
	data->Instance->CFGR1 |= sequence->resolution & ADC_CFGR1_RES;
	data->Instance->CHSELR = (1 << data->channel_id);

	data->Instance->ISR |= ADC_ISR_EOC;

	adc_context_start_read(&data->ctx, sequence);
	err = adc_context_wait_for_completion(&data->ctx);
	adc_context_release(&data->ctx, err);

	return err;
}

/********************************************
 *********** ADC context-functions **********
 ********************************************/

static void adc_context_start_sampling(struct adc_context *ctx)
{
	struct adc_stm32_data *data =
		CONTAINER_OF(ctx, struct adc_stm32_data, ctx);

	data->channels = ctx->sequence->channels;
	data->repeat_buffer = data->buffer;

	adc_stm32_start_conversion(data->dev);
}

static void adc_context_update_buffer_pointer(struct adc_context *ctx,
					      bool repeat_sampling)
{
	struct adc_stm32_data *data =
		CONTAINER_OF(ctx, struct adc_stm32_data, ctx);

	if (repeat_sampling) {
		data->buffer = data->repeat_buffer;
	}
}

static void adc_stm32_isr(void *arg)
{
	struct device *dev = (struct device *)arg;

	struct adc_stm32_data *data = dev->driver_data;
	const struct adc_stm32_cfg *config = dev->config->config_info;

	LOG_INF("ISR triggered.");
}

/********************************************
 ******************* API ********************
 ********************************************/

static int adc_stm32_read(struct device *dev,
			  const struct adc_sequence *sequence)
{
	struct adc_stm32_data *data = dev->driver_data;

	adc_context_lock(&data->ctx, false, NULL);

	return start_read(dev, sequence);
}

#ifdef CONFIG_ADC_ASYNC
static int adc_stm32_read_async(struct device *dev,
				 const struct adc_sequence *sequence,
				 struct k_poll_signal *async)
{
	struct adc_stm32_data *data = dev->driver_data;


	adc_context_lock(&data->ctx, true, async);

	return start_read(dev, sequence);
}
#endif

int adc_stm32_channel_setup(struct device *dev,
			    const struct adc_channel_cfg *channel_cfg)
{
	u8_t channel_id = channel_cfg->channel_id;
	LOG_INF("Channel ID %d", channel_id);
	
	if (channel_id > 18) {
		LOG_ERR("Channel %d is not valid", channel_id);
		return -EINVAL;
	}

	if (channel_cfg->acquisition_time != ADC_ACQ_TIME_DEFAULT) {
		LOG_ERR("Invalid channel acquisition time");
		return -EINVAL;
	}

	if (channel_cfg->differential) {
		LOG_ERR("Differential channels are not supported");
		return -EINVAL;
	}

	if (channel_cfg->gain != ADC_GAIN_1) {
		LOG_ERR("Invalid channel gain");
		return -EINVAL;
	}

	if (channel_cfg->reference != ADC_REF_INTERNAL) {
		LOG_ERR("Invalid channel reference");
		return -EINVAL;
	}

	LOG_INF("Channel setup succeeded!");

	return 0;
}

/********************************************
 ****************** INIT ********************
 ********************************************/

static int adc_stm32_init(struct device *dev)
{
	struct adc_stm32_data *data = dev->driver_data;
	const struct adc_stm32_cfg *adc_cfg = dev->config->config_info;
	

	LOG_DBG("Init STM32 ADC");

	switch (adc_cfg->dev_nr) {
#ifdef CONFIG_ADC_0
		case 0:
			LOG_INF("ADC0 activated!");
			data->Instance = ADC1;
			break;
#endif
#ifdef CONFIG_ADC_1
		case 1:
			LOG_INF("ADC1 activated!");
			data->Instance = ADC2;
			break;
#endif
#ifdef CONFIG_ADC_2
		case 2:
			LOG_INF("ADC2 activated!");
			data->Instance = ADC3;
			break;
#endif
	}

	data->Instance->CFGR1 = 0;
	data->Instance->CFGR2 = 0;

	ADC->CCR = ADC_CCR_VREFEN | ADC_CCR_TSEN | ADC_CCR_PRESC_1;

	data->Instance->SMPR |= ADC_SMPR_SMP;
	data->Instance->ISR |= ADC_ISR_EOC;

	adc_context_unlock_unconditionally(&data->ctx);

	return 0;
}

static struct adc_driver_api api_stm32_driver_api = {
	.channel_setup = adc_stm32_channel_setup,
	.read = adc_stm32_read,
#ifdef CONFIG_ADC_ASYNC
	.read_async = adc_stm32_read_async,
#endif
};

#ifdef CONFIG_ADC_0

static void adc0_stm32_cfg_func(struct device *dev);

static const struct adc_stm32_cfg adc0_stm_cfg = {
	.cfg_func = adc0_stm32_cfg_func,
	.dev_nr = 0,
};

static struct adc_stm32_data adc_drv_data_dev0 = {
	ADC_CONTEXT_INIT_TIMER(adc_drv_data_dev0, ctx),
	ADC_CONTEXT_INIT_LOCK(adc_drv_data_dev0, ctx),
	ADC_CONTEXT_INIT_SYNC(adc_drv_data_dev0, ctx),
};

DEVICE_AND_API_INIT(adc_0, DT_ADC_0_NAME, &adc_stm32_init,
		    &adc_drv_data_dev0, &adc0_stm_cfg, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &api_stm32_driver_api);

static void adc0_stm32_cfg_func(struct device *dev)
{
	IRQ_CONNECT(DT_ST_STM32_ADC_ADC_0_IRQ, DT_ST_STM32_ADC_ADC_0_IRQ_PRIORITY, adc_stm32_isr,
		    DEVICE_GET(adc_0), 0);
	irq_enable(DT_ST_STM32_ADC_ADC_0_IRQ);
}

#endif // CONFIG_ADC_0

#ifdef CONFIG_ADC_1

static void adc1_stm32_cfg_func(struct device *dev);

static const struct adc_stm32_cfg adc1_stm_cfg = {
	.cfg_func = adc1_stm32_cfg_func,
	.dev_nr = 1,
};

static struct adc_stm32_data adc_drv_data_dev1 = {
	ADC_CONTEXT_INIT_TIMER(adc_drv_data_dev1, ctx),
	ADC_CONTEXT_INIT_LOCK(adc_drv_data_dev1, ctx),
	ADC_CONTEXT_INIT_SYNC(adc_drv_data_dev1, ctx),
};

DEVICE_AND_API_INIT(adc_stm32, DT_ADC_1_NAME, &adc_stm32_init,
		    &adc_drv_data_dev1, &adc_config_dev1, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &api_stm32_driver_api);

static void adc0_stm32_cfg_func(struct device *dev)
{
	IRQ_CONNECT(DT_ST_STM32_ADC_ADC_1_IRQ, DT_ST_STM32_ADC_ADC_1_IRQ_PRIORITY, adc_stm32_isr,
		    DEVICE_GET(adc_1), 0);
	irq_enable(DT_ST_STM32_ADC_ADC_1_IRQ);
}

#endif // CONFIG_ADC_1

#ifdef CONFIG_ADC_2

static void adc2_stm32_cfg_func(struct device *dev);

static const struct adc_stm32_cfg adc2_stm_cfg = {
	.cfg_func = adc2_stm32_cfg_func,
	.dev_nr = 2,
};

static struct adc_stm32_data adc_drv_data_dev2 = {
	ADC_CONTEXT_INIT_TIMER(adc_drv_data_dev2, ctx),
	ADC_CONTEXT_INIT_LOCK(adc_drv_data_dev2, ctx),
	ADC_CONTEXT_INIT_SYNC(adc_drv_data_dev2, ctx),
};

DEVICE_AND_API_INIT(adc_stm32, DT_ADC_2_NAME, &adc_stm32_init,
		    &adc_drv_data_dev2, &adc_config_dev2, POST_KERNEL,
		    CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &api_stm32_driver_api);

static void adc0_stm32_cfg_func(struct device *dev)
{
	IRQ_CONNECT(DT_ST_STM32_ADC_ADC_2_IRQ, DT_ST_STM32_ADC_ADC_2_IRQ_PRIORITY, adc_stm32_isr,
		    DEVICE_GET(adc_2), 0);
	irq_enable(DT_ST_STM32_ADC_ADC_2_IRQ);
}

#endif // CONFIG_ADC_2
