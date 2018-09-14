/*
 * Copyright (c) 2018 Diego Sueiro
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <i2c.h>
#include <misc/util.h>
#include "i2c-priv.h"
#include <em_cmu.h>
#include <em_i2c.h>
#include <em_gpio.h>
#include <board.h>


#define DEV_CFG(dev) \
	((struct i2c_gecko_config * const)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct i2c_gecko_data * const)(dev)->driver_data)
#define DEV_BASE(dev) \
	((I2C_TypeDef *)(DEV_CFG(dev))->base)

struct i2c_gecko_config {
	I2C_TypeDef *base;
	CMU_Clock_TypeDef clock;
	I2C_Init_TypeDef i2cInit;
	struct soc_gpio_pin pin_sda;
	struct soc_gpio_pin pin_scl;
	unsigned int loc;
	u32_t bitrate;
	void (*irq_config_func)(struct device *dev);
};

struct i2c_gecko_data {
	struct k_sem device_sync_sem;
	u32_t dev_config;
};

void i2c_gecko_config_pins(struct device *dev,
			   const struct soc_gpio_pin *pin_sda,
			   const struct soc_gpio_pin *pin_scl, u8_t loc)
{
	I2C_TypeDef *base = DEV_BASE(dev);

	soc_gpio_configure(pin_scl);
	soc_gpio_configure(pin_sda);

#ifdef _I2C_ROUTEPEN_MASK
	base->ROUTEPEN = I2C_ROUTEPEN_SDAPEN | I2C_ROUTEPEN_SCLPEN;
	base->ROUTELOC0 = (loc << _I2C_ROUTELOC0_SDALOC_SHIFT)
			   | (loc << _I2C_ROUTELOC0_SCLLOC_SHIFT);
#else
	base->ROUTE = I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN | (loc << 8);
#endif
}

static int i2c_gecko_configure(struct device *dev, u32_t dev_config_raw)
{
	I2C_TypeDef *base = DEV_BASE(dev);
	struct i2c_gecko_config *config = DEV_CFG(dev);
	struct i2c_gecko_data *data = DEV_DATA(dev);
	u32_t baudrate;

	if (!(I2C_MODE_MASTER & dev_config_raw)) {
		return -EINVAL;
	}

	switch (I2C_SPEED_GET(dev_config_raw)) {
	case I2C_SPEED_STANDARD:
		baudrate = KHZ(100);
		break;
	case I2C_SPEED_FAST:
		baudrate = MHZ(1);
		break;
	default:
		return -EINVAL;
	}

	data->dev_config = dev_config_raw;
	config->i2cInit.freq = baudrate;

	I2C_Init(base, &config->i2cInit);

	return 0;
}

static int i2c_gecko_transfer(struct device *dev, struct i2c_msg *msgs,
			      u8_t num_msgs, u16_t addr)
{
	I2C_TypeDef *base = DEV_BASE(dev);
	struct i2c_gecko_data *data = DEV_DATA(dev);
	I2C_TransferSeq_TypeDef seq;
	I2C_TransferReturn_TypeDef ret = -EIO;
	u32_t timeout = 300000;

	if (!num_msgs) {
		return 0;
	}

	seq.addr = addr << 1;

	do {
		seq.buf[0].data = msgs->buf;
		seq.buf[0].len	= msgs->len;

		if ((msgs->flags & I2C_MSG_RW_MASK) == I2C_MSG_READ) {
			seq.flags = I2C_FLAG_READ;
		} else {
			seq.flags = I2C_FLAG_WRITE;
			if (num_msgs > 1) {
				/* Next message */
				msgs++;
				num_msgs--;
				if ((msgs->flags & I2C_MSG_RW_MASK)
				    == I2C_MSG_READ) {
					seq.flags = I2C_FLAG_WRITE_READ;
				} else {
					seq.flags = I2C_FLAG_WRITE_WRITE;
				}
				seq.buf[1].data = msgs->buf;
				seq.buf[1].len	= msgs->len;
			}
		}

		if (data->dev_config & I2C_ADDR_10_BITS) {
			seq.flags |= I2C_FLAG_10BIT_ADDR;
		}

		/* Do a polled transfer */
		ret = I2C_TransferInit(base, &seq);
		while (ret == i2cTransferInProgress && timeout--) {
			ret = I2C_Transfer(base);
		}

		if (ret != i2cTransferDone) {
			goto finish;
		}

		/* Next message */
		msgs++;
		num_msgs--;
	} while (num_msgs);

finish:
	if (ret != i2cTransferDone) {
		ret = -EIO;
	}
	return ret;
}


static void i2c_gecko_isr(void *arg)
{
	struct device *dev = (struct device *)arg;
	I2C_TypeDef *base = DEV_BASE(dev);

	I2C_IntDisable(base, _I2C_IEN_MASK);
}

static int i2c_gecko_init(struct device *dev)
{
	struct i2c_gecko_config *config = DEV_CFG(dev);
	u32_t bitrate_cfg;
	int error;

	CMU_ClockEnable(config->clock, true);

	i2c_gecko_config_pins(dev, &config->pin_sda,
			      &config->pin_scl, config->loc);

	bitrate_cfg = _i2c_map_dt_bitrate(config->bitrate);

	error = i2c_gecko_configure(dev, I2C_MODE_MASTER | bitrate_cfg);
	if (error) {
		return error;
	}

	config->irq_config_func(dev);

	return 0;
}

static const struct i2c_driver_api i2c_gecko_driver_api = {
	.configure = i2c_gecko_configure,
	.transfer = i2c_gecko_transfer,
};

#ifdef CONFIG_I2C_0
static void i2c_gecko_config_func_0(struct device *dev);

static struct i2c_gecko_config i2c_gecko_config_0 = {
	.base = (I2C_TypeDef *)I2C_0_BASE_ADDRESS,
	.clock = cmuClock_I2C0,
	.i2cInit = I2C_INIT_DEFAULT,
	.pin_sda = PIN_I2C0_SDA,
	.pin_scl = PIN_I2C0_SCL,
	.loc = I2C_0_LOCATION,
	.bitrate = I2C_0_CLOCK_FREQUENCY,
	.irq_config_func = i2c_gecko_config_func_0,
};

static struct i2c_gecko_data i2c_gecko_data_0;

DEVICE_AND_API_INIT(i2c_gecko_0, I2C_0_LABEL, &i2c_gecko_init,
			&i2c_gecko_data_0, &i2c_gecko_config_0,
			POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
			&i2c_gecko_driver_api);

static void i2c_gecko_config_func_0(struct device *dev)
{
	ARG_UNUSED(dev);

	IRQ_CONNECT(I2C_0_IRQ, I2C_0_IRQ_PRIORITY,
			i2c_gecko_isr, DEVICE_GET(i2c_gecko_0), 0);

	irq_enable(I2C_0_IRQ);
}
#endif /* CONFIG_I2C_1 */

#ifdef CONFIG_I2C_1
static void i2c_gecko_config_func_1(struct device *dev);

static struct i2c_gecko_config i2c_gecko_config_1 = {
	.base = (I2C_TypeDef *)I2C_1_BASE_ADDRESS,
	.clock = cmuClock_I2C1,
	.i2cInit = I2C_INIT_DEFAULT,
	.pin_sda = PIN_I2C1_SDA,
	.pin_scl = PIN_I2C1_SCL,
	.loc = I2C_1_LOCATION,
	.bitrate = I2C_1_CLOCK_FREQUENCY,
	.irq_config_func = i2c_gecko_config_func_1,
};

static struct i2c_gecko_data i2c_gecko_data_1;

DEVICE_AND_API_INIT(i2c_gecko_1, I2C_1_LABEL, &i2c_gecko_init,
			&i2c_gecko_data_1, &i2c_gecko_config_1,
			POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEVICE,
			&i2c_gecko_driver_api);

static void i2c_gecko_config_func_1(struct device *dev)
{
	ARG_UNUSED(dev);

	IRQ_CONNECT(I2C_1_IRQ, I2C_1_IRQ_PRIORITY,
			i2c_gecko_isr, DEVICE_GET(i2c_gecko_1), 0);

	irq_enable(I2C_1_IRQ);
}
#endif /* CONFIG_I2C_1 */
