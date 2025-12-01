// SPDX-License-Identifier: GPL-2.0-only
/*
 * AD9910 SPI DDS (Direct Digital Synthesizer) driver
 *
 * Copyright 2025 Analog Devices Inc.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/iio/events.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/interrupt.h>
#include <linux/log2.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/spi/spi.h>
#include <linux/sysfs.h>
#include <linux/types.h>
#include <linux/units.h>

/* Register addresses */
#define AD9910_REG_CFR1			0x00
#define AD9910_REG_CFR2			0x01
#define AD9910_REG_CFR3			0x02
#define AD9910_REG_AUX_DAC		0x03
#define AD9910_REG_IO_UPDATE_RATE	0x04
#define AD9910_REG_FTW			0x07
#define AD9910_REG_POW			0x08
#define AD9910_REG_ASF			0x09
#define AD9910_REG_MULTICHIP_SYNC	0x0A
#define AD9910_REG_DRG_LIMIT		0x0B
#define AD9910_REG_DRG_STEP		0x0C
#define AD9910_REG_DRG_RATE		0x0D
#define AD9910_REG_PROFILE0		0x0E
#define AD9910_REG_PROFILE1		0x0F
#define AD9910_REG_PROFILE2		0x10
#define AD9910_REG_PROFILE3		0x11
#define AD9910_REG_PROFILE4		0x12
#define AD9910_REG_PROFILE5		0x13
#define AD9910_REG_PROFILE6		0x14
#define AD9910_REG_PROFILE7		0x15
#define AD9910_REG_RAM			0x16

#define AD9910_REG_PROFILE(x)		(AD9910_REG_PROFILE0 + (x))
#define AD9910_REG_HIGH32		0x100

/* CFR1 bit definitions */
#define AD9910_CFR1_RAM_ENABLE_MSK		BIT(31)
#define AD9910_CFR1_RAM_PLAYBACK_DEST_MSK	GENMASK(30, 29)
#define AD9910_CFR1_MANUAL_OSK_EXT_CTL_MSK	BIT(23)
#define AD9910_CFR1_INV_SINC_EN_MSK		BIT(22)
#define AD9910_CFR1_INT_PROFILE_CTL_MSK		GENMASK(20, 17)
#define AD9910_CFR1_SELECT_SINE_MSK		BIT(16)
#define AD9910_CFR1_LOAD_LRR_IO_UPDATE_MSK	BIT(15)
#define AD9910_CFR1_AUTOCLR_DIG_RAMP_ACCUM_MSK	BIT(14)
#define AD9910_CFR1_AUTOCLR_PHASE_ACCUM_MSK	BIT(13)
#define AD9910_CFR1_CLEAR_DIG_RAMP_ACCUM_MSK	BIT(12)
#define AD9910_CFR1_CLEAR_PHASE_ACCUM_MSK	BIT(11)
#define AD9910_CFR1_LOAD_ARR_IO_UPDATE_MSK	BIT(10)
#define AD9910_CFR1_OSK_ENABLE_MSK		BIT(9)
#define AD9910_CFR1_SELECT_AUTO_OSK_MSK		BIT(8)
#define AD9910_CFR1_DIGITAL_POWER_DOWN_MSK	BIT(7)
#define AD9910_CFR1_DAC_POWER_DOWN_MSK		BIT(6)
#define AD9910_CFR1_REFCLK_INPUT_POWER_DOWN_MSK	BIT(5)
#define AD9910_CFR1_AUX_DAC_POWER_DOWN_MSK	BIT(4)
#define AD9910_CFR1_SOFT_POWER_DOWN_MSK		GENMASK(7, 4)
#define AD9910_CFR1_EXT_POWER_DOWN_CTL_MSK	BIT(3)
#define AD9910_CFR1_SDIO_INPUT_ONLY_MSK		BIT(1)
#define AD9910_CFR1_LSB_FIRST_MSK		BIT(0)

/* CFR2 bit definitions */
#define AD9910_CFR2_AMP_SCALE_SINGLE_TONE_MSK	BIT(24)
#define AD9910_CFR2_INTERNAL_IO_UPDATE_MSK	BIT(23)
#define AD9910_CFR2_SYNC_CLK_EN_MSK		BIT(22)
#define AD9910_CFR2_DRG_DEST_MSK		GENMASK(21, 20)
#define AD9910_CFR2_DRG_ENABLE_MSK		BIT(19)
#define AD9910_CFR2_DRG_NO_DWELL_HIGH_MSK	BIT(18)
#define AD9910_CFR2_DRG_NO_DWELL_LOW_MSK	BIT(17)
#define AD9910_CFR2_READ_EFFECTIVE_FTW_MSK	BIT(16)
#define AD9910_CFR2_IO_UPDATE_RATE_CTL_MSK	GENMASK(15, 14)
#define AD9910_CFR2_PDCLK_ENABLE_MSK		BIT(11)
#define AD9910_CFR2_PDCLK_INVERT_MSK		BIT(10)
#define AD9910_CFR2_TXENABLE_INVERT_MSK		BIT(9)
#define AD9910_CFR2_MATCHED_LATENCY_EN_MSK	BIT(7)
#define AD9910_CFR2_DATA_ASM_HOLD_LAST_MSK	BIT(6)
#define AD9910_CFR2_SYNC_TIMING_VAL_DISABLE_MSK	BIT(5)
#define AD9910_CFR2_PARALLEL_DATA_PORT_EN_MSK	BIT(4)
#define AD9910_CFR2_FM_GAIN_MSK			GENMASK(3, 0)

/* CFR3 bit definitions */
#define AD9910_CFR3_OPEN_MSK			0x08070000
#define AD9910_CFR3_DRV0_MSK			GENMASK(29, 28)
#define AD9910_CFR3_VCO_SEL_MSK			GENMASK(26, 24)
#define AD9910_CFR3_ICP_MSK			GENMASK(21, 19)
#define AD9910_CFR3_REFCLK_IN_DIV_BYPASS_MSK	BIT(15)
#define AD9910_CFR3_REFCLK_IN_DIV_RESETB_MSK	BIT(14)
#define AD9910_CFR3_PFD_RESET_MSK		BIT(10)
#define AD9910_CFR3_PLL_EN_MSK			BIT(8)
#define AD9910_CFR3_N_MSK			GENMASK(7, 1)

/* Auxiliary DAC Control Register Bits */
#define AD9910_AUX_DAC_FSC_MSK			GENMASK(7, 0)

/* ASF Register Bits */
#define AD9910_ASF_AMP_RAMP_RATE_MSK		GENMASK(31, 16)
#define AD9910_ASF_AMP_SCALE_FACTOR_MSK		GENMASK(15, 2)
#define AD9910_ASF_AMP_STEP_SIZE_MSK		GENMASK(1, 0)

/* Multichip Sync Register Bits */
#define AD9910_MC_SYNC_VALIDATION_DELAY_MSK	GENMASK(31, 28)
#define AD9910_MC_SYNC_RECEIVER_ENABLE_MSK	BIT(27)
#define AD9910_MC_SYNC_GENERATOR_ENABLE_MSK	BIT(26)
#define AD9910_MC_SYNC_GENERATOR_POLARITY_MSK	BIT(25)
#define AD9910_MC_SYNC_STATE_PRESET_MSK		GENMASK(23, 18)
#define AD9910_MC_SYNC_OUTPUT_DELAY_MSK		GENMASK(15, 11)
#define AD9910_MC_SYNC_INPUT_DELAY_MSK		GENMASK(7, 3)

/* Digital Ramp Limit Register */
#define AD9910_DRG_LIMIT_UPPER_MSK		GENMASK_ULL(63, 32)
#define AD9910_DRG_LIMIT_LOWER_MSK		GENMASK_ULL(31, 0)

/* Digital Ramp Step Register */
#define AD9910_DRG_STEP_DEC_MSK			GENMASK_ULL(63, 32)
#define AD9910_DRG_STEP_INC_MSK			GENMASK_ULL(31, 0)

/* Digital Ramp Rate Register */
#define AD9910_DRG_RATE_DEC_MSK			GENMASK(31, 16)
#define AD9910_DRG_RATE_INC_MSK			GENMASK(15, 0)

/* Profile Register Format (Single Tone Mode) */
#define AD9910_PROFILE_ST_ASF_MSK		GENMASK_ULL(61, 48)
#define AD9910_PROFILE_ST_POW_MSK		GENMASK_ULL(47, 32)
#define AD9910_PROFILE_ST_FTW_MSK		GENMASK_ULL(31, 0)

/* Profile Register Format (RAM Mode) */
#define AD9910_PROFILE_RAM_ADDR_STEP_RATE_MSK	GENMASK_ULL(55, 40)
#define AD9910_PROFILE_RAM_END_ADDR_MSK		GENMASK_ULL(39, 30)
#define AD9910_PROFILE_RAM_START_ADDR_MSK	GENMASK_ULL(23, 14)
#define AD9910_PROFILE_RAM_NO_DWELL_HIGH_MSK	BIT_ULL(5)
#define AD9910_PROFILE_RAM_ZERO_CROSSING_MSK	BIT_ULL(3)
#define AD9910_PROFILE_RAM_MODE_CONTROL_MSK	GENMASK_ULL(2, 0)

/* Device constants */
#define AD9910_MAX_SYSCLK_HZ		(1000UL * HZ_PER_MHZ)

#define AD9910_POW_SCALE(x)		((x) << 16)
#define AD9910_ASF_MAX			(BIT(15) - 1)
#define AD9910_NUM_PROFILES		8

/* PLL constants */
#define AD9910_PLL_MIN_N		12
#define AD9910_PLL_MAX_N		127

#define AD9910_PLL_IN_MIN_FREQ_HZ	(3200UL * HZ_PER_KHZ)
#define AD9910_PLL_IN_MAX_FREQ_HZ	(60000UL * HZ_PER_KHZ)

#define AD9910_PLL_OUT_MIN_FREQ_HZ	(420UL * HZ_PER_MHZ)
#define AD9910_PLL_OUT_MAX_FREQ_HZ	(1000UL * HZ_PER_MHZ)

#define AD9910_VCO0_RANGE_AUTO_MAX_HZ	(465UL * HZ_PER_MHZ)
#define AD9910_VCO1_RANGE_AUTO_MAX_HZ	(545UL * HZ_PER_MHZ)
#define AD9910_VCO2_RANGE_AUTO_MAX_HZ	(650UL * HZ_PER_MHZ)
#define AD9910_VCO3_RANGE_AUTO_MAX_HZ	(790UL * HZ_PER_MHZ)
#define AD9910_VCO4_RANGE_AUTO_MAX_HZ	(885UL * HZ_PER_MHZ)
#define AD9910_VCO_RANGE_NUM		6

#define AD9910_ICP_MIN_uA		212
#define AD9910_ICP_MAX_uA		387

#define AD9910_DAC_IOUT_MAX_uA		31590
#define AD9910_DAC_IOUT_DEFAULT_uA	20070
#define AD9910_DAC_IOUT_MIN_uA		8640

#define AD9910_REFDIV2_MIN_FREQ_HZ	(120UL * HZ_PER_MHZ)
#define AD9910_REFDIV2_MAX_FREQ_HZ	(1900UL * HZ_PER_MHZ)

#define AD9910_DEST_FREQUENCY		0
#define AD9910_DEST_PHASE		1
#define AD9910_DEST_AMPLITUDE		2
#define AD9910_DEST_POLAR		3

#define AD9910_DRG_DEST_NUM		3
#define AD9910_RAM_DEST_NUM		4

#define AD9910_DRG_MODE_RAMP_DOWN	0x0
#define AD9910_DRG_MODE_RAMP_UP		0x1
#define AD9910_DRG_MODE_RAMP_BIDIR	0x2

#define AD9910_CHANNEL_SINGLE_TONE	0
#define AD9910_CHANNEL_OSK		1
#define AD9910_CHANNEL_DRG		2
#define AD9910_CHANNEL_RAM		3
#define AD9910_CHANNEL_PARALLEL_PORT	4
#define AD9910_CHANNEL_SHARED		5

#define AD9910_SPI_READ			BIT(7)
#define AD9910_SPI_ADDR_MASK		GENMASK(4, 0)

enum {
	AD9910_PROFILE,
	AD9910_POWERDOWN,
	AD9910_SYSCLK_FREQUENCY,

	AD9910_OSK_RAMP_RATE,
	AD9910_OSK_STEP,

	AD9910_DRG_FREQ_UPPER_LIMIT,
	AD9910_DRG_PHASE_UPPER_LIMIT,
	AD9910_DRG_AMP_UPPER_LIMIT,
	AD9910_DRG_FREQ_LOWER_LIMIT,
	AD9910_DRG_PHASE_LOWER_LIMIT,
	AD9910_DRG_AMP_LOWER_LIMIT,
	AD9910_DRG_FREQ_INC_STEP,
	AD9910_DRG_PHASE_INC_STEP,
	AD9910_DRG_AMP_INC_STEP,
	AD9910_DRG_FREQ_DEC_STEP,
	AD9910_DRG_PHASE_DEC_STEP,
	AD9910_DRG_AMP_DEC_STEP,
	AD9910_DRG_INC_STEP_RATE,
	AD9910_DRG_DEC_STEP_RATE,
};

struct ad9910_data {
	/* PLL configuration */
	u8 pll_multiplier;
	u8 pll_vco_range;
	u16 pll_charge_pump_current;

	bool ref_div2_en;
	u8 refclk_out_drv;

	/* Feature flags */
	bool inverse_sinc_enable;
	bool select_sine_output;
	bool sync_clk_enable;
	bool pdclk_enable;
	bool pdclk_invert;
	bool tx_enable_invert;

	/* DAC configuration */
	u32 dac_output_current;
};

struct ad9910_state {
	struct spi_device *spi;
	struct clk *refclk;

	struct gpio_desc *gpio_powerdown;
	struct gpio_desc *gpio_m_reset;
	struct gpio_desc *gpio_io_reset;
	struct gpio_desc *gpio_io_update;
	struct gpio_desc *gpio_profile[3];
	struct gpio_desc *gpio_drctl;
	struct gpio_desc *gpio_drover;
	struct gpio_desc *gpio_drhold;

	int irq_drover;
	u8 drg_oper_mode;
	u8 profile_active;

	u32 reg_cfr1;
	u32 reg_cfr2;
	u32 reg_ftw;
	u16 reg_pow;
	u32 reg_asf;

	u64 reg_drg_limit;
	u64 reg_drg_step;
	u32 reg_drg_rate;

	u64 reg_profiles_st[8];

	u32 sysclk_hz;

	struct ad9910_data data;

	struct mutex lock;
};

static const char * const ad9910_power_supplies[] = {
	"dvdd-io33", "avdd33", "dvdd18", "avdd18",
};

static const char * const ad9910_channel_str[] = {
	[AD9910_CHANNEL_SINGLE_TONE] = "single_tone",
	[AD9910_CHANNEL_OSK] = "output_shift_keying",
	[AD9910_CHANNEL_DRG] = "digital_ramp_generator",
	[AD9910_CHANNEL_RAM] = "ram",
	[AD9910_CHANNEL_PARALLEL_PORT] = "parallel_port",
	[AD9910_CHANNEL_SHARED] = "shared"
};

static const char * const ad9910_destination_str[] = {
	[AD9910_DEST_FREQUENCY] = "frequency",
	[AD9910_DEST_PHASE] = "phase",
	[AD9910_DEST_AMPLITUDE] = "amplitude",
	[AD9910_DEST_POLAR] = "polar"
};

static const char * const ad9910_drg_oper_mode_str[] = {
	[AD9910_DRG_MODE_RAMP_DOWN] = "ramp_down",
	[AD9910_DRG_MODE_RAMP_UP] = "ramp_up",
	[AD9910_DRG_MODE_RAMP_BIDIR] = "bidirectional"
};

static const u16 ad9910_charge_pump_currents[] = {
	AD9910_ICP_MIN_uA, 237, 262, 287, 312, 337, 363, AD9910_ICP_MAX_uA
};

static int ad9910_io_update(struct ad9910_state *st)
{
	if (st->gpio_io_update) {
		gpiod_set_value_cansleep(st->gpio_io_update, 1);
		udelay(1);
		gpiod_set_value_cansleep(st->gpio_io_update, 0);
	}

	return 0;
}

static int ad9910_spi_read(struct ad9910_state *st, u8 reg, void *data, size_t len)
{
	u8 inst = AD9910_SPI_READ | (reg & AD9910_SPI_ADDR_MASK);

	struct spi_transfer t[] = {
		{ .tx_buf = &inst, .len = 1, },
		{ .rx_buf = data, .len = len, },
	};

	return spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
}

static int ad9910_spi_write(struct ad9910_state *st, u8 reg, const void *data,
			    size_t len, bool update)
{
	u8 inst = reg & AD9910_SPI_ADDR_MASK;
	int ret;

	struct spi_transfer t[] = {
		{ .tx_buf = &inst, .len = 1, },
		{ .tx_buf = data, .len = len, },
	};

	ret = spi_sync_transfer(st->spi, t, ARRAY_SIZE(t));
	if (!ret && update)
		return ad9910_io_update(st);

	return ret;
}

static inline int ad9910_spi_read16(struct ad9910_state *st, u8 reg, u16 *data)
{
	int ret;
	__be16 result __aligned(IIO_DMA_MINALIGN);

	ret = ad9910_spi_read(st, reg, &result, sizeof(result));
	if (ret < 0)
		return ret;

	*data = be16_to_cpu(result);
	return ret;
}

static inline int ad9910_spi_write16(struct ad9910_state *st, u8 reg, u16 data,
				     bool update)
{
	int ret;
	__be16 value __aligned(IIO_DMA_MINALIGN) = cpu_to_be16(data);

	ret = ad9910_spi_write(st, reg, &value, sizeof(value), update);
	if (ret < 0)
		return ret;

	dev_dbg(&st->spi->dev, "REG[%d] <= 0x%X\n", reg, data);
	return ret;
}

static inline int ad9910_spi_read32(struct ad9910_state *st, u8 reg, u32 *data)
{
	int ret;
	__be32 result __aligned(IIO_DMA_MINALIGN);

	ret = ad9910_spi_read(st, reg, &result, sizeof(result));
	if (ret < 0)
		return ret;

	*data = be32_to_cpu(result);
	return ret;
}

static inline int ad9910_spi_write32(struct ad9910_state *st, u8 reg, u32 data,
				     bool update)
{
	int ret;
	__be32 value __aligned(IIO_DMA_MINALIGN) = cpu_to_be32(data);

	ret = ad9910_spi_write(st, reg, &value, sizeof(value), update);
	if (ret < 0)
		return ret;

	dev_dbg(&st->spi->dev, "REG[%d] <= 0x%X\n", reg, data);
	return ret;
}

static inline int ad9910_spi_read64(struct ad9910_state *st, u8 reg, u64 *data)
{
	int ret;
	__be64 result __aligned(IIO_DMA_MINALIGN);

	ret = ad9910_spi_read(st, reg, &result, sizeof(result));
	if (ret < 0)
		return ret;

	*data = be64_to_cpu(result);
	return ret;
}

static inline int ad9910_spi_write64(struct ad9910_state *st, u8 reg, u64 data,
				     bool update)
{
	int ret;
	__be64 value __aligned(IIO_DMA_MINALIGN) = cpu_to_be64(data);

	ret = ad9910_spi_write(st, reg, &value, sizeof(value), update);
	if (ret < 0)
		return ret;

	dev_dbg(&st->spi->dev, "REG[%d] <= 0x%llX\n", reg, data);
	return ret;
}

static irqreturn_t ad9910_interrupt(int irq, void *private)
{
	struct iio_dev *indio_dev = private;
	struct ad9910_state *st = iio_priv(indio_dev);

	if (st->irq_drover == irq && st->drg_oper_mode & AD9910_DRG_MODE_RAMP_BIDIR)
	{
		/* In bidirectional mode, toggle the DRCTL pin on DROVER */
		if (st->gpio_drctl) {
			st->drg_oper_mode ^= AD9910_DRG_MODE_RAMP_UP;
			gpiod_set_value(st->gpio_drctl,
					st->drg_oper_mode & AD9910_DRG_MODE_RAMP_UP);
		}
	}

	return IRQ_HANDLED;
}

static int ad9910_set_profile(struct ad9910_state *st, u8 profile)
{
	if (profile >= AD9910_NUM_PROFILES)
		return -EINVAL;

	if (st->gpio_profile[0])
		gpiod_set_value_cansleep(st->gpio_profile[0], profile & 0x01);
	if (st->gpio_profile[1])
		gpiod_set_value_cansleep(st->gpio_profile[1], (profile >> 1) & 0x01);
	if (st->gpio_profile[2])
		gpiod_set_value_cansleep(st->gpio_profile[2], (profile >> 2) & 0x01);

	st->profile_active = profile;

	dev_dbg(&st->spi->dev, "selected profile %d\n", profile);
	return 0;
}

static int ad9910_set_drg_destination(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan,
				      unsigned int val)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	u32 cfr2;
	int ret;

	guard(mutex)(&st->lock);

	cfr2 = st->reg_cfr2 & ~AD9910_CFR2_DRG_DEST_MSK;
	cfr2 |= FIELD_PREP(AD9910_CFR2_DRG_DEST_MSK, val);

	ret = ad9910_spi_write32(st, AD9910_REG_CFR2, cfr2, true);
	if (ret)
		return ret;

	st->reg_cfr2 = cfr2;

	return 0;
}

static int ad9910_get_drg_destination(struct iio_dev *indio_dev,
				      const struct iio_chan_spec *chan)
{
	struct ad9910_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	return FIELD_GET(AD9910_CFR2_DRG_DEST_MSK, st->reg_cfr2);
}

static int ad9910_set_drg_oper_mode(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan,
				    unsigned int val)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	int ret;
	u32 val32;

	guard(mutex)(&st->lock);

	if (!st->gpio_drctl)
		return -ENOTSUPP;

	if (val & AD9910_DRG_MODE_RAMP_BIDIR) {
		if (!st->gpio_drover || st->irq_drover < 0)
			return -ENOTSUPP;
		else if (!(st->drg_oper_mode & AD9910_DRG_MODE_RAMP_BIDIR)) {
			enable_irq(st->irq_drover);

			val32 = st->reg_cfr2 & ~AD9910_CFR2_DRG_NO_DWELL_HIGH_MSK;
			val32 &= ~AD9910_CFR2_DRG_NO_DWELL_LOW_MSK;
			ret = ad9910_spi_write32(st, AD9910_REG_CFR2, val32, true);
			if (ret)
				return ret;
			st->reg_cfr2 = val32;
		}
	} else if (st->drg_oper_mode & AD9910_DRG_MODE_RAMP_BIDIR) {
		if (st->gpio_drover && st->irq_drover >= 0) {
			disable_irq(st->irq_drover);

			val32 = st->reg_cfr2 | AD9910_CFR2_DRG_NO_DWELL_HIGH_MSK;
			val32 |= AD9910_CFR2_DRG_NO_DWELL_LOW_MSK;
			ret = ad9910_spi_write32(st, AD9910_REG_CFR2, val32, true);
			if (ret)
				return ret;
			st->reg_cfr2 = val32;
		}
	}

	ret = gpiod_set_value_cansleep(st->gpio_drctl, val & AD9910_DRG_MODE_RAMP_UP);
	st->drg_oper_mode = val;

	return ret;
}

static int ad9910_get_drg_oper_mode(struct iio_dev *indio_dev,
				    const struct iio_chan_spec *chan)
{
	struct ad9910_state *st = iio_priv(indio_dev);

	guard(mutex)(&st->lock);

	if (!st->gpio_drctl)
		return -ENOTSUPP;

	if (st->drg_oper_mode & AD9910_DRG_MODE_RAMP_BIDIR)
		return AD9910_DRG_MODE_RAMP_BIDIR;

	return st->drg_oper_mode;
}

static ssize_t ad9910_read(struct iio_dev *indio_dev,
			   uintptr_t private,
			   const struct iio_chan_spec *chan,
			   char *buf)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	int val;

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_PROFILE:
		val = st->profile_active;
		break;
	case AD9910_POWERDOWN:
		val = !!FIELD_GET(AD9910_CFR1_SOFT_POWER_DOWN_MSK, st->reg_cfr1);
		break;
	case AD9910_OSK_STEP:
		if (FIELD_GET(AD9910_CFR1_SELECT_AUTO_OSK_MSK, st->reg_cfr1))
			val = BIT(FIELD_GET(AD9910_ASF_AMP_STEP_SIZE_MSK, st->reg_asf));
		else
			val = 0;
		break;
	case AD9910_DRG_AMP_UPPER_LIMIT:
		val = FIELD_GET(AD9910_DRG_LIMIT_UPPER_MSK, st->reg_drg_limit);
		break;
	case AD9910_DRG_AMP_LOWER_LIMIT:
		val = FIELD_GET(AD9910_DRG_LIMIT_LOWER_MSK, st->reg_drg_limit);
		break;
	case AD9910_DRG_AMP_INC_STEP:
		val = FIELD_GET(AD9910_DRG_STEP_INC_MSK, st->reg_drg_step);
		break;
	case AD9910_DRG_AMP_DEC_STEP:
		val = FIELD_GET(AD9910_DRG_STEP_DEC_MSK, st->reg_drg_step);
		break;
	case AD9910_SYSCLK_FREQUENCY:
		val = st->sysclk_hz;
		break;
	default:
		return -EINVAL;
	}

	return iio_format_value(buf, IIO_VAL_INT, 1, &val);
}

static ssize_t ad9910_read_step_rate(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     char *buf)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	int vals[2];
	u32 tmp32;

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_OSK_RAMP_RATE:
		tmp32 = FIELD_GET(AD9910_ASF_AMP_RAMP_RATE_MSK, st->reg_asf);
		break;
	case AD9910_DRG_INC_STEP_RATE:
		tmp32 = FIELD_GET(AD9910_DRG_RATE_INC_MSK, st->reg_drg_rate);
		break;
	case AD9910_DRG_DEC_STEP_RATE:
		tmp32 = FIELD_GET(AD9910_DRG_RATE_DEC_MSK, st->reg_drg_rate);
		break;
	default:
		return -EINVAL;
	}

	if (tmp32 == 0)
		return -ERANGE;

	tmp32 *= 4;
	vals[0] = st->sysclk_hz / tmp32;
	vals[1] = div_u64((u64)(st->sysclk_hz % tmp32) * MICRO, tmp32);

	return iio_format_value(buf, IIO_VAL_INT_PLUS_MICRO, ARRAY_SIZE(vals), vals);
}

static ssize_t ad9910_read_drg_attrs(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     char *buf)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	int vals[2];

	u32 reg_val;
	u64 tmp64;

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_DRG_FREQ_UPPER_LIMIT:
		reg_val = FIELD_GET(AD9910_DRG_LIMIT_UPPER_MSK, st->reg_drg_limit);
		tmp64 = (u64)reg_val * st->sysclk_hz;
		break;
	case AD9910_DRG_PHASE_UPPER_LIMIT:
		reg_val = FIELD_GET(AD9910_DRG_LIMIT_UPPER_MSK, st->reg_drg_limit);
		tmp64 = (u64)reg_val * 360;
		break;
	case AD9910_DRG_FREQ_LOWER_LIMIT:
		reg_val = FIELD_GET(AD9910_DRG_LIMIT_LOWER_MSK, st->reg_drg_limit);
		tmp64 = (u64)reg_val * st->sysclk_hz;
		break;
	case AD9910_DRG_PHASE_LOWER_LIMIT:
		reg_val = FIELD_GET(AD9910_DRG_LIMIT_LOWER_MSK, st->reg_drg_limit);
		tmp64 = (u64)reg_val * 360;
		break;
	case AD9910_DRG_FREQ_INC_STEP:
		reg_val = FIELD_GET(AD9910_DRG_STEP_INC_MSK, st->reg_drg_step);
		tmp64 = (u64)reg_val * st->sysclk_hz;
		break;
	case AD9910_DRG_PHASE_INC_STEP:
		reg_val = FIELD_GET(AD9910_DRG_STEP_INC_MSK, st->reg_drg_step);
		tmp64 = (u64)reg_val * 360;
		break;
	case AD9910_DRG_FREQ_DEC_STEP:
		reg_val = FIELD_GET(AD9910_DRG_STEP_DEC_MSK, st->reg_drg_step);
		tmp64 = (u64)reg_val * st->sysclk_hz;
		break;
	case AD9910_DRG_PHASE_DEC_STEP:
		reg_val = FIELD_GET(AD9910_DRG_STEP_DEC_MSK, st->reg_drg_step);
		tmp64 = (u64)reg_val * 360;
		break;
	default:
		return -EINVAL;
	}

	vals[0] = tmp64 >> 32;
	vals[1] = ((tmp64 & 0xFFFFFFFFULL) * MICRO) >> 32;

	return iio_format_value(buf, IIO_VAL_INT_PLUS_MICRO, ARRAY_SIZE(vals), vals);
}

static ssize_t ad9910_write(struct iio_dev *indio_dev,
			    uintptr_t private,
			    const struct iio_chan_spec *chan,
			    const char *buf, size_t len)
{
	struct ad9910_state *st = iio_priv(indio_dev);

	u32 val32;
	u32 tmp32;
	u64 tmp64;
	int ret;

	ret = kstrtou32(buf, 10, &val32);
	if (ret)
		return ret;

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_PROFILE:
		if (val32 > 7)
			return -EINVAL;
		ret = ad9910_set_profile(st, val32);
		break;
	case AD9910_POWERDOWN:
		if (val32)
			tmp32 = st->reg_cfr1 | AD9910_CFR1_SOFT_POWER_DOWN_MSK;
		else
			tmp32 = st->reg_cfr1 & ~AD9910_CFR1_SOFT_POWER_DOWN_MSK;

		ret = ad9910_spi_write32(st, AD9910_REG_CFR1, tmp32, true);
		if (!ret)
			st->reg_cfr1 = tmp32;
		break;
	case AD9910_OSK_STEP:
		if (val32 != 0 || val32 > 8 || !is_power_of_2(val32))
			return -EINVAL;

		if (val32) {
			val32 = ilog2(val32);
			tmp32 = st->reg_asf & ~AD9910_ASF_AMP_STEP_SIZE_MSK;
			tmp32 |= FIELD_PREP(AD9910_ASF_AMP_STEP_SIZE_MSK, val32);
			ret = ad9910_spi_write32(st, AD9910_REG_ASF, tmp32, true);
			if (ret)
				return ret;

			st->reg_asf = tmp32;
			tmp32 = st->reg_cfr1 | AD9910_CFR1_SELECT_AUTO_OSK_MSK;
			ret = ad9910_spi_write32(st, AD9910_REG_CFR1, tmp32, true);
			if (!ret)
				st->reg_cfr1 = tmp32;
		} else {
			tmp32 = st->reg_cfr1 & ~AD9910_CFR1_SELECT_AUTO_OSK_MSK;
			ret = ad9910_spi_write32(st, AD9910_REG_CFR1, tmp32, true);
			if (!ret)
				st->reg_cfr1 = tmp32;
		}
		break;
	case AD9910_DRG_AMP_UPPER_LIMIT:
		tmp64 = st->reg_drg_limit & ~AD9910_DRG_LIMIT_UPPER_MSK;
		tmp64 |= FIELD_PREP(AD9910_DRG_LIMIT_UPPER_MSK, val32);

		ret = ad9910_spi_write64(st, AD9910_REG_DRG_LIMIT, tmp64, true);
		if (!ret)
			st->reg_drg_limit = tmp64;
		break;
	case AD9910_DRG_AMP_LOWER_LIMIT:
		tmp64 = st->reg_drg_limit & ~AD9910_DRG_LIMIT_LOWER_MSK;
		tmp64 |= FIELD_PREP(AD9910_DRG_LIMIT_LOWER_MSK, val32);

		ret = ad9910_spi_write64(st, AD9910_REG_DRG_LIMIT, tmp64, true);
		if (!ret)
			st->reg_drg_limit = tmp64;
		break;
	case AD9910_DRG_AMP_INC_STEP:
		tmp64 = st->reg_drg_step & ~AD9910_DRG_STEP_INC_MSK;
		tmp64 |= FIELD_PREP(AD9910_DRG_STEP_INC_MSK, val32);

		ret = ad9910_spi_write64(st, AD9910_REG_DRG_STEP, tmp64, true);
		if (!ret)
			st->reg_drg_step = tmp64;
		break;
	case AD9910_DRG_AMP_DEC_STEP:
		tmp64 = st->reg_drg_step & ~AD9910_DRG_STEP_DEC_MSK;
		tmp64 |= FIELD_PREP(AD9910_DRG_STEP_DEC_MSK, val32);

		ret = ad9910_spi_write64(st, AD9910_REG_DRG_STEP, tmp64, true);
		if (!ret)
			st->reg_drg_step = tmp64;
		break;
	default:
		return -EINVAL;
	}

	return ret ? ret : len;
}

static ssize_t ad9910_write_step_rate(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      const char *buf, size_t len)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	int val, val2;
	u32 reg_val;
	u64 rate_val;
	int ret;

	ret = iio_str_to_fixpoint(buf, 100000, &val, &val2);
	if (ret)
		return ret;

	if (val == 0 && val2 == 0)
		return -EINVAL;

	rate_val = (u64)st->sysclk_hz * MICRO;
	rate_val = DIV64_U64_ROUND_CLOSEST(rate_val, ((u64)val * MICRO + val2) * 4);

	if (rate_val == 0 || rate_val >= BIT_ULL(16))
		return -EINVAL;

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_OSK_RAMP_RATE:
		reg_val = st->reg_asf & ~AD9910_ASF_AMP_RAMP_RATE_MSK;
		reg_val |= FIELD_PREP(AD9910_ASF_AMP_RAMP_RATE_MSK, rate_val);

		ret = ad9910_spi_write32(st, AD9910_REG_ASF, reg_val, true);
		if (!ret)
			st->reg_asf = reg_val;
		break;
	case AD9910_DRG_INC_STEP_RATE:
		reg_val = st->reg_drg_rate & ~AD9910_DRG_RATE_INC_MSK;
		reg_val |= FIELD_PREP(AD9910_DRG_RATE_INC_MSK, rate_val);

		ret = ad9910_spi_write32(st, AD9910_REG_DRG_RATE, reg_val, true);
		if (!ret)
			st->reg_drg_rate = reg_val;
		break;
	case AD9910_DRG_DEC_STEP_RATE:
		reg_val = st->reg_drg_rate & ~AD9910_DRG_RATE_DEC_MSK;
		reg_val |= FIELD_PREP(AD9910_DRG_RATE_DEC_MSK, rate_val);

		ret = ad9910_spi_write32(st, AD9910_REG_DRG_RATE, reg_val, true);
		if (!ret)
			st->reg_drg_rate = reg_val;
		break;
	default:
		return -EINVAL;
	}

	return ret ? ret : len;
}

static ssize_t ad9910_write_drg_phase(struct iio_dev *indio_dev,
				      uintptr_t private,
				      const struct iio_chan_spec *chan,
				      const char *buf, size_t len)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	int val, val2;
	u32 tmp32;
	u64 tmp64;
	int ret;

	ret = iio_str_to_fixpoint(buf, 100000, &val, &val2);
	if (ret)
		return ret;

	val %= 360;
	if (val < 0)
		val += 360;

	tmp64 = mul_u64_u64_div_u64((u64)val * MICRO + val2, BIT_ULL(33),
					(u64)MICRO * 360);
	tmp32 = (tmp64 + 1) >> 1;

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_DRG_PHASE_UPPER_LIMIT:
		tmp64 = st->reg_drg_limit & ~AD9910_DRG_LIMIT_UPPER_MSK;
		tmp64 |= FIELD_PREP(AD9910_DRG_LIMIT_UPPER_MSK, tmp32);

		ret = ad9910_spi_write64(st, AD9910_REG_DRG_LIMIT, tmp64, true);
		if (!ret)
			st->reg_drg_limit = tmp64;
		break;
	case AD9910_DRG_PHASE_LOWER_LIMIT:
		tmp64 = st->reg_drg_limit & ~AD9910_DRG_LIMIT_LOWER_MSK;
		tmp64 |= FIELD_PREP(AD9910_DRG_LIMIT_LOWER_MSK, tmp32);

		ret = ad9910_spi_write64(st, AD9910_REG_DRG_LIMIT, tmp64, true);
		if (!ret)
			st->reg_drg_limit = tmp64;
		break;
	case AD9910_DRG_PHASE_INC_STEP:
		tmp64 = st->reg_drg_step & ~AD9910_DRG_STEP_INC_MSK;
		tmp64 |= FIELD_PREP(AD9910_DRG_STEP_INC_MSK, tmp32);

		ret = ad9910_spi_write64(st, AD9910_REG_DRG_STEP, tmp64, true);
		if (!ret)
			st->reg_drg_step = tmp64;
		break;
	case AD9910_DRG_PHASE_DEC_STEP:
		tmp64 = st->reg_drg_step & ~AD9910_DRG_STEP_DEC_MSK;
		tmp64 |= FIELD_PREP(AD9910_DRG_STEP_DEC_MSK, tmp32);

		ret = ad9910_spi_write64(st, AD9910_REG_DRG_STEP, tmp64, true);
		if (!ret)
			st->reg_drg_step = tmp64;
		break;
	default:
		return -EINVAL;
	}

	return ret ? ret : len;
}

static ssize_t ad9910_write_drg_freq(struct iio_dev *indio_dev,
				     uintptr_t private,
				     const struct iio_chan_spec *chan,
				     const char *buf, size_t len)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	int val, val2;
	u32 tmp32;
	u64 tmp64;
	int ret;

	ret = iio_str_to_fixpoint(buf, 100000, &val, &val2);
	if (ret)
		return ret;

	if (val >= st->sysclk_hz / 2)
		return -EINVAL;

	tmp64 = mul_u64_u64_div_u64((u64)val * MICRO + val2, BIT_ULL(33),
					(u64)MICRO * st->sysclk_hz);
	tmp32 = (tmp64 + 1) >> 1;

	guard(mutex)(&st->lock);

	switch (private) {
	case AD9910_DRG_FREQ_UPPER_LIMIT:
		tmp64 = st->reg_drg_limit & ~AD9910_DRG_LIMIT_UPPER_MSK;
		tmp64 |= FIELD_PREP(AD9910_DRG_LIMIT_UPPER_MSK, tmp32);

		ret = ad9910_spi_write64(st, AD9910_REG_DRG_LIMIT, tmp64, true);
		if (!ret)
			st->reg_drg_limit = tmp64;
		break;
	case AD9910_DRG_FREQ_LOWER_LIMIT:
		tmp64 = st->reg_drg_limit & ~AD9910_DRG_LIMIT_LOWER_MSK;
		tmp64 |= FIELD_PREP(AD9910_DRG_LIMIT_LOWER_MSK, tmp32);

		ret = ad9910_spi_write64(st, AD9910_REG_DRG_LIMIT, tmp64, true);
		if (!ret)
			st->reg_drg_limit = tmp64;
		break;
	case AD9910_DRG_FREQ_INC_STEP:
		tmp64 = st->reg_drg_step & ~AD9910_DRG_STEP_INC_MSK;
		tmp64 |= FIELD_PREP(AD9910_DRG_STEP_INC_MSK, tmp32);

		ret = ad9910_spi_write64(st, AD9910_REG_DRG_STEP, tmp64, true);
		if (!ret)
			st->reg_drg_step = tmp64;
		break;
	case AD9910_DRG_FREQ_DEC_STEP:
		tmp64 = st->reg_drg_step & ~AD9910_DRG_STEP_DEC_MSK;
		tmp64 |= FIELD_PREP(AD9910_DRG_STEP_DEC_MSK, tmp32);

		ret = ad9910_spi_write64(st, AD9910_REG_DRG_STEP, tmp64, true);
		if (!ret)
			st->reg_drg_step = tmp64;
		break;
	default:
		return -EINVAL;
	}

	return ret ? ret : len;
}

static const struct iio_enum ad9910_drg_destination_enum = {
	.items = ad9910_destination_str,
	.num_items = AD9910_DRG_DEST_NUM,
	.set = ad9910_set_drg_destination,
	.get = ad9910_get_drg_destination,
};

static const struct iio_enum ad9910_drg_oper_mode_enum = {
	.items = ad9910_drg_oper_mode_str,
	.num_items = ARRAY_SIZE(ad9910_drg_oper_mode_str),
	.set = ad9910_set_drg_oper_mode,
	.get = ad9910_get_drg_oper_mode,
};

#define AD9910_EXT_INFO(_name, _ident, _shared) { \
	.name = _name, \
	.read = ad9910_read, \
	.write = ad9910_write, \
	.private = _ident, \
	.shared = _shared, \
}

#define AD9910_STEP_RATE_EXT_INFO(_name, _ident) { \
	.name = _name, \
	.read = ad9910_read_step_rate, \
	.write = ad9910_write_step_rate, \
	.private = _ident, \
	.shared = IIO_SEPARATE, \
}

#define AD9910_DRG_PHASE_EXT_INFO(_name, _ident) { \
	.name = _name, \
	.read = ad9910_read_drg_attrs, \
	.write = ad9910_write_drg_phase, \
	.private = _ident, \
	.shared = IIO_SEPARATE, \
}

#define AD9910_DRG_FREQ_EXT_INFO(_name, _ident) { \
	.name = _name, \
	.read = ad9910_read_drg_attrs, \
	.write = ad9910_write_drg_freq, \
	.private = _ident, \
	.shared = IIO_SEPARATE, \
}

static const struct iio_chan_spec_ext_info ad9910_shared_ext_info[] = {
	AD9910_EXT_INFO("profile", AD9910_PROFILE, IIO_SHARED_BY_TYPE),
	AD9910_EXT_INFO("powerdown", AD9910_POWERDOWN, IIO_SHARED_BY_TYPE),
	AD9910_EXT_INFO("sysclk_frequency", AD9910_SYSCLK_FREQUENCY, IIO_SHARED_BY_TYPE),
	{ },
};

static const struct iio_chan_spec_ext_info ad9910_osk_ext_info[] = {
	AD9910_EXT_INFO("scale_increment", AD9910_OSK_STEP, IIO_SEPARATE),
	AD9910_STEP_RATE_EXT_INFO("sampling_frequency", AD9910_OSK_RAMP_RATE),
	{ },
};

static const struct iio_chan_spec_ext_info ad9910_drg_ext_info[] = {
	IIO_ENUM("destination", IIO_SEPARATE, &ad9910_drg_destination_enum),
	IIO_ENUM_AVAILABLE("destination", IIO_SEPARATE, &ad9910_drg_destination_enum),
	IIO_ENUM("operating_mode", IIO_SEPARATE, &ad9910_drg_oper_mode_enum),
	IIO_ENUM_AVAILABLE("operating_mode", IIO_SEPARATE, &ad9910_drg_oper_mode_enum),
	AD9910_DRG_FREQ_EXT_INFO("max_frequency", AD9910_DRG_FREQ_UPPER_LIMIT),
	AD9910_DRG_FREQ_EXT_INFO("min_frequency", AD9910_DRG_FREQ_LOWER_LIMIT),
	AD9910_DRG_FREQ_EXT_INFO("frequency_increment", AD9910_DRG_FREQ_INC_STEP),
	AD9910_DRG_FREQ_EXT_INFO("frequency_decrement", AD9910_DRG_FREQ_DEC_STEP),
	AD9910_DRG_PHASE_EXT_INFO("max_phase", AD9910_DRG_PHASE_UPPER_LIMIT),
	AD9910_DRG_PHASE_EXT_INFO("min_phase", AD9910_DRG_PHASE_LOWER_LIMIT),
	AD9910_DRG_PHASE_EXT_INFO("phase_increment", AD9910_DRG_PHASE_INC_STEP),
	AD9910_DRG_PHASE_EXT_INFO("phase_decrement", AD9910_DRG_PHASE_DEC_STEP),
	AD9910_EXT_INFO("max_scale", AD9910_DRG_AMP_UPPER_LIMIT, IIO_SEPARATE),
	AD9910_EXT_INFO("min_scale", AD9910_DRG_AMP_LOWER_LIMIT, IIO_SEPARATE),
	AD9910_EXT_INFO("scale_increment", AD9910_DRG_AMP_INC_STEP, IIO_SEPARATE),
	AD9910_EXT_INFO("scale_decrement", AD9910_DRG_AMP_DEC_STEP, IIO_SEPARATE),
	AD9910_STEP_RATE_EXT_INFO("increment_sampling_frequency", AD9910_DRG_INC_STEP_RATE),
	AD9910_STEP_RATE_EXT_INFO("decrement_sampling_frequency", AD9910_DRG_DEC_STEP_RATE),
	{ },
};

static const struct iio_chan_spec ad9910_channels[] = {
	{
		.type = IIO_ALTVOLTAGE,
		.output = 1,
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_FREQUENCY) |
					    BIT(IIO_CHAN_INFO_PHASE) |
					    BIT(IIO_CHAN_INFO_SCALE),
		.ext_info = ad9910_shared_ext_info,
	},
	{
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = AD9910_CHANNEL_SINGLE_TONE,
		.info_mask_separate = BIT(IIO_CHAN_INFO_FREQUENCY) |
				      BIT(IIO_CHAN_INFO_PHASE) |
				      BIT(IIO_CHAN_INFO_SCALE),
	},
	{
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = AD9910_CHANNEL_OSK,
		.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE),
		.ext_info = ad9910_osk_ext_info,
	},
	{
		.type = IIO_ALTVOLTAGE,
		.indexed = 1,
		.output = 1,
		.channel = AD9910_CHANNEL_DRG,
		.info_mask_separate = BIT(IIO_CHAN_INFO_ENABLE),
		.ext_info = ad9910_drg_ext_info,
	},

};

static int ad9910_read_raw(struct iio_dev *indio_dev,
			   struct iio_chan_spec const *chan,
			   int *val, int *val2, long info)
{
	struct ad9910_state *st = iio_priv(indio_dev);

	u64 tmp64;
	u32 tmp32;
	u16 tmp16;

	guard(mutex)(&st->lock);

	switch (info) {
	case IIO_CHAN_INFO_ENABLE:
		switch (chan->channel) {
			case AD9910_CHANNEL_OSK:
				*val = FIELD_GET(AD9910_CFR1_OSK_ENABLE_MSK, st->reg_cfr1);
				break;
			case AD9910_CHANNEL_DRG:
				*val = FIELD_GET(AD9910_CFR2_DRG_ENABLE_MSK, st->reg_cfr2);
				break;
			default:
				return -EINVAL;
		}
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_FREQUENCY:
		if (chan->info_mask_separate & BIT(IIO_CHAN_INFO_FREQUENCY))
			tmp32 = FIELD_GET(AD9910_PROFILE_ST_FTW_MSK,
					  st->reg_profiles_st[st->profile_active]);
		else
			tmp32 = st->reg_ftw;
		tmp64 = (u64)tmp32 * st->sysclk_hz;
		*val = tmp64 >> 32;
		*val2 = (tmp64 & 0xFFFFFFFFULL) * MICRO >> 32;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_PHASE:
		if (chan->info_mask_separate & BIT(IIO_CHAN_INFO_PHASE))
			tmp16 = FIELD_GET(AD9910_PROFILE_ST_POW_MSK,
					  st->reg_profiles_st[st->profile_active]);
		else
			tmp16 = st->reg_pow;
		tmp32 = (u32)tmp16 * 360;
		*val = tmp32 >> 16;
		*val2 = ((u64)tmp32 & 0xFFFF) * MICRO >> 16;
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SCALE:
		if (chan->info_mask_separate & BIT(IIO_CHAN_INFO_SCALE)) {
			tmp16 = FIELD_GET(AD9910_PROFILE_ST_ASF_MSK,
					  st->reg_profiles_st[st->profile_active]);
		} else {
			tmp16 = FIELD_GET(AD9910_ASF_AMP_SCALE_FACTOR_MSK, st->reg_asf);
		}
		*val = tmp16;
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad9910_write_raw(struct iio_dev *indio_dev,
			    struct iio_chan_spec const *chan,
			    int val, int val2, long info)
{
	struct ad9910_state *st = iio_priv(indio_dev);

	u16 tmp16;
	u32 tmp32;
	u64 tmp64;
	int ret;
	bool ram_en = !!FIELD_GET(AD9910_CFR1_RAM_ENABLE_MSK, st->reg_cfr1);

	guard(mutex)(&st->lock);

	switch (info) {
	case IIO_CHAN_INFO_ENABLE:
		switch (chan->channel) {
			case AD9910_CHANNEL_OSK:
				tmp32 = st->reg_cfr1 & ~AD9910_CFR1_OSK_ENABLE_MSK;
				tmp32 |= FIELD_PREP(AD9910_CFR1_OSK_ENABLE_MSK, val? 1 : 0);
				ret = ad9910_spi_write32(st, AD9910_REG_CFR1, tmp32, true);
				if (!ret)
					st->reg_cfr1 = tmp32;
				break;
			case AD9910_CHANNEL_DRG:
				tmp32 = st->reg_cfr2 & ~AD9910_CFR2_DRG_ENABLE_MSK;
				tmp32 |= FIELD_PREP(AD9910_CFR2_DRG_ENABLE_MSK, val? 1 : 0);
				ret = ad9910_spi_write32(st, AD9910_REG_CFR2, tmp32, true);
				if (!ret)
					st->reg_cfr2 = tmp32;
				break;
			default:
				ret = -EINVAL;
				break;
		}
		break;
	case IIO_CHAN_INFO_FREQUENCY:
		if (val < 0 || val >= st->sysclk_hz / 2)
			return -EINVAL;

		tmp32 = mul_u64_u64_div_u64((u64)val * MICRO + val2, BIT_ULL(32),
					    (u64)MICRO * st->sysclk_hz);
		if (chan->info_mask_separate & BIT(IIO_CHAN_INFO_FREQUENCY)) {
			/* single-tone ftw */
			tmp64 = st->reg_profiles_st[st->profile_active] & ~AD9910_PROFILE_ST_FTW_MSK;
			tmp64 |= FIELD_PREP(AD9910_PROFILE_ST_FTW_MSK, tmp32);

			if (!ram_en) {
				ret = ad9910_spi_write64(st,
							 AD9910_REG_PROFILE(st->profile_active),
							 tmp64, true);
				if (ret)
					break;
			}

			st->reg_profiles_st[st->profile_active] = tmp64;
		} else {
			/* global ftw */
			ret = ad9910_spi_write32(st, AD9910_REG_FTW, tmp32, true);
			if (ret)
				break;
			st->reg_ftw = tmp32;
		}
		break;
	case IIO_CHAN_INFO_PHASE:
		val %= 360;
		if (val < 0)
			val += 360;
		tmp16 = div_u64(AD9910_POW_SCALE((u64)val * MICRO + val2), MICRO * 360);
		if (chan->info_mask_separate & BIT(IIO_CHAN_INFO_PHASE)) {
			/* single-tone pow */
			tmp64 = st->reg_profiles_st[st->profile_active] & ~AD9910_PROFILE_ST_POW_MSK;
			tmp64 |= FIELD_PREP(AD9910_PROFILE_ST_POW_MSK, tmp16);

			if (!ram_en) {
				ret = ad9910_spi_write64(st,
							 AD9910_REG_PROFILE(st->profile_active),
							 tmp64, true);
				if (ret)
					break;
			}

			st->reg_profiles_st[st->profile_active] = tmp64;
		} else {
			/* global pow */
			ret = ad9910_spi_write16(st, AD9910_REG_POW, tmp16, true);
			if (ret)
				break;
			st->reg_pow = tmp16;
		}
		break;
	case IIO_CHAN_INFO_SCALE:
		if (val < 0 || val > AD9910_ASF_MAX)
			return -EINVAL;

		if (chan->info_mask_separate & BIT(IIO_CHAN_INFO_SCALE)) {
			/* single-tone asf */
			tmp64 = st->reg_profiles_st[st->profile_active] & ~AD9910_PROFILE_ST_ASF_MSK;
			tmp64 |= FIELD_PREP(AD9910_PROFILE_ST_ASF_MSK, val);

			if (!ram_en) {
				ret = ad9910_spi_write64(st,
							 AD9910_REG_PROFILE(st->profile_active),
							 tmp64, true);
				if (ret)
					break;
			}

			st->reg_profiles_st[st->profile_active] = tmp64;
		} else {
			/* global asf */
			tmp32 = st->reg_asf & ~AD9910_ASF_AMP_SCALE_FACTOR_MSK;
			tmp32 |= FIELD_PREP(AD9910_ASF_AMP_SCALE_FACTOR_MSK, val);

			ret = ad9910_spi_write32(st, AD9910_REG_ASF, tmp32, true);
			if (!ret)
				st->reg_asf = tmp32;
		}
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static int ad9910_write_raw_get_fmt(struct iio_dev *indio_dev,
				    struct iio_chan_spec const *chan,
				    long mask)
{
	switch (mask) {
	case IIO_CHAN_INFO_ENABLE:
		return IIO_VAL_INT;
	case IIO_CHAN_INFO_FREQUENCY:
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_PHASE:
		return IIO_VAL_INT_PLUS_MICRO;
	case IIO_CHAN_INFO_SCALE:
		return IIO_VAL_INT;
	default:
		return -EINVAL;
	}
}

static int ad9910_reg_access(struct iio_dev *indio_dev,
			     unsigned int reg,
			     unsigned int writeval,
			     unsigned int *readval)
{
	struct ad9910_state *st = iio_priv(indio_dev);
	int ret;
	u64 tmp64;
	u32 tmp32;
	u16 tmp16;

	bool high32 = !!(reg & AD9910_REG_HIGH32);
	reg &= ~AD9910_REG_HIGH32;

	if (reg > AD9910_REG_RAM)
		return -EINVAL;

	guard(mutex)(&st->lock);

	/* set profile pins when reading profile registers */
	if (readval && reg >= AD9910_REG_PROFILE0 && reg <= AD9910_REG_PROFILE7)
	{
		ret = ad9910_set_profile(st, reg - AD9910_REG_PROFILE0);
		if (ret < 0)
			return ret;
	}

	switch (reg) {
	case AD9910_REG_DRG_LIMIT:
	case AD9910_REG_DRG_STEP:
	case AD9910_REG_PROFILE0:
	case AD9910_REG_PROFILE1:
	case AD9910_REG_PROFILE2:
	case AD9910_REG_PROFILE3:
	case AD9910_REG_PROFILE4:
	case AD9910_REG_PROFILE5:
	case AD9910_REG_PROFILE6:
	case AD9910_REG_PROFILE7:
		ret = ad9910_spi_read64(st, reg, &tmp64);
		if (ret < 0)
			return ret;

		if (readval) {
			if (high32)
				*readval = tmp64 >> 32;
			else
				*readval = tmp64 & 0xFFFFFFFFULL;
		} else {
			if (high32) {
				tmp64 &= 0xFFFFFFFF00000000ULL;
				tmp64 |= (u64)writeval << 32;
			} else {
				tmp64 &= 0xFFFFFFFFULL;
				tmp64 |= (u64)writeval;
			}
			ret = ad9910_spi_write64(st, reg, tmp64, true);
		}
		break;
	case AD9910_REG_POW:
		if (readval) {
			ret = ad9910_spi_read16(st, reg, &tmp16);
			if (ret < 0)
				return ret;

			*readval = tmp16;
		} else {
			tmp16 = writeval;
			ret = ad9910_spi_write16(st, reg, tmp16, true);
		}
		break;
	default:
		if (readval) {
			ret = ad9910_spi_read32(st, reg, &tmp32);
			if (ret < 0)
				return ret;

			*readval = tmp32;
		} else {
			tmp32 = writeval;
			ret = ad9910_spi_write32(st, reg, tmp32, true);
		}
		break;
	}

	return ret;
}

static int ad9910_read_label(struct iio_dev *indio_dev,
			     const struct iio_chan_spec *chan,
			     char *label)
{
	const char *label_str = chan->indexed ? ad9910_channel_str[chan->channel] :
						ad9910_channel_str[AD9910_CHANNEL_SHARED];

	return sysfs_emit(label, "%s\n", label_str);
}

static const struct iio_info ad9910_info = {
	.read_raw = ad9910_read_raw,
	.write_raw = ad9910_write_raw,
	.write_raw_get_fmt = ad9910_write_raw_get_fmt,
	.read_label = ad9910_read_label,
	.debugfs_reg_access = &ad9910_reg_access,
};

static int ad9910_set_dac_current(struct ad9910_state *st, bool update)
{
	u32 fsc_code;

	/* FSC = (86.4 / Rset) * (1 + CODE/256) where Rset = 10k ohms */
	fsc_code = ((st->data.dac_output_current / 90) - 96);
	fsc_code &= 0xFFU;

	return ad9910_spi_write32(st, AD9910_REG_AUX_DAC, fsc_code, update);
}

static int ad9910_cfg_sysclk(struct ad9910_state *st, bool update)
{
	u32 cp_index;
	u32 cfr3 = AD9910_CFR3_OPEN_MSK;

	cfr3 |= FIELD_PREP(AD9910_CFR3_DRV0_MSK, st->data.refclk_out_drv);
	st->sysclk_hz = clk_get_rate(st->refclk);

	if (st->data.pll_multiplier) {
		st->sysclk_hz *= st->data.pll_multiplier;
		if (st->sysclk_hz < AD9910_PLL_OUT_MIN_FREQ_HZ ||
		    st->sysclk_hz > AD9910_PLL_OUT_MAX_FREQ_HZ) {
			dev_err(&st->spi->dev, "invalid vco frequency: %u Hz\n", st->sysclk_hz);
			return -ERANGE;
		}

		if (st->data.pll_vco_range >= AD9910_VCO_RANGE_NUM) {
			if (st->sysclk_hz <= AD9910_VCO0_RANGE_AUTO_MAX_HZ)
				st->data.pll_vco_range = 0;
			else if (st->sysclk_hz <= AD9910_VCO1_RANGE_AUTO_MAX_HZ)
				st->data.pll_vco_range = 1;
			else if (st->sysclk_hz <= AD9910_VCO2_RANGE_AUTO_MAX_HZ)
				st->data.pll_vco_range = 2;
			else if (st->sysclk_hz <= AD9910_VCO3_RANGE_AUTO_MAX_HZ)
				st->data.pll_vco_range = 3;
			else if (st->sysclk_hz <= AD9910_VCO4_RANGE_AUTO_MAX_HZ)
				st->data.pll_vco_range = 4;
			else
				st->data.pll_vco_range = 5;
			dev_dbg(&st->spi->dev, "auto-selected VCO range: %u\n",
				st->data.pll_vco_range);
		}

		cp_index = find_closest(st->data.pll_charge_pump_current,
					ad9910_charge_pump_currents,
					ARRAY_SIZE(ad9910_charge_pump_currents));
		cfr3 |= FIELD_PREP(AD9910_CFR3_VCO_SEL_MSK, st->data.pll_vco_range) |
			FIELD_PREP(AD9910_CFR3_ICP_MSK, cp_index) |
			FIELD_PREP(AD9910_CFR3_N_MSK, st->data.pll_multiplier) |
			AD9910_CFR3_PLL_EN_MSK;
	} else {
		cfr3 |= AD9910_CFR3_VCO_SEL_MSK |
			AD9910_CFR3_ICP_MSK |
			AD9910_CFR3_REFCLK_IN_DIV_RESETB_MSK |
			FIELD_PREP(AD9910_CFR3_REFCLK_IN_DIV_BYPASS_MSK, !st->data.ref_div2_en) |
			AD9910_CFR3_PFD_RESET_MSK;
		if (st->data.ref_div2_en)
			st->sysclk_hz >>= 1;
	}

	return ad9910_spi_write32(st, AD9910_REG_CFR3, cfr3, update);
}

static int ad9910_parse_fw(struct ad9910_state *st)
{
	struct device *dev = &st->spi->dev;
	u32 tmp;
	int ret;

	/* PLL configuration */
	st->data.pll_multiplier = 0;
	ret = device_property_read_u32(dev, "adi,pll-multiplier", &tmp);
	if (!ret) {
		if (tmp < AD9910_PLL_MIN_N && tmp > AD9910_PLL_MAX_N)
			return dev_err_probe(dev, -ERANGE,
						"invalid PLL multiplier %u\n", tmp);
		else
			st->data.pll_multiplier = tmp;
	}

	if (st->data.pll_multiplier) {
		st->data.pll_vco_range = AD9910_VCO_RANGE_NUM;
		ret = device_property_read_u32(dev, "adi,pll-vco-range", &tmp);
		if (!ret) {
			if (tmp >= AD9910_VCO_RANGE_NUM)
				dev_err_probe(dev, -ERANGE,
					      "invalid VCO range: %u\n", tmp);
			else
				st->data.pll_vco_range = (u8)tmp;
		}

		st->data.pll_charge_pump_current = AD9910_ICP_MAX_uA;
		ret = device_property_read_u32(dev, "adi,charge-pump-current-microamp", &tmp);
		if (!ret) {
			if (tmp < AD9910_ICP_MIN_uA && tmp > AD9910_ICP_MAX_uA)
				return dev_err_probe(dev, -ERANGE,
						     "invalid charge pump current %u\n", tmp);
			st->data.pll_charge_pump_current = tmp;
		}
	} else {
		st->data.ref_div2_en = device_property_read_bool(dev, "adi,reference-div2-enable");
	}

	/* Feature flags */
	st->data.inverse_sinc_enable = device_property_read_bool(dev, "adi,inverse-sinc-enable");
	st->data.select_sine_output = device_property_read_bool(dev, "adi,select-sine-output");
	st->data.sync_clk_enable = !device_property_read_bool(dev, "adi,sync-clk-disable");
	st->data.pdclk_enable = !device_property_read_bool(dev, "adi,pdclk-disable");
	st->data.pdclk_invert = device_property_read_bool(dev, "adi,pdclk-invert");
	st->data.tx_enable_invert = device_property_read_bool(dev, "adi,tx-enable-invert");

	/* DAC full-scale current */
	st->data.dac_output_current = AD9910_DAC_IOUT_DEFAULT_uA;
	ret = device_property_read_u32(dev, "adi,dac-output-current-microamp", &tmp);
	if (!ret) {
		if (tmp >= AD9910_DAC_IOUT_MIN_uA && tmp <= AD9910_DAC_IOUT_MAX_uA)
			st->data.dac_output_current = tmp;
		else
			return dev_err_probe(dev, -ERANGE, "Invalid DAC output current %u\n", tmp);
	}

	return 0;
}

static int ad9910_setup(struct ad9910_state *st)
{
	u32 reg32;
	int ret;

	/* out of reset */
	if (st->gpio_m_reset) {
		udelay(5);
		gpiod_set_value_cansleep(st->gpio_m_reset, 0);
	}

	if (st->gpio_io_reset)
		gpiod_set_value_cansleep(st->gpio_io_reset, 0);

	/* configure CFR1 */
	reg32 = AD9910_CFR1_SDIO_INPUT_ONLY_MSK;
	reg32 |= AD9910_CFR1_MANUAL_OSK_EXT_CTL_MSK |
		 FIELD_PREP(AD9910_CFR1_INV_SINC_EN_MSK, st->data.inverse_sinc_enable) |
		 FIELD_PREP(AD9910_CFR1_SELECT_SINE_MSK, st->data.select_sine_output);

	ret = ad9910_spi_write32(st, AD9910_REG_CFR1, reg32, false);
	if (ret < 0)
		return ret;

	st->reg_cfr1 = reg32;

	/* configure CFR2 */
	reg32 = AD9910_CFR2_AMP_SCALE_SINGLE_TONE_MSK;
	reg32 |= AD9910_CFR2_SYNC_TIMING_VAL_DISABLE_MSK |
		 AD9910_CFR2_DRG_NO_DWELL_HIGH_MSK |
		 AD9910_CFR2_DRG_NO_DWELL_LOW_MSK |
		 FIELD_PREP(AD9910_CFR2_SYNC_CLK_EN_MSK, st->data.sync_clk_enable) |
		 FIELD_PREP(AD9910_CFR2_PDCLK_ENABLE_MSK, st->data.pdclk_enable) |
		 FIELD_PREP(AD9910_CFR2_PDCLK_INVERT_MSK, st->data.pdclk_invert) |
		 FIELD_PREP(AD9910_CFR2_TXENABLE_INVERT_MSK, st->data.tx_enable_invert);

	if (!st->gpio_io_update) {
		reg32 |= AD9910_CFR2_INTERNAL_IO_UPDATE_MSK;
	}

	ret = ad9910_spi_write32(st, AD9910_REG_CFR2, reg32, false);
	if (ret < 0)
		return ret;

	st->reg_cfr2 = reg32;

	/* configure sysclk (CFR3) */
	ret = ad9910_cfg_sysclk(st, false);
	if (ret < 0)
		return ret;

	ret = ad9910_set_dac_current(st, false);
	if (ret < 0)
		return ret;

	/* configure step rate with default values */
	reg32 = FIELD_PREP(AD9910_ASF_AMP_RAMP_RATE_MSK, 1);
	ret = ad9910_spi_write32(st, AD9910_REG_ASF, reg32, false);
	if (ret < 0)
		return ret;

	st->reg_asf = reg32;

	reg32 = FIELD_PREP(AD9910_DRG_RATE_DEC_MSK, 1) |
		FIELD_PREP(AD9910_DRG_RATE_INC_MSK, 1);
	ret = ad9910_spi_write32(st, AD9910_REG_DRG_RATE, reg32, false);
	if (ret < 0)
		return ret;

	st->reg_drg_rate = reg32;

	return ad9910_io_update(st);
}

static void ad9910_power_down(void *data)
{
	struct ad9910_state *st = data;
	u32 cfr1;
	int ret;

	if (st->gpio_powerdown) {
		gpiod_set_value_cansleep(st->gpio_powerdown, 1);
	} else {
		cfr1 = st->reg_cfr1 | AD9910_CFR1_SOFT_POWER_DOWN_MSK;
		ret = ad9910_spi_write32(st, AD9910_REG_CFR1, cfr1, true);
		if (ret) {
			dev_err(&st->spi->dev, "failed to power down device: %d\n", ret);
			return;
		}

		st->reg_cfr1 = cfr1;
	}
}

static int ad9910_probe(struct spi_device *spi)
{
	struct iio_dev *indio_dev;
	struct ad9910_state *st;
	int ret;

	indio_dev = devm_iio_device_alloc(&spi->dev, sizeof(*st));
	if (!indio_dev)
		return -ENOMEM;

	st = iio_priv(indio_dev);
	st->spi = spi;

	spi_set_drvdata(spi, indio_dev);

	st->refclk = devm_clk_get_enabled(&spi->dev, NULL);
	if (IS_ERR(st->refclk))
		return -EPROBE_DEFER;

	ret = devm_regulator_bulk_get_enable(&spi->dev,
					     ARRAY_SIZE(ad9910_power_supplies),
					     ad9910_power_supplies);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "Failed to get regulators\n");

	st->gpio_powerdown = devm_gpiod_get_optional(&spi->dev, "powerdown", GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_powerdown))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_powerdown),
				     "failed to get powerdown gpio\n");

	st->gpio_m_reset = devm_gpiod_get_index_optional(&spi->dev, "reset", 0, GPIOD_OUT_HIGH);
	if (IS_ERR(st->gpio_m_reset))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_m_reset),
				     "failed to get master reset gpio\n");

	st->gpio_io_reset = devm_gpiod_get_index_optional(&spi->dev, "reset", 1, GPIOD_OUT_HIGH);
	if (IS_ERR(st->gpio_io_reset))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_io_reset),
				     "failed to get io reset gpio\n");

	st->gpio_io_update = devm_gpiod_get_optional(&spi->dev, "update", GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_io_update))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_io_update),
				     "failed to get io update gpio\n");

	/* profile pins */
	st->gpio_profile[0] = devm_gpiod_get_index_optional(&spi->dev, "profile", 0, GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_profile[0]))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_profile[0]),
				     "failed to get profile0 gpio\n");

	st->gpio_profile[1] = devm_gpiod_get_index_optional(&spi->dev, "profile", 1, GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_profile[1]))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_profile[1]),
				     "failed to get profile1 gpio\n");

	st->gpio_profile[2] = devm_gpiod_get_index_optional(&spi->dev, "profile", 2, GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_profile[2]))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_profile[2]),
				     "failed to get profile2 gpio\n");

	st->gpio_drctl = devm_gpiod_get_index_optional(&spi->dev, "drg", 0, GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_drctl))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_drctl),
				     "failed to get drctl gpio\n");

	st->gpio_drover = devm_gpiod_get_index_optional(&spi->dev, "drg", 1, GPIOD_IN);
	if (IS_ERR(st->gpio_drover))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_drover),
				     "failed to get drover gpio\n");

	st->gpio_drhold = devm_gpiod_get_index_optional(&spi->dev, "drg", 2, GPIOD_OUT_LOW);
	if (IS_ERR(st->gpio_drhold))
		return dev_err_probe(&spi->dev, PTR_ERR(st->gpio_drhold),
				     "failed to get drhold gpio\n");

	if (st->gpio_drover) {
		st->irq_drover = gpiod_to_irq(st->gpio_drover);
		if (st->irq_drover >= 0) {
			ret = devm_request_irq(&spi->dev, st->irq_drover, ad9910_interrupt,
					       IRQF_TRIGGER_RISING | IRQF_ONESHOT | IRQF_NO_AUTOEN,
					       "ad9910_event", indio_dev);
			if (ret)
				return dev_err_probe(&spi->dev, ret,
						     "failed to request drover irq: %d\n",
						     st->irq_drover);
		}
	}

	ret = ad9910_parse_fw(st);
	if (ret)
		return ret;

	ret = devm_mutex_init(&spi->dev, &st->lock);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "failed to initialize mutex\n");

	indio_dev->name = spi_get_device_id(spi)->name;
	indio_dev->info = &ad9910_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->channels = ad9910_channels;
	indio_dev->num_channels = ARRAY_SIZE(ad9910_channels);

	ret = ad9910_setup(st);
	if (ret < 0)
		return dev_err_probe(&spi->dev, ret, "device setup failed\n");

	ret = devm_add_action_or_reset(&spi->dev, ad9910_power_down, st);
	if (ret)
		return dev_err_probe(&spi->dev, ret, "failed to add power down action\n");

	return devm_iio_device_register(&spi->dev, indio_dev);
}

static const struct spi_device_id ad9910_id[] = {
	{"ad9910", 0},
	{}
};
MODULE_DEVICE_TABLE(spi, ad9910_id);

static const struct of_device_id ad9910_of_match[] = {
	{ .compatible = "adi,ad9910" },
	{ }
};
MODULE_DEVICE_TABLE(of, ad9910_of_match);

static struct spi_driver ad9910_driver = {
	.driver = {
		.name = "ad9910",
		.of_match_table = ad9910_of_match,
	},
	.probe = ad9910_probe,
	.id_table = ad9910_id,
};
module_spi_driver(ad9910_driver);

MODULE_AUTHOR("Rodrigo Alencar <rodrigo.alencar@analog.com>");
MODULE_DESCRIPTION("Analog Devices AD9910 DDS driver");
MODULE_LICENSE("GPL");
