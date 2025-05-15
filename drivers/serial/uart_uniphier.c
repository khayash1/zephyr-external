// SPDX-License-Identifier: Apache-2.0
/*
 * uart_uniphier.c - UniPhier serial driver
 *
 * Copyright (c) 2010, 2012-2015 Wind River Systems, Inc.
 * Copyright (c) 2020-2023 Intel Corp.
 * Copyright (c) 2025 Socionext Inc.
 */

#define DT_DRV_COMPAT socionext_uniphier_uart

/**
 * @brief UniPhier serial rriver
 *
 * This is the driver for the Intel UniPhier UART.
 * Based on ns16550 driver.
 *
 * Before individual UART port can be used, uart_uniphier_port_init() has to be
 * called to setup the port.
 */

#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/types.h>

#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/sys/sys_io.h>
#include <zephyr/spinlock.h>
#include <zephyr/irq.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(uart_uniphier, CONFIG_UART_LOG_LEVEL);

/* register definitions */
#define REG_UNP_TDR    0x00  /* Transmitter holding reg. */
#define REG_UNP_RDR    0x00  /* Receiver data reg. */
#define REG_UNP_IER    0x04  /* Interrupt enable reg. */
#define REG_UNP_IIR    0x08  /* Interrupt ID reg. */
#define REG_UNP_FCR    0x0C  /* FIFO control reg. */
#define REG_UNP_LCRMCR 0x10  /* Line/Modem control reg. */
#define REG_UNP_LSR    0x14  /* Line status reg.*/
#define REG_UNP_MSR    0x18  /* Modem status reg. */
#define REG_UNP_DLR    0x24  /* Baud rate divisor */

/* interrupt enable register */
#define IER_RXRDY BIT(0)  /* receiver data ready */
#define IER_TBE   BIT(1)  /* transmit bit enable */
#define IER_LSR   BIT(2)  /* line status interrupts */
#define IER_MSI   BIT(3)  /* modem status interrupts */

/* interrupt identification register */
#define IIR_NIP     BIT(0)                   /* no interrupt pending */
#define IIR_THRE    BIT(1)                   /* transmit holding register empty interrupt */
#define IIR_RBRF    BIT(2)                   /* receiver buffer register full interrupt */
#define IIR_LS      (BIT(2) | BIT(1))        /* receiver line status interrupt */
#define IIR_FE      (BIT(7) | BIT(6))        /* FIFO mode enabled */
#define IIR_ID_MASK (IIR_THRE | IIR_RBRF)    /* interrupt ID mask without NIP */
#define IIR_MASK    (IIR_ID_MASK | IIR_NIP)  /* interrupt id bits mask  */

/* FIFO control register */
#define FCR_FIFO     BIT(0)             /* enable XMIT and RCVR FIFO */
#define FCR_RCVRCLR  BIT(1)             /* clear RCVR FIFO */
#define FCR_XMITCLR  BIT(2)             /* clear XMIT FIFO */
#define FCR_TFIFO_1  0x0                /* 1 byte in XMIT FIFO */
#define FCR_TFIFO_16 BIT(4)             /* 16 bytes in XMIT FIFO */
#define FCR_TFIFO_32 BIT(5)             /* 32 bytes in XMIT FIFO */
#define FCR_TFIFO_56 (BIT(5) | BIT(4))  /* 56 bytes in XMIT FIFO */
#define FCR_RFIFO_1  0x0                /* 1 byte in RCVR FIFO */
#define FCR_RFIFO_16 BIT(6)             /* 16 bytes in RCVR FIFO */
#define FCR_RFIFO_32 BIT(7)             /* 32 bytes in RCVR FIFO */
#define FCR_RFIFO_56 (BIT(7) | BIT(6))  /* 56 bytes in RCVR FIFO */

/* line control register */
#define LCR_CS5   0x0                /* 5 bits data size */
#define LCR_CS6   BIT(0)             /* 6 bits data size */
#define LCR_CS7   BIT(1)             /* 7 bits data size */
#define LCR_CS8   (BIT(1) | BIT(0))  /* 8 bits data size */
#define LCR_2_STB BIT(2)             /* 2 stop bits */
#define LCR_1_STB 0x0                /* 1 stop bit */
#define LCR_PEN   BIT(3)             /* parity enable */
#define LCR_PDIS  0x0                /* parity disable */
#define LCR_EPS   BIT(4)             /* even parity select */
#define LCR_SP    BIT(5)             /* stick parity select */
#define LCR_SBRK  BIT(6)             /* break control bit */

/* modem control register */
#define MCR_DTR   BIT(0)  /* dtr output */
#define MCR_RTS   BIT(1)  /* rts output */
#define MCR_LOOP  BIT(4)  /* loop back */
#define MCR_AFCE  BIT(5)  /* auto flow control enable */

/* line status register */
#define LSR_RXRDY BIT(0)  /* receiver data available */
#define LSR_OE    BIT(1)  /* overrun error */
#define LSR_PE    BIT(2)  /* parity error */
#define LSR_FE    BIT(3)  /* framing error */
#define LSR_BI    BIT(4)  /* break interrupt */
#define LSR_THRE  BIT(5)  /* transmit holding register empty */
#define LSR_TEMT  BIT(6)  /* transmitter empty */
#define LSR_EOB_MASK (LSR_BI | LSR_FE | LSR_PE | LSR_OE)  /* Error or Break mask */

/* constants for modem status register */

#define MSR_DCTS  BIT(0)  /* cts change */
#define MSR_DDSR  BIT(1)  /* dsr change */
#define MSR_DRI   BIT(2)  /* ring change */
#define MSR_DDCD  BIT(3)  /* data carrier change */
#define MSR_CTS   BIT(4)  /* complement of cts */
#define MSR_DSR   BIT(5)  /* complement of dsr */
#define MSR_RI    BIT(6)  /* complement of ring signal */
#define MSR_DCD   BIT(7)  /* complement of dcd */

#define TDR(dev)    (DEVICE_MMIO_GET(dev) + REG_UNP_TDR)
#define RDR(dev)    (DEVICE_MMIO_GET(dev) + REG_UNP_RDR)
#define IER(dev)    (DEVICE_MMIO_GET(dev) + REG_UNP_IER)
#define IIR(dev)    (DEVICE_MMIO_GET(dev) + REG_UNP_IIR)
#define FCR(dev)    (DEVICE_MMIO_GET(dev) + REG_UNP_FCR)
#define LCRMCR(dev) (DEVICE_MMIO_GET(dev) + REG_UNP_LCRMCR)
#define LSR(dev)    (DEVICE_MMIO_GET(dev) + REG_UNP_LSR)
#define MSR(dev)    (DEVICE_MMIO_GET(dev) + REG_UNP_MSR)
#define DLR(dev)    (DEVICE_MMIO_GET(dev) + REG_UNP_DLR)

#define IIRC(dev) (((struct uart_uniphier_dev_data *)(dev)->data)->iir_cache)

/** Device config structure */
struct uart_uniphier_dev_config {
	DEVICE_MMIO_ROM;
	uint32_t sys_clk_freq;
	const struct device *clock_dev;
	clock_control_subsys_t clock_subsys;
#if defined(CONFIG_UART_INTERRUPT_DRIVEN)
	uart_irq_config_func_t	irq_config_func;
#endif
};

/** Device data structure */
struct uart_uniphier_dev_data {
	DEVICE_MMIO_RAM;
	struct uart_config uart_config;
	struct k_spinlock lock;
	uint8_t fifo_size;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	uint8_t iir_cache;	/**< cache of IIR since it clears when read */
	uart_irq_callback_user_data_t cb;  /**< Callback function pointer */
	void *cb_data;	/**< Callback function arg */
#endif
};

static uint32_t get_uart_baudrate_divisor(const struct device *dev,
					  uint32_t baud_rate,
					  uint32_t pclk)
{
	ARG_UNUSED(dev);
	/*
	 * calculate baud rate divisor. a variant of
	 * (uint32_t)(pclk / (16.0 * baud_rate) + 0.5)
	 */
	return ((pclk + (baud_rate << 3)) / baud_rate) >> 4;
}

static inline int uniphier_read_char(const struct device *dev, unsigned char *c)
{
	if ((sys_read32(LSR(dev)) & LSR_RXRDY) != 0) {
		*c = sys_read32(RDR(dev));
		return 0;
	}

	return -1;
}

static void set_baud_rate(const struct device *dev, uint32_t baud_rate, uint32_t pclk)
{
	struct uart_uniphier_dev_data * const dev_data = dev->data;
	uint32_t divisor; /* baud rate divisor */

	if ((baud_rate != 0U) && (pclk != 0U)) {
		divisor = get_uart_baudrate_divisor(dev, baud_rate, pclk);

		sys_write32(divisor & 0xffff, DLR(dev));

		dev_data->uart_config.baudrate = baud_rate;
	}
}

static int uart_uniphier_configure(const struct device *dev,
				  const struct uart_config *cfg)
{
	struct uart_uniphier_dev_data * const dev_data = dev->data;
	const struct uart_uniphier_dev_config * const dev_cfg = dev->config;
	k_spinlock_key_t key;
	uint8_t c;
	uint32_t pclk = 0U;
	uint32_t val;
	int ret;

	key = k_spin_lock(&dev_data->lock);

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	IIRC(dev) = 0U;
#endif

	/*
	 * set clock frequency from clock_frequency property if valid,
	 * otherwise, get clock frequency from clock manager
	 */
	if (dev_cfg->sys_clk_freq != 0U) {
		pclk = dev_cfg->sys_clk_freq;
	} else {
		if (!device_is_ready(dev_cfg->clock_dev)) {
			ret = -EINVAL;
			goto out;
		}

		ret = clock_control_on(dev_cfg->clock_dev, dev_cfg->clock_subsys);
		if (ret != 0 && ret != -EALREADY && ret != -ENOSYS) {
			goto out;
		}

		if (clock_control_get_rate(dev_cfg->clock_dev,
					   dev_cfg->clock_subsys,
					   &pclk) != 0) {
			ret = -EINVAL;
			goto out;
		}
	}

	set_baud_rate(dev, cfg->baudrate, pclk);

	/* Local structure to hold temporary values to pass to sys_write32() */
	struct uart_config uart_cfg;

	switch (cfg->data_bits) {
	case UART_CFG_DATA_BITS_5:
		uart_cfg.data_bits = LCR_CS5;
		break;
	case UART_CFG_DATA_BITS_6:
		uart_cfg.data_bits = LCR_CS6;
		break;
	case UART_CFG_DATA_BITS_7:
		uart_cfg.data_bits = LCR_CS7;
		break;
	case UART_CFG_DATA_BITS_8:
		uart_cfg.data_bits = LCR_CS8;
		break;
	default:
		ret = -ENOTSUP;
		goto out;
	}

	switch (cfg->stop_bits) {
	case UART_CFG_STOP_BITS_1:
		uart_cfg.stop_bits = LCR_1_STB;
		break;
	case UART_CFG_STOP_BITS_2:
		uart_cfg.stop_bits = LCR_2_STB;
		break;
	default:
		ret = -ENOTSUP;
		goto out;
	}

	switch (cfg->parity) {
	case UART_CFG_PARITY_NONE:
		uart_cfg.parity = LCR_PDIS;
		break;
	case UART_CFG_PARITY_EVEN:
		uart_cfg.parity = LCR_PEN | LCR_EPS;
		break;
	case UART_CFG_PARITY_ODD:
		uart_cfg.parity = LCR_PEN;
		break;
	default:
		ret = -ENOTSUP;
		goto out;
	}

	dev_data->uart_config = *cfg;

	/* data bits, stop bits, parity */
	val = (((uart_cfg.data_bits | uart_cfg.stop_bits | uart_cfg.parity) << 8)
	      | MCR_RTS | MCR_DTR);
	sys_write32(val, LCRMCR(dev));

	/*
	 * Program FIFO: enabled, mode 0 (set for compatibility with quark),
	 * generate the interrupt at 8th byte
	 * Clear TX and RX FIFO
	 */
	val = FCR_FIFO | FCR_RFIFO_1 | FCR_TFIFO_1 | FCR_RCVRCLR | FCR_XMITCLR;
	sys_write32(val, FCR(dev));

	if ((sys_read32(IIR(dev)) & IIR_FE) == IIR_FE) {
		dev_data->fifo_size = 16;
	} else {
		dev_data->fifo_size = 1;
	}

	/* clear the port */
	(void)uniphier_read_char(dev, &c);

	/* disable interrupts  */
	sys_write32(0, IER(dev));
	ret = 0;

out:
	k_spin_unlock(&dev_data->lock, key);
	return ret;
};

#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE

static int uart_uniphier_config_get(const struct device *dev,
				   struct uart_config *cfg)
{
	struct uart_uniphier_dev_data *data = dev->data;

	cfg->baudrate = data->uart_config.baudrate;
	cfg->parity = data->uart_config.parity;
	cfg->stop_bits = data->uart_config.stop_bits;
	cfg->data_bits = data->uart_config.data_bits;
	cfg->flow_ctrl = data->uart_config.flow_ctrl;

	return 0;
}

#endif /* CONFIG_UART_USE_RUNTIME_CONFIGURE */

/**
 * @brief Initialize individual UART port
 *
 * This routine is called to reset the chip in a quiescent state.
 *
 * @param dev UART device struct
 *
 * @return 0 if successful, failed otherwise
 */
static int uart_uniphier_init(const struct device *dev)
{
	struct uart_uniphier_dev_data *data = dev->data;
	const struct uart_uniphier_dev_config *dev_cfg = dev->config;
	int ret;

	ARG_UNUSED(dev_cfg);

	DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

	ret = uart_uniphier_configure(dev, &data->uart_config);
	if (ret != 0) {
		return ret;
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	dev_cfg->irq_config_func(dev);
#endif

	return 0;
}

/**
 * @brief Poll the device for input.
 *
 * @param dev UART device struct
 * @param c Pointer to character
 *
 * @return 0 if a character arrived, -1 if the input buffer if empty.
 */
static int uart_uniphier_poll_in(const struct device *dev, unsigned char *c)
{
	struct uart_uniphier_dev_data *data = dev->data;
	k_spinlock_key_t key;
	int ret = -1;

	key = k_spin_lock(&data->lock);
	ret = uniphier_read_char(dev, c);
	k_spin_unlock(&data->lock, key);

	return ret;
}

/**
 * @brief Output a character in polled mode.
 *
 * Checks if the transmitter is empty. If empty, a character is written to
 * the data register.
 *
 * If the hardware flow control is enabled then the handshake signal CTS has to
 * be asserted in order to send a character.
 *
 * @param dev UART device struct
 * @param c Character to send
 */
static void uart_uniphier_poll_out(const struct device *dev,
					   unsigned char c)
{
	struct uart_uniphier_dev_data *data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&data->lock);

	while ((sys_read32(LSR(dev)) & LSR_THRE) == 0) {
	}
	sys_write32(c, TDR(dev));

	k_spin_unlock(&data->lock, key);
}

/**
 * @brief Check if an error was received
 *
 * @param dev UART device struct
 *
 * @return one of UART_ERROR_OVERRUN, UART_ERROR_PARITY, UART_ERROR_FRAMING,
 * UART_BREAK if an error was detected, 0 otherwise.
 */
static int uart_uniphier_err_check(const struct device *dev)
{
	struct uart_uniphier_dev_data *data = dev->data;
	k_spinlock_key_t key;
	int check;

	key = k_spin_lock(&data->lock);
	check = (sys_read32(LSR(dev)) & LSR_EOB_MASK);
	k_spin_unlock(&data->lock, key);

	return check >> 1;
}

#if CONFIG_UART_INTERRUPT_DRIVEN

/**
 * @brief Fill FIFO with data
 *
 * @param dev UART device struct
 * @param tx_data Data to transmit
 * @param size Number of bytes to send
 *
 * @return Number of bytes sent
 */
static int uart_uniphier_fifo_fill(const struct device *dev,
				  const uint8_t *tx_data,
				  int size)
{
	struct uart_uniphier_dev_data *data = dev->data;
	k_spinlock_key_t key;
	int i;

	key = k_spin_lock(&data->lock);

	for (i = 0; (i < size) && (i < data->fifo_size); i++) {
		sys_write32(tx_data[i], TDR(dev));
	}

	k_spin_unlock(&data->lock, key);

	return i;
}

/**
 * @brief Read data from FIFO
 *
 * @param dev UART device struct
 * @param rxData Data container
 * @param size Container size
 *
 * @return Number of bytes read
 */
static int uart_uniphier_fifo_read(const struct device *dev, uint8_t *rx_data,
				  const int size)
{
	struct uart_uniphier_dev_data *data = dev->data;
	k_spinlock_key_t key;
	int i;

	key = k_spin_lock(&data->lock);

	for (i = 0; (i < size) && (uniphier_read_char(dev, &rx_data[i]) != -1); i++) {
	}

	k_spin_unlock(&data->lock, key);

	return i;
}

/**
 * @brief Enable TX interrupt in IER
 *
 * @param dev UART device struct
 */
static void uart_uniphier_irq_tx_enable(const struct device *dev)
{
	struct uart_uniphier_dev_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t val;

	key = k_spin_lock(&data->lock);

	val = sys_read32(IER(dev));
	val |= IER_TBE;
	sys_write32(val, IER(dev));

	k_spin_unlock(&data->lock, key);
}

/**
 * @brief Disable TX interrupt in IER
 *
 * @param dev UART device struct
 */
static void uart_uniphier_irq_tx_disable(const struct device *dev)
{
	struct uart_uniphier_dev_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t val;

	key = k_spin_lock(&data->lock);

	val = sys_read32(IER(dev));
	val &= ~IER_TBE;
	sys_write32(val, IER(dev));

	k_spin_unlock(&data->lock, key);
}

/**
 * @brief Check if Tx IRQ has been raised
 *
 * @param dev UART device struct
 *
 * @return 1 if an IRQ is ready, 0 otherwise
 */
static int uart_uniphier_irq_tx_ready(const struct device *dev)
{
	struct uart_uniphier_dev_data *data = dev->data;
	k_spinlock_key_t key;
	int ret = -1;

	key = k_spin_lock(&data->lock);
	ret = ((IIRC(dev) & IIR_ID_MASK) == IIR_THRE) ? 1 : 0;
	k_spin_unlock(&data->lock, key);

	return ret;
}

/**
 * @brief Check if nothing remains to be transmitted
 *
 * @param dev UART device struct
 *
 * @return 1 if nothing remains to be transmitted, 0 otherwise
 */
static int uart_uniphier_irq_tx_complete(const struct device *dev)
{
	struct uart_uniphier_dev_data *data = dev->data;
	k_spinlock_key_t key;
	int ret = -1;

	key = k_spin_lock(&data->lock);
	ret = ((sys_read32(LSR(dev)) & (LSR_TEMT | LSR_THRE))
				== (LSR_TEMT | LSR_THRE)) ? 1 : 0;
	k_spin_unlock(&data->lock, key);

	return ret;
}

/**
 * @brief Enable RX interrupt in IER
 *
 * @param dev UART device struct
 */
static void uart_uniphier_irq_rx_enable(const struct device *dev)
{
	struct uart_uniphier_dev_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t val;

	key = k_spin_lock(&data->lock);

	val = sys_read32(IER(dev));
	val |= IER_RXRDY;
	sys_write32(val, IER(dev));

	k_spin_unlock(&data->lock, key);
}

/**
 * @brief Disable RX interrupt in IER
 *
 * @param dev UART device struct
 */
static void uart_uniphier_irq_rx_disable(const struct device *dev)
{
	struct uart_uniphier_dev_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t val;

	key = k_spin_lock(&data->lock);

	val = sys_read32(IER(dev));
	val &= ~IER_RXRDY;
	sys_write32(val, IER(dev));

	k_spin_unlock(&data->lock, key);
}

/**
 * @brief Check if Rx IRQ has been raised
 *
 * @param dev UART device struct
 *
 * @return 1 if an IRQ is ready, 0 otherwise
 */
static int uart_uniphier_irq_rx_ready(const struct device *dev)
{
	struct uart_uniphier_dev_data *data = dev->data;
	k_spinlock_key_t key;
	int ret = -1;

	key = k_spin_lock(&data->lock);
	ret = ((IIRC(dev) & IIR_ID_MASK) == IIR_RBRF) ? 1 : 0;
	k_spin_unlock(&data->lock, key);

	return ret;
}

/**
 * @brief Enable error interrupt in IER
 *
 * @param dev UART device struct
 */
static void uart_uniphier_irq_err_enable(const struct device *dev)
{
	struct uart_uniphier_dev_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t val;

	key = k_spin_lock(&data->lock);

	val = sys_read32(IER(dev));
	val |= IER_LSR;
	sys_write32(val, IER(dev));

	k_spin_unlock(&data->lock, key);
}

/**
 * @brief Disable error interrupt in IER
 *
 * @param dev UART device struct
 *
 * @return 1 if an IRQ is ready, 0 otherwise
 */
static void uart_uniphier_irq_err_disable(const struct device *dev)
{
	struct uart_uniphier_dev_data *data = dev->data;
	k_spinlock_key_t key;
	uint32_t val;

	key = k_spin_lock(&data->lock);

	val = sys_read32(IER(dev));
	val &= ~IER_LSR;
	sys_write32(val, IER(dev));

	k_spin_unlock(&data->lock, key);
}

/**
 * @brief Check if any IRQ is pending
 *
 * @param dev UART device struct
 *
 * @return 1 if an IRQ is pending, 0 otherwise
 */
static int uart_uniphier_irq_is_pending(const struct device *dev)
{
	struct uart_uniphier_dev_data *data = dev->data;
	k_spinlock_key_t key;
	int ret;

	key = k_spin_lock(&data->lock);
	ret = (!(IIRC(dev) & IIR_NIP)) ? 1 : 0;
	k_spin_unlock(&data->lock, key);

	return ret;
}

/**
 * @brief Update cached contents of IIR
 *
 * @param dev UART device struct
 *
 * @return Always 1
 */
static int uart_uniphier_irq_update(const struct device *dev)
{
	struct uart_uniphier_dev_data *data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&data->lock);
	IIRC(dev) = sys_read32(IIR(dev));
	k_spin_unlock(&data->lock, key);

	return 1;
}

/**
 * @brief Set the callback function pointer for IRQ.
 *
 * @param dev UART device struct
 * @param cb Callback function pointer.
 */
static void uart_uniphier_irq_callback_set(const struct device *dev,
					  uart_irq_callback_user_data_t cb,
					  void *cb_data)
{
	struct uart_uniphier_dev_data * const dev_data = dev->data;
	k_spinlock_key_t key;

	key = k_spin_lock(&dev_data->lock);
	dev_data->cb = cb;
	dev_data->cb_data = cb_data;
	k_spin_unlock(&dev_data->lock, key);
}

/**
 * @brief Interrupt service routine.
 *
 * This simply calls the callback function, if one exists.
 *
 * @param arg Argument to ISR.
 */
static void uart_uniphier_isr(const struct device *dev)
{
	struct uart_uniphier_dev_data * const dev_data = dev->data;

	if (dev_data->cb) {
		dev_data->cb(dev, dev_data->cb_data);
	}
}

#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#ifdef CONFIG_UART_UNIPHIER_LINE_CTRL

/**
 * @brief Manipulate line control for UART.
 *
 * @param dev UART device struct
 * @param ctrl The line control to be manipulated
 * @param val Value to set the line control
 *
 * @return 0 if successful, failed otherwise
 */
static int uart_uniphier_line_ctrl_set(const struct device *dev,
				      uint32_t ctrl, uint32_t val)
{
	struct uart_uniphier_dev_data *data = dev->data;
	const struct uart_uniphier_dev_config *const dev_cfg = dev->config;
	uint32_t mcr, chg, pclk = 0U;
	k_spinlock_key_t key;

	if (dev_cfg->sys_clk_freq != 0U) {
		pclk = dev_cfg->sys_clk_freq;
	} else {
		if (device_is_ready(dev_cfg->clock_dev)) {
			clock_control_get_rate(dev_cfg->clock_dev, dev_cfg->clock_subsys, &pclk);
		}
	}

	switch (ctrl) {
	case UART_LINE_CTRL_BAUD_RATE:
		set_baud_rate(dev, val, pclk);
		return 0;

	case UART_LINE_CTRL_RTS:
	case UART_LINE_CTRL_DTR:
		key = k_spin_lock(&data->lock);
		mcr = sys_read32(LCRMCR(dev));

		if (ctrl == UART_LINE_CTRL_RTS) {
			chg = MCR_RTS;
		} else {
			chg = MCR_DTR;
		}

		if (val) {
			mcr |= chg;
		} else {
			mcr &= ~(chg);
		}
		sys_write32(mcr, LCRMCR(dev));
		k_spin_unlock(&data->lock, key);

		return 0;
	}

	return -ENOTSUP;
}

#endif /* CONFIG_UART_UNIPHIER_LINE_CTRL */

static DEVICE_API(uart, uart_uniphier_driver_api) = {
	.poll_in = uart_uniphier_poll_in,
	.poll_out = uart_uniphier_poll_out,
	.err_check = uart_uniphier_err_check,
#ifdef CONFIG_UART_USE_RUNTIME_CONFIGURE
	.configure = uart_uniphier_configure,
	.config_get = uart_uniphier_config_get,
#endif
#ifdef CONFIG_UART_INTERRUPT_DRIVEN
	.fifo_fill = uart_uniphier_fifo_fill,
	.fifo_read = uart_uniphier_fifo_read,
	.irq_tx_enable = uart_uniphier_irq_tx_enable,
	.irq_tx_disable = uart_uniphier_irq_tx_disable,
	.irq_tx_ready = uart_uniphier_irq_tx_ready,
	.irq_tx_complete = uart_uniphier_irq_tx_complete,
	.irq_rx_enable = uart_uniphier_irq_rx_enable,
	.irq_rx_disable = uart_uniphier_irq_rx_disable,
	.irq_rx_ready = uart_uniphier_irq_rx_ready,
	.irq_err_enable = uart_uniphier_irq_err_enable,
	.irq_err_disable = uart_uniphier_irq_err_disable,
	.irq_is_pending = uart_uniphier_irq_is_pending,
	.irq_update = uart_uniphier_irq_update,
	.irq_callback_set = uart_uniphier_irq_callback_set,
#endif
#ifdef CONFIG_UART_UNIPHIER_LINE_CTRL
	.line_ctrl_set = uart_uniphier_line_ctrl_set,
#endif
};

#define UART_UNIPHIER_IRQ_FLAGS(n) \
	COND_CODE_1(DT_INST_IRQ_HAS_CELL(n, sense),                            \
		    (DT_INST_IRQ(n, sense)),                                   \
		    (0))

/* IO-port or MMIO based UART */
#define UART_UNIPHIER_IRQ_CONFIG(n)                                            \
	static void uart_uniphier_irq_config_func##n(const struct device *dev) \
	{                                                                      \
		ARG_UNUSED(dev);                                               \
		IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority),	       \
			    uart_uniphier_isr, DEVICE_DT_INST_GET(n),	       \
			    UART_UNIPHIER_IRQ_FLAGS(n));                       \
		irq_enable(DT_INST_IRQN(n));                                   \
	}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define DEV_CONFIG_IRQ_FUNC_INIT(n) \
	.irq_config_func = uart_uniphier_irq_config_func##n,
#define UART_UNIPHIER_IRQ_FUNC_DECLARE(n) \
	static void uart_uniphier_irq_config_func##n(const struct device *dev);
#define UART_UNIPHIER_IRQ_FUNC_DEFINE(n) \
	UART_UNIPHIER_IRQ_CONFIG(n)
#else
/* !CONFIG_UART_INTERRUPT_DRIVEN */
#define DEV_CONFIG_IRQ_FUNC_INIT(n)
#define UART_UNIPHIER_IRQ_FUNC_DECLARE(n)
#define UART_UNIPHIER_IRQ_FUNC_DEFINE(n)
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#define UART_UNIPHIER_COMMON_DEV_CFG_INITIALIZER(n)                                  \
		COND_CODE_1(DT_INST_NODE_HAS_PROP(n, clock_frequency), (             \
				.sys_clk_freq = DT_INST_PROP(n, clock_frequency),    \
				.clock_dev = NULL,                                   \
				.clock_subsys = NULL,                                \
			), (                                                         \
				.sys_clk_freq = 0,                                   \
				.clock_dev = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(n)),  \
				.clock_subsys = (clock_control_subsys_t) DT_INST_PHA(\
								0, clocks, clkid),   \
			)                                                            \
		)                                                                    \
		IF_ENABLED(DT_INST_NODE_HAS_PROP(n, resets),                         \
			(.reset_spec = RESET_DT_SPEC_INST_GET(n),))

#define UART_UNIPHIER_COMMON_DEV_DATA_INITIALIZER(n)                                 \
		.uart_config.baudrate = DT_INST_PROP_OR(n, current_speed, 0),        \
		.uart_config.parity = UART_CFG_PARITY_NONE,                          \
		.uart_config.stop_bits = UART_CFG_STOP_BITS_1,                       \
		.uart_config.data_bits = UART_CFG_DATA_BITS_8,                       \
		.uart_config.flow_ctrl =                                             \
			COND_CODE_1(DT_INST_PROP_OR(n, hw_flow_control, 0),          \
				    (UART_CFG_FLOW_CTRL_RTS_CTS),                    \
				    (UART_CFG_FLOW_CTRL_NONE)),                      \

#define UART_UNIPHIER_DEVICE_IO_MMIO_INIT(n)                                         \
	UART_UNIPHIER_IRQ_FUNC_DECLARE(n);                                           \
	static const struct uart_uniphier_dev_config uart_uniphier_dev_cfg_##n = {   \
		DEVICE_MMIO_ROM_INIT(DT_DRV_INST(n)),                                \
		UART_UNIPHIER_COMMON_DEV_CFG_INITIALIZER(n)                          \
		DEV_CONFIG_IRQ_FUNC_INIT(n)                                          \
	};                                                                           \
	static struct uart_uniphier_dev_data uart_uniphier_dev_data_##n = {          \
		UART_UNIPHIER_COMMON_DEV_DATA_INITIALIZER(n)                         \
	};                                                                           \
	DEVICE_DT_INST_DEFINE(n, uart_uniphier_init, NULL,                           \
			      &uart_uniphier_dev_data_##n, &uart_uniphier_dev_cfg_##n, \
			      PRE_KERNEL_1, CONFIG_SERIAL_INIT_PRIORITY,             \
			      &uart_uniphier_driver_api);                            \
	UART_UNIPHIER_IRQ_FUNC_DEFINE(n)

#define UART_UNIPHIER_DEVICE_INIT(n)                                                 \
	UART_UNIPHIER_DEVICE_IO_MMIO_INIT(n)

DT_INST_FOREACH_STATUS_OKAY(UART_UNIPHIER_DEVICE_INIT)
