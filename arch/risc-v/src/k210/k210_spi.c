/****************************************************************************
 * arch/risc-v/src/k210/k210_spi.c
 *
 * Derives from software originally provided by Canaan Inc
 *
 *   Copyright 2018 Canaan Inc
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/spi/spi.h>
#include <arch/board/board.h>

#include "riscv_arch.h"
#include "k210_clockconfig.h"
#include "k210_spi.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct k210_spidev_s
{
  struct spi_dev_s spidev;       /* Externally visible part */
  uint32_t         spibase;      /* SPIn base address */
  uint8_t          bus;          /* Bus number */
  bool             initialized;  /* Has SPI interface been initialized */
  sem_t            exclsem;      /* Held while chip is selected */
  uint32_t         frequency;    /* Requested clock frequency */
  uint32_t         actual;       /* Actual clock frequency */
  uint8_t          nbits;        /* Width of word in bits (8/16/32) */
  uint8_t          mode;         /* Mode (0/1/2/3) */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* Helpers */

static inline uint32_t spi_getreg(FAR struct k210_spidev_s *priv,
                                    uint8_t offset);

static inline void spi_putreg(FAR struct k210_spidev_s *priv,
                                uint8_t offset, uint32_t value);

static void spi_modifyctl0(FAR struct k210_spidev_s *priv,
                           uint32_t setbits, uint32_t clrbits);

static inline uint32_t spi_readword(FAR struct k210_spidev_s *priv);
static inline void spi_writeword(FAR struct k210_spidev_s *priv,
                                 uint32_t byte);

/* SPI methods */

static int         spi_lock(FAR struct spi_dev_s *dev, bool lock);
static uint32_t    spi_setfrequency(FAR struct spi_dev_s *dev,
                                    uint32_t frequency);
static void        spi_setmode(FAR struct spi_dev_s *dev,
                               enum spi_mode_e mode);
static void        spi_setbits(FAR struct spi_dev_s *dev, int nbits);
static uint32_t    spi_send(FAR struct spi_dev_s *dev, uint32_t wd);

#ifdef CONFIG_SPI_EXCHANGE
static void        spi_exchange(FAR struct spi_dev_s *dev,
                                FAR const void *txbuffer,
                                FAR void *rxbuffer, size_t nwords);
#else
static void        spi_sndblock(FAR struct spi_dev_s *dev,
                                FAR const void *txbuffer,
                                size_t nwords);
static void        spi_recvblock(FAR struct spi_dev_s *dev,
                                 FAR void *rxbuffer,
                                 size_t nwords);
#endif

/* Initialization */

static void        spi_bus_initialize(FAR struct k210_spidev_s *priv);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_K210_SPI0

static const struct spi_ops_s g_spi0ops =
{
  .lock              = spi_lock,
  .select            = k210_spi0select,
  .setfrequency      = spi_setfrequency,
  .setmode           = spi_setmode,
  .setbits           = spi_setbits,
  .status            = k210_spi0status,
  .send              = spi_send,
#ifdef CONFIG_SPI_EXCHANGE
  .exchange          = spi_exchange,
#else
  .sndblock          = spi_sndblock,
  .recvblock         = spi_recvblock,
#endif
};

static struct k210_spidev_s g_spi0dev =
{
  .spidev   =
              {
               &g_spi0ops
              },
  .spibase  = K210_SPI0_BASE,
  .bus      = 0,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: spi_getreg
 *
 * Description:
 *   Get the contents of the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *
 * Returned Value:
 *   The contents of the 32-bit register
 *
 ****************************************************************************/

static inline uint32_t spi_getreg(FAR struct k210_spidev_s *priv,
                                  uint8_t offset)
{
  return getreg32((uintptr_t)priv->spibase + offset);
}

/****************************************************************************
 * Name: spi_putreg
 *
 * Description:
 *   Write a 32-bit value to the SPI register at offset
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *   offset - offset to the register of interest
 *   value  - the 32-bit value to be written
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_putreg(FAR struct k210_spidev_s *priv,
                              uint8_t offset,
                              uint32_t value)
{
  putreg32(value, (uintptr_t)priv->spibase + offset);
}

/****************************************************************************
 * Name: spi_modifyctl0
 *
 * Description:
 *   Clear and set bits in the CTL0 register
 *
 * Input Parameters:
 *   priv    - Device-specific state data
 *   clrbits - The bits to clear
 *   setbits - The bits to set
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_modifyctl0(FAR struct k210_spidev_s *priv,
                           uint32_t setbits, uint32_t clrbits)
{
  uint32_t val;
  val = spi_getreg(priv, SPI_CTL0_OFFSET);
  val &= ~clrbits;
  val |= setbits;
  spi_putreg(priv, SPI_CTL0_OFFSET, val);
}

/****************************************************************************
 * Name: spi_lock
 *
 * Description:
 *   On SPI buses where there are multiple devices, it will be necessary to
 *   lock SPI to have exclusive access to the buses for a sequence of
 *   transfers.  The bus should be locked before the chip is selected. After
 *   locking the SPI bus, the caller should then also call the setfrequency,
 *   setbits, and setmode methods to make sure that the SPI is properly
 *   configured for the device.  If the SPI bus is being shared, then it
 *   may have been left in an incompatible state.
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   lock - true: Lock spi bus, false: unlock SPI bus
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static int spi_lock(FAR struct spi_dev_s *dev, bool lock)
{
  FAR struct k210_spidev_s *priv = (FAR struct k210_spidev_s *)dev;
  int ret;

  if (lock)
    {
      ret = nxsem_wait_uninterruptible(&priv->exclsem);
    }
  else
    {
      ret = nxsem_post(&priv->exclsem);
    }

  return ret;
}

/****************************************************************************
 * Name: spi_readword
 *
 * Description:
 *   Read one word from SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *
 * Returned Value:
 *   Byte as read
 *
 ****************************************************************************/

static inline uint32_t spi_readword(FAR struct k210_spidev_s *priv)
{
  uint32_t ret;

  /* Set transfer mode */

  spi_modifyctl0(priv,
                 SPI_CTL0_TMODE_RX << SPI_CTL0_TMODE_SHIFT,
                 SPI_CTL0_TMODE_MASK);

  /* Set ctrl1 */

  spi_putreg(priv, SPI_CTL1_OFFSET, 0); /* 0: 1word */

  /* Write dummy data?? */

  spi_putreg(priv, SPI_DAT_OFFSET, 0xffffffff);

  /* Set chip select */

  spi_putreg(priv, SPI_SEN_OFFSET, 1 << 3); /* TODO */

  /* Enable Master */

  spi_putreg(priv, SPI_MEN_OFFSET, 1);

  /* Wait until the receive fifo is not empty */

  //while (spi_getreg(priv, SPI_RFL_OFFSET) == 0)
    {
    }

  switch (priv->nbits)
    {
      case 8:
        ret = spi_getreg(priv, SPI_DAT_OFFSET);
        ret &= 0xff;
        break;

      case 16:
        ret = spi_getreg(priv, SPI_DAT_OFFSET);
        ret &= 0xffff;
        break;

      case 32:
        ret = spi_getreg(priv, SPI_DAT_OFFSET);
        break;

      default:
        ASSERT(false);
        ret = 0;
        break;
    }

  /* Disable chip select */

  spi_putreg(priv, SPI_SEN_OFFSET, 0);

  /* Disable Master */

  spi_putreg(priv, SPI_MEN_OFFSET, 0);

  return ret;
}

/****************************************************************************
 * Name: spi_writeword
 *
 * Description:
 *   Write one byte to SPI
 *
 * Input Parameters:
 *   priv - Device-specific state data
 *   byte - Byte to send
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static inline void spi_writeword(FAR struct k210_spidev_s *priv,
                                 uint32_t word)
{
  /* Set transfer mode */

  spi_modifyctl0(priv,
                 SPI_CTL0_TMODE_TX << SPI_CTL0_TMODE_SHIFT,
                 SPI_CTL0_TMODE_MASK);

  /* Enable Master */

  spi_putreg(priv, SPI_MEN_OFFSET, 1);

  /* Set chip select */

  spi_putreg(priv, SPI_SEN_OFFSET, 1 << 3); /* TODO */

  /* TODO: Consider word size (1/2/4) ?? */

  /* Wait until the TX FIFO has a room to send */

  while (spi_getreg(priv, SPI_TFL_OFFSET) == 32)
    {
    }

  /* Then send the word (NOTE: target register is 32bit) */

  switch (priv->nbits)
    {
      case 8:
        spi_putreg(priv, SPI_DAT_OFFSET, (0xff & word));
        break;

      case 16:
        spi_putreg(priv, SPI_DAT_OFFSET, (0xffff & word));
        break;

      case 32:
        spi_putreg(priv, SPI_DAT_OFFSET, word);
        break;

      default:
        ASSERT(false);
        break;
    }

  /* Check status register */

  while ((spi_getreg(priv, SPI_STS_OFFSET) & 0x05) != 0x04)
    {
    }

  /* Disable chip select */

  spi_putreg(priv, SPI_SEN_OFFSET, 0);

  /* Disable Master */

  spi_putreg(priv, SPI_MEN_OFFSET, 0);
}

/****************************************************************************
 * Name: spi_send
 *
 * Description:
 *   Exchange one word on SPI
 *
 * Input Parameters:
 *   dev - Device-specific state data
 *   wd  - The word to send.  the size of the data is determined by the
 *         number of bits selected for the SPI interface.
 *
 * Returned Value:
 *   response
 *
 ****************************************************************************/

static uint32_t spi_send(FAR struct spi_dev_s *dev, uint32_t wd)
{
  FAR struct k210_spidev_s *priv = (FAR struct k210_spidev_s *)dev;
  uint32_t ret;

  DEBUGASSERT(priv && priv->spibase);

  spi_writeword(priv, wd);
  ret = spi_readword(priv);

#if 0 /* TODO */
  uint32_t regval;

  /* Check and clear any error flags */

  regval = spi_getreg(priv, K210_SPI_SR_OFFSET);

  spiinfo("Sent: %04x Return: %04x Status: %02x\n", wd, ret, regval);
  UNUSED(regval);
#endif

  return ret;
}

/****************************************************************************
 * Name: spi_exchange (no DMA)
 *
 * Description:
 *   Exchange a block of data on SPI without using DMA
 *
 * Input Parameters:
 *   dev      - Device-specific state data
 *   txbuffer - A pointer to the buffer of data to be sent
 *   rxbuffer - A pointer to a buffer in which to receive data
 *   nwords   - the length of data to be exchaned in units of words.
 *              The wordsize is determined by the number of bits-per-word
 *              selected for the SPI interface.  If nbits <= 8, the data is
 *              packed into uint8_t's; if nbits >8, the data is packed into
 *              uint16_t's
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_exchange(FAR struct spi_dev_s *dev, FAR const void *txbuffer,
                         FAR void *rxbuffer, size_t nwords)
{
  FAR struct k210_spidev_s *priv = (FAR struct k210_spidev_s *)dev;
  DEBUGASSERT(priv && priv->spibase);

  spiinfo("txbuffer=%p rxbuffer=%p nwords=%d\n", txbuffer, rxbuffer, nwords);

  /* 8- or 16-bit mode? */

  if (priv->nbits > 8)
    {
      /* 16-bit mode */

      const uint16_t *src  = (const uint16_t *)txbuffer;
            uint16_t *dest = (uint16_t *)rxbuffer;
            uint16_t  word;

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (src)
            {
              word = *src++;
            }
          else
            {
              word = 0xffff;
            }

          /* Exchange one word */

          word = (uint16_t)spi_send(dev, (uint32_t)word);

          /* Is there a buffer to receive the return value? */

          if (dest)
            {
              *dest++ = word;
            }
        }
    }
  else
    {
      /* 8-bit mode */

      const uint8_t *src  = (const uint8_t *)txbuffer;
            uint8_t *dest = (uint8_t *)rxbuffer;
            uint8_t  word;

      while (nwords-- > 0)
        {
          /* Get the next word to write.  Is there a source buffer? */

          if (src)
            {
              word = *src++;
            }
          else
            {
              word = 0xff;
            }

          /* Exchange one word */

          word = (uint8_t)spi_send(dev, (uint32_t)word);

          /* Is there a buffer to receive the return value? */

          if (dest)
            {
              *dest++ = word;
            }
        }
    }
}

/****************************************************************************
 * Name: spi_setfrequency
 *
 * Description:
 *   Set the SPI frequency.
 *
 * Input Parameters:
 *   dev -       Device-specific state data
 *   frequency - The SPI frequency requested
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static uint32_t spi_setfrequency(FAR struct spi_dev_s *dev, uint32_t freq)
{
  FAR struct k210_spidev_s *priv = (FAR struct k210_spidev_s *)dev;
  uint32_t spiclk;
  uint32_t div;

  if (freq != priv->frequency)
    {
      spiclk = k210_get_spiclk(priv->bus);
      div = spiclk / freq;

      if (div < 2)
        {
          div = 2;
        }
      else if (div > 65534)
        {
          div = 65534;
        }

      spi_putreg(priv, SPI_DIV_OFFSET, div);
      priv->actual = spiclk / div;
    }

  return priv->actual;
}

/****************************************************************************
 * Name: spi_setmode
 *
 * Description:
 *   Set the SPI mode.  see enum spi_mode_e for mode definitions
 *
 * Input Parameters:
 *   dev  - Device-specific state data
 *   mode - The SPI mode requested (0/1/2/3)
 *
 * Returned Value:
 *   Returns the actual frequency selected
 *
 ****************************************************************************/

static void spi_setmode(FAR struct spi_dev_s *dev, enum spi_mode_e mode)
{
  FAR struct k210_spidev_s *priv = (FAR struct k210_spidev_s *)dev;

  spiinfo("mode=%d\n", mode);

  /* Has the mode changed? */

  if (mode != priv->mode)
    {
      /* Set the mode to the controller */

      spi_modifyctl0(priv,
                     mode << SPI_CTL0_WMODE_SHIFT,
                     SPI_CTL0_WMODE_MASK);

      /* Save the mode */

      priv->mode = mode;
    }
}

/****************************************************************************
 * Name: spi_setbits
 *
 * Description:
 *   Set the number of bits per word.
 *
 * Input Parameters:
 *   dev   - Device-specific state data
 *   nbits - The number of bits requested (8/16/32)
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_setbits(FAR struct spi_dev_s *dev, int nbits)
{
  FAR struct k210_spidev_s *priv = (FAR struct k210_spidev_s *)dev;

  spiinfo("nbits=%d\n", nbits);

  /* Has the number of bits changed? */

  if (nbits != priv->nbits)
    {
      /* Set the bits to the controller */

      spi_modifyctl0(priv,
                     (nbits - 1) << SPI_CTL0_DFS_SHIFT,
                     SPI_CTL0_DFS_MASK);

      /* Save the nbits */

      priv->nbits = nbits;
    }
}

/****************************************************************************
 * Name: spi_bus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus in its default state (Master, 8-bit,
 *   mode 0, etc.)
 *
 * Input Parameters:
 *   priv   - private SPI device structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

static void spi_bus_initialize(FAR struct k210_spidev_s *priv)
{
  /* NOTE: we assume that APB2 & SPIn clock is enabled */

  /* Set defautlt freq/nbits/mode */

  spi_setfrequency((FAR struct spi_dev_s *)priv, 400000);

  spi_putreg(priv, SPI_CTL0_OFFSET, 0); /* clear */
  spi_setbits((FAR struct spi_dev_s *)priv, 8);
  spi_setmode((FAR struct spi_dev_s *)priv, SPIDEV_MODE0);

  /* Initialize the SPI semaphore that enforces mutually exclusive access */

  nxsem_init(&priv->exclsem, 0, 1);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: k210_spibus_initialize
 *
 * Description:
 *   Initialize the selected SPI bus
 *
 * Input Parameters:
 *   Port number (for hardware that has multiple SPI interfaces)
 *
 * Returned Value:
 *   Valid SPI device structure reference on success; a NULL on failure
 *
 ****************************************************************************/

FAR struct spi_dev_s *k210_spibus_initialize(int bus)
{
  FAR struct k210_spidev_s *priv = NULL;

  irqstate_t flags = enter_critical_section();

#ifdef CONFIG_K210_SPI0
  if (bus == 0)
    {
      /* Select SPI0 */

      priv = &g_spi0dev;

      /* Only configure if the bus is not already configured */

      if (!priv->initialized)
        {
          /* TODO : Configure SPI0 pins: SCK, MISO, and MOSI */

          /* Set up default configuration: Master, 8-bit, etc. */

          spi_bus_initialize(priv);
          priv->initialized = true;
        }
    }
  else
#endif
    {
      spierr("ERROR: Unsupported SPI bus: %d\n", bus);
    }

  leave_critical_section(flags);
  return (FAR struct spi_dev_s *)priv;
}
