#pragma once
#pragma message "ESP32 Hardware SPI support added"

FASTLED_NAMESPACE_BEGIN

/*
 * ESP32 Hardware DMA-Based SPI Driver
 *
 * Copyright (c) 2020 Nick Wallace
 * Derived from code for ESP8266 hardware SPI by Benoit Anastay.
 * Derived from code for the esp-idf-component library by:
 * Copyright (c) 2020 Ruslan V. Uss <unclerus@gmail.com>
              2021 Tomoyuki Sakurai <y@rombik.org>
 *
 * led_strip_spi: https://github.com/UncleRus/esp-idf-lib/tree/master/components/led_strip_spi
 */
/*
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <driver/spi_master.h>

// Conditional compilation for ESP32-S3 to utilize its flexible SPI capabilities
#if CONFIG_IDF_TARGET_ESP32S3
#pragma message "Targeting ESP32S3, which has better SPI support. Configuring for flexible pin assignment."
#pragma message "DATA_PIN and CLOCK_PIN are used"
#else // Configuration for other ESP32 variants
#pragma error "DMA is only tested to work on ESP32-S3 chips"
#endif

/**
 * Default DMA channel to use. Default is `SPI_DMA_CH_AUTO` for ESP-IDF v4.3
 * and newer, 1 for older versions.
 */

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 3, 0)
#define LED_STRIP_SPI_DEFAULT_DMA_CHAN (1)
#else
#define LED_STRIP_SPI_DEFAULT_DMA_CHAN SPI_DMA_CH_AUTO
#endif

#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(4, 0, 0)
#define LED_STRIP_SPI_DEFAULT_HOST_DEVICE HSPI_HOST
#else
#define LED_STRIP_SPI_DEFAULT_HOST_DEVICE                                                                              \
    SPI2_HOST ///< Default is `SPI2_HOST` (`HSPI_HOST` if `esp-idf` version is v3.x).
#endif
static const char *SPI_TAG = "fastspi_esp32_dma";

template <uint8_t DATA_PIN, uint8_t CLOCK_PIN, uint32_t SPI_SPEED> class ESP32DMASPIOutput
{
    void *dmaBuffer;
    size_t bufferSize;
    spi_host_device_t host_device;     //< SPI host device name, such as `SPI2_HOST`.
    int queue_size;                    ///< Queue size used by `spi_device_queue_trans()`.
    spi_device_handle_t device_handle; ///< Device handle assigned by the driver. The caller must provide this.
    int dma_chan;                      ///< DMA channel to use. Either 1 or 2.
    spi_transaction_t transaction;     ///< SPI transaction used internally by the driver.
    int bytePosition;                  // the position we are at in the DMA buffer

  public:
    // Verify that the pins are valid
    static_assert(FastPin<DATA_PIN>::validpin(), "Invalid data pin specified");
    static_assert(FastPin<CLOCK_PIN>::validpin(), "Invalid clock pin specified");

    ESP32DMASPIOutput()
        : host_device(LED_STRIP_SPI_DEFAULT_HOST_DEVICE), dma_chan(LED_STRIP_SPI_DEFAULT_DMA_CHAN), queue_size(1)
    {
    }

    esp_err_t init(size_t bufferSize)
    {
        ESP_LOGD(SPI_TAG, "Buffer Size is: %d", bufferSize);
        esp_err_t err = ESP_FAIL;

        this->bufferSize = bufferSize;

        spi_bus_config_t bus_config = {0};
        bus_config.mosi_io_num = DATA_PIN;
        bus_config.sclk_io_num = CLOCK_PIN;
        bus_config.miso_io_num = -1;
        bus_config.quadhd_io_num = -1;
        bus_config.quadwp_io_num = -1;
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(4, 4, 0)
        bus_config.data4_io_num = -1;
        bus_config.data5_io_num = -1;
        bus_config.data6_io_num = -1;
#endif
        bus_config.flags = SPICOMMON_BUSFLAG_MASTER;
        bus_config.max_transfer_sz = this->bufferSize;

        spi_device_interface_config_t device_interface_config = {0};
        device_interface_config.clock_speed_hz = SPI_SPEED * 1000000; // translate from MHz to Hz
        device_interface_config.mode = SPI_MODE3;
        device_interface_config.spics_io_num = -1;
        device_interface_config.queue_size = this->queue_size;
        device_interface_config.command_bits = 0;
        device_interface_config.address_bits = 0;
        device_interface_config.dummy_bits = 0;

        this->dmaBuffer = heap_caps_malloc(this->bufferSize, MALLOC_CAP_DMA | MALLOC_CAP_32BIT);
        if (this->dmaBuffer == NULL)
        {
            ESP_LOGE(SPI_TAG, "heap_caps_malloc()");
            err = ESP_ERR_NO_MEM;
            return err;
        }
        memset(this->dmaBuffer, 0, this->bufferSize);

        /* XXX length is in bit */
        this->transaction.length = this->bufferSize * 8;

        ESP_LOGD(SPI_TAG, "SPI buffer initialized");

        err = spi_bus_initialize(this->host_device, &bus_config, this->dma_chan);
        if (err != ESP_OK)
        {
            ESP_LOGE(SPI_TAG, "spi_bus_initialize(): %s", esp_err_to_name(err));
            return err;
        }
        ESP_LOGD(SPI_TAG, "SPI bus initialized");

        err = spi_bus_add_device(this->host_device, &device_interface_config, &this->device_handle);
        if (err != ESP_OK)
        {
            ESP_LOGE(SPI_TAG, "spi_bus_add_device(): %s", esp_err_to_name(err));
            return err;
        }
        ESP_LOGI(SPI_TAG, "LED strip initialized");
        return ESP_OK;
    }
    /*
        // stop the SPI output.  Pretty much a NOP with software, as there's no registers to kick
        static void stop()
        {
        }

        // wait until the SPI subsystem is ready for more data to write.  A NOP when bitbanging
        static void wait() __attribute__((always_inline))
        {
        }
        static void waitFully() __attribute__((always_inline))
        {
            wait();
        }

        static void writeByteNoWait(uint8_t b) __attribute__((always_inline))
        {
            writeByte(b);
        }
        static void writeBytePostWait(uint8_t b) __attribute__((always_inline))
        {
            writeByte(b);
            wait();
        }

        static void writeWord(uint16_t w) __attribute__((always_inline))
        {
            writeByte(w >> 8);
            writeByte(w & 0xFF);
        }

        // naive writeByte implementation, simply calls writeBit on the 8 bits in the byte.
        static void writeByte(uint8_t b)
        {
            ledSPI.transfer(b);
        }
    */
  public:
    esp_err_t flush()
    {
        esp_err_t err = ESP_FAIL;
        spi_transaction_t *t;

        this->transaction.tx_buffer = this->dmaBuffer;
        err = spi_device_queue_trans(this->device_handle, &this->transaction, portMAX_DELAY);
        if (err != ESP_OK)
        {
            ESP_LOGE(SPI_TAG, "spi_device_queue_trans(): %s", esp_err_to_name(err));
            return err;
        }
        err = spi_device_get_trans_result(this->device_handle, &t, portMAX_DELAY);
        if (err != ESP_OK)
        {
            ESP_LOGE(SPI_TAG, "spi_device_get_trans_result(): %s", esp_err_to_name(err));
            return err;
        }
        return ESP_OK;
    }
    void beginDMATransaction()
    {
    }
    // this is a noop for right now
    void waitFully()
    {
    }
    // this just resets the position of our byte marker to zero
    void select()
    {
        bytePosition = 0;
    }

    // send out the data and then reset our byte position
    void release()
    {
        this->flush();
        bytePosition = 0;
        memset(this->dmaBuffer, 0, this->bufferSize);
    }

    void writeByte(uint8_t byte)
    {
        if (bytePosition >= this->bufferSize)
        {
            ESP_LOGE(SPI_TAG, "Attempt to write beyond allocated buffer '%d'", this->bufferSize);
            return;
        }

        reinterpret_cast<uint8_t *>(this->dmaBuffer)[bytePosition] = byte;
        bytePosition++;
    }

    // write a block of len uint8_ts out.  Need to type this better so that explicit casts into the call aren't
    // required. note that this template version takes a class parameter for a per-byte modifier to the data.
    template <class D> void writeBytes(FASTLED_REGISTER uint8_t *data, int len)
    {
        uint8_t *end = data + len;
        while (data != end)
        {
            writeByte(D::adjust(*data++));
        }
        D::postBlock(len);
    }

    // default version of writing a block of data out to the SPI port, with no data modifications being made
    void writeBytes(FASTLED_REGISTER uint8_t *data, int len)
    {
        writeBytes<DATA_NOP>(data, len);
    }

    // write a single bit out, which bit from the passed in byte is determined by template parameter
    template <uint8_t BIT> inline void writeBit(uint8_t b)
    {
        // TODO: This seems broken
        this->writeByte(b);
    }
    void writeWord(uint16_t w) __attribute__((always_inline))
    {
        writeByte(w >> 8);
        writeByte(w & 0xFF);
    }
};

FASTLED_NAMESPACE_END
