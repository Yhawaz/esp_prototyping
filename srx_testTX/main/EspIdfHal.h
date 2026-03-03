#pragma once

#include "RadioLib.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

class EspIdfHal : public RadioLibHal {
public:
    EspIdfHal(int8_t spiMosi, int8_t spiMiso, int8_t spiClk,
              spi_host_device_t spiHost = SPI2_HOST, uint32_t spiFreq = 8000000)
        : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING),
          _spiMosi(spiMosi), _spiMiso(spiMiso), _spiClk(spiClk),
          _spiHost(spiHost), _spiFreq(spiFreq), _spiHandle(nullptr) {}

    void init() override {
        spiBegin();
    }

    void term() override {
        spiEnd();
    }

    // GPIO
    void pinMode(uint32_t pin, uint32_t mode) override {
        if (pin == RADIOLIB_NC) return;
        gpio_config_t cfg = {};
        cfg.pin_bit_mask = (1ULL << pin);
        cfg.mode = (mode == OUTPUT) ? GPIO_MODE_OUTPUT : GPIO_MODE_INPUT;
        cfg.pull_up_en = GPIO_PULLUP_DISABLE;
        cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
        cfg.intr_type = GPIO_INTR_DISABLE;
        gpio_config(&cfg);
    }

    void digitalWrite(uint32_t pin, uint32_t value) override {
        if (pin == RADIOLIB_NC) return;
        gpio_set_level((gpio_num_t)pin, value);
    }

    uint32_t digitalRead(uint32_t pin) override {
        if (pin == RADIOLIB_NC) return 0;
        return gpio_get_level((gpio_num_t)pin);
    }

    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void),
                         uint32_t mode) override {
        if (interruptNum == RADIOLIB_NC) return;
        gpio_num_t pin = (gpio_num_t)interruptNum;
        gpio_set_intr_type(pin, (mode == RISING) ? GPIO_INTR_POSEDGE : GPIO_INTR_NEGEDGE);
        gpio_install_isr_service(0);
        gpio_isr_handler_add(pin, (gpio_isr_t)interruptCb, nullptr);
        gpio_intr_enable(pin);
    }

    void detachInterrupt(uint32_t interruptNum) override {
        if (interruptNum == RADIOLIB_NC) return;
        gpio_isr_handler_remove((gpio_num_t)interruptNum);
    }

    // Timing
    void delay(unsigned long ms) override {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }

    void delayMicroseconds(unsigned long us) override {
        esp_rom_delay_us(us);
    }

    unsigned long millis() override {
        return (unsigned long)(esp_timer_get_time() / 1000);
    }

    unsigned long micros() override {
        return (unsigned long)esp_timer_get_time();
    }

    long pulseIn(uint32_t pin, uint32_t state,
                 unsigned long timeout) override {
        // not needed for LoRa
        return 0;
    }

    // SPI
    void spiBegin() {
        spi_bus_config_t busCfg = {};
        busCfg.mosi_io_num = _spiMosi;
        busCfg.miso_io_num = _spiMiso;
        busCfg.sclk_io_num = _spiClk;
        busCfg.quadwp_io_num = -1;
        busCfg.quadhd_io_num = -1;
        busCfg.max_transfer_sz = 256;
        spi_bus_initialize(_spiHost, &busCfg, SPI_DMA_CH_AUTO);
    }

    void spiBeginTransaction() override {
        // device added here with CS handled by RadioLib
        if (_spiHandle == nullptr) {
            spi_device_interface_config_t devCfg = {};
            devCfg.clock_speed_hz = _spiFreq;
            devCfg.mode = 0;
            devCfg.spics_io_num = -1;  // RadioLib handles CS
            devCfg.queue_size = 1;
            spi_bus_add_device(_spiHost, &devCfg, &_spiHandle);
        }
    }

    void spiTransfer(uint8_t *out, size_t len, uint8_t *in) override {
        spi_transaction_t t = {};
        t.length = len * 8;
        t.tx_buffer = out;
        t.rx_buffer = in;
        spi_device_transmit(_spiHandle, &t);
    }

    void spiEndTransaction() override {
        // nothing needed
    }

    void spiEnd() {
        if (_spiHandle) {
            spi_bus_remove_device(_spiHandle);
            _spiHandle = nullptr;
        }
        spi_bus_free(_spiHost);
    }

private:
    int8_t _spiMosi, _spiMiso, _spiClk;
    spi_host_device_t _spiHost;
    uint32_t _spiFreq;
    spi_device_handle_t _spiHandle;
};
