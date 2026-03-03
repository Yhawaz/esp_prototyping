#pragma once

#define INPUT   0x01
#define OUTPUT  0x03
#define LOW     0x0
#define HIGH    0x1
#define RISING  0x01
#define FALLING 0x02

#include "RadioLib.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "EspIdfHal.h"
#include <cstdio>

// pin assignments — change to match your wiring
#define PIN_MOSI   8
#define PIN_MISO   4
#define PIN_SCK    5
#define PIN_CS     9
#define PIN_RST    2
#define PIN_DIO0   6
#define PIN_DIO1   3

extern "C" void app_main(void) {
    EspIdfHal *hal = new EspIdfHal(PIN_MOSI, PIN_MISO, PIN_SCK);

    SX1276 radio = new Module(hal, PIN_CS, PIN_DIO0, PIN_RST, PIN_DIO1);
    //SX1276 radio = new Module(hal, PIN_CS, RADIOLIB_NC, PIN_RST, RADIOLIB_NC);

    printf("Initializing SX1276...\n");
    int state = radio.begin(
        915.0,   // freq MHz (use 868.0 for EU)
        125.0,   // bandwidth kHz
        7,       // spreading factor
        5,       // coding rate 4/5
        0x12,    // sync word
        17,      // output power dBm
        8        // preamble length
    );

    if (state != RADIOLIB_ERR_NONE) {
        printf("Radio init failed, code %d\n", state);
        return;
    }
    printf("Radio init OK\n");

    // transmit
    while(1){
        state = radio.transmit("0XDEADBEEF MEOW EMOW MEOW");
        if (state == RADIOLIB_ERR_NONE) {
            printf("TX success\n");
        } else {
            printf("TX failed, code %d\n", state);
        }
        vTaskDelay(500 / portTICK_PERIOD_MS); // Delay for 300 millisecond
    }

    // receive (blocking)
    uint8_t buf[256];
    state = radio.receive(buf, sizeof(buf));
    if (state == RADIOLIB_ERR_NONE) {
        printf("RX: %s, RSSI: %.1f dBm\n", (char*)buf, radio.getRSSI());
    }
}
