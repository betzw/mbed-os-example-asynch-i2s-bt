/* mbed Microcontroller Library
 * Copyright (c) 2015 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include "mbed-drivers/mbed.h"
#include "mbed-drivers/I2S.h"
#include <stdio.h>
#include <stdlib.h>
#include "minar/minar.h"
#include "core-util/Event.h"

#include "sta350bw/sta350bw.h"

/* Change overall behavior */
// #define EARLY_TX_START
// #define USE_MEMCPY
// #define COMPARE_BUFFERS

#ifdef COMPARE_BUFFERS
#include "my_song.inc"
#endif

#if DEVICE_I2S

#if !defined(YOTTA_CFG_HARDWARE_TEST_PINS_I2S_DPIN) ||	\
    !defined(YOTTA_CFG_HARDWARE_TEST_PINS_I2S_SCLK) ||	\
    !defined(YOTTA_CFG_HARDWARE_TEST_PINS_I2S_WSEL) ||	\
    !defined(YOTTA_CFG_HARDWARE_TEST_PINS_I2S_FDPX) ||  \
    !defined(YOTTA_CFG_HARDWARE_TEST_PINS_I2S_MCLK) ||  \
    !defined(YOTTA_CFG_HARDWARE_TEST_PINS_I2C_SDA)  ||	\
    !defined(YOTTA_CFG_HARDWARE_TEST_PINS_I2C_SCL)
#error This example requires I2S & I2C test pins to be defined. Please define the hardware.test-pins.i2s.dpin/sclk/wsel/fdpx yotta confing values
#endif

#ifndef NDEBUG
/* betzw: enable debugging while using sleep modes */
#include "x-nucleo-common/DbgMCU.h"
static DbgMCU enable_dbg;
#endif // !NDEBUG

using namespace minar;

#define AUDIO_FREQ          (44100)

#define DMA_BUF_SAMPLE_NUM  (110)
#define NR_CHANNELS         (2)
#define NR_BYTES_PER_SAMPLE (NR_CHANNELS*sizeof(int16_t))

#define DMA_BUFFER_SIZE     (DMA_BUF_SAMPLE_NUM*NR_BYTES_PER_SAMPLE)
#define DMA_BUF_HALF_SIZE   (DMA_BUFFER_SIZE / 2)

static int8_t dma_buffer_rx[DMA_BUFFER_SIZE];
#ifdef USE_MEMCPY
static int8_t dma_buffer_tx[DMA_BUFFER_SIZE];
#endif

class I2STest {

public:
    I2STest():
	toggle1(PC_0),
	toggle2(PC_1),
	dev_i2c(YOTTA_CFG_HARDWARE_TEST_PINS_I2C_SDA, PullUp, 
		YOTTA_CFG_HARDWARE_TEST_PINS_I2C_SCL, PullUp),
    	sta350(&dev_i2c, PA_10,
	       YOTTA_CFG_HARDWARE_TEST_PINS_I2S_DPIN, YOTTA_CFG_HARDWARE_TEST_PINS_I2S_SCLK,
	       YOTTA_CFG_HARDWARE_TEST_PINS_I2S_WSEL, YOTTA_CFG_HARDWARE_TEST_PINS_I2S_FDPX,
	       YOTTA_CFG_HARDWARE_TEST_PINS_I2S_MCLK),
	recv_i2s(PC_12, PC_10, PA_4) {
    	// reset debug toggles
    	toggle1 = toggle2 = 0;

    	/* configure sound terminal */
    	if(sta350.Init(37, AUDIO_FREQ)) {
	    printf("%s(%d): sta350bw init failed!\r\n", __func__, __LINE__);
	    exit(-1);
    	}

    	/* configure I2S in reception */
    	recv_i2s.audio_frequency(AUDIO_FREQ);
    	recv_i2s.set_mode(MASTER_RX);
    	recv_i2s.format(16, 32);

    	printf("\r\nTransfer test inited!\r\n");
    }
    
    void start_reception() {
        printf("Starting reception\r\n");

        int res = recv_i2s.transfer()
	    .rx((void*)dma_buffer_rx, sizeof(dma_buffer_rx))
	    .callback(I2S::event_callback_t(this, &I2STest::reception_complete_cb), I2S_EVENT_ALL)
	    .circular(true)
	    .apply();
	
        if(res != 0) {
	    error("%s, %d: res=%d\r\n", __func__, __LINE__, res);
        }
    }

    void start_transmission() {
        printf("Starting transmission\r\n");

        int res = sta350.dev_i2s.transfer()
#ifdef USE_MEMCPY
	    .tx((void*)dma_buffer_tx, sizeof(dma_buffer_tx))
#else
	    .tx((void*)dma_buffer_rx, sizeof(dma_buffer_rx))
#endif
	    .callback(I2S::event_callback_t(this, &I2STest::transfer_complete_cb), I2S_EVENT_ALL)
	    .circular(true)
	    .apply();

        if(res != 0) {
	    error("%s, %d: res=%d\r\n", __func__, __LINE__, res);
        }
    }

    void start() {
    	start_reception();
#ifdef EARLY_TX_START
    	start_transmission();
#endif
    }

private:
#ifdef COMPARE_BUFFERS
    void compare_half(bool first) {
	static int16_t *my_dma = (int16_t*)dma_buffer_rx;
	static unsigned long nr_compares = 0;
	unsigned int pos, i, stop;

	/* Wait a bit with comparing after restart */

	if(nr_compares++ < 3000) return;

	if(first) {
	    pos = 0;
	    stop = DMA_BUF_HALF_SIZE/sizeof(int16_t);
	} else {
	    pos = DMA_BUF_HALF_SIZE/sizeof(int16_t);
	    stop = DMA_BUFFER_SIZE/sizeof(int16_t);
	}
	    
	/* find tone start */
	for(; pos < stop; pos += 2) {
	    int val1 = my_dma[pos];
	    int val2 = my_dma[pos+1];
	    int val3 = my_dma[pos+2];

	    if((val3 != 0) && !val1 && !val2) break;
	}
	
	for(i = 0; pos < stop; pos += 2, i += 2) {
	    int val1 = my_dma[pos];
	    int val2 = my_dma[pos+1];

	    i %= sizeof(my_song);

	    if((abs(val1) != abs(my_song[i])) || (abs(val2) != abs(my_song[i+1]))) {
		printf("#%lu [%u, %u]: (%d, %d) (%d, %d)\r\n", 
		       nr_compares, pos/2, i/2,
		       val1, val2,
		       my_song[i], my_song[i+1]);
	    }
	}
    }
#endif // COMPARE_BUFFERS

    void reception_complete_cb(Buffer tx_buffer, Buffer rx_buffer, int narg) {
        (void)tx_buffer;
        (void)rx_buffer;

#ifdef USE_MEMCPY
	if(narg & I2S_EVENT_RX_COMPLETE) {
	    memcpy(&dma_buffer_tx[DMA_BUF_HALF_SIZE], &dma_buffer_rx[DMA_BUF_HALF_SIZE], DMA_BUF_HALF_SIZE);
	} else {
	    memcpy(&dma_buffer_tx[0], &dma_buffer_rx[0], DMA_BUF_HALF_SIZE);
	}
#endif

#ifndef EARLY_TX_START
	static bool first_time = true;

	if(first_time) {
	    first_time = false;
	    start_transmission();
        }
#endif

#ifdef COMPARE_BUFFERS
	if(narg & I2S_EVENT_RX_HALF_COMPLETE) {
	    compare_half(true);
	} else {
	    compare_half(false);
	}
#endif

	if(!(narg & (I2S_EVENT_RX_COMPLETE | I2S_EVENT_RX_HALF_COMPLETE))) {
	    error("Unexpected rx event!\r\n");
	}

        toggle2 = !toggle2;
    }

    void transfer_complete_cb(Buffer tx_buffer, Buffer rx_buffer, int narg) {
        (void)tx_buffer;
        (void)rx_buffer;

	if(!(narg & (I2S_EVENT_TX_COMPLETE | I2S_EVENT_TX_HALF_COMPLETE))) {
	    error("Unexpected tx event!\r\n");
	}

        toggle1 = !toggle1;
    }

private:
    DigitalOut toggle1; // betzw: for debug only
    DigitalOut toggle2; // betzw: for debug only
    DevI2C dev_i2c;
    STA350BW sta350;
    I2S recv_i2s;
};

void app_start(int, char*[]) {
    static I2STest test;
    Scheduler::postCallback(mbed::util::FunctionPointer0<void>(&test, &I2STest::start).bind());
}

#else
void app_start(int, char*[]) {
    printf("The target does not support I2S (asynch) API.\r\n");
}
#endif
