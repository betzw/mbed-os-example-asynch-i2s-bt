/*
 * ARM mbed OS 5 example for asynchronous I2S/DMA streaming between BT front-end & back-end based on STa350BW
 */
#include "mbed.h"
#include "I2S.h"
#include <stdio.h>
#include <stdlib.h>

#include "sta350bw/sta350bw.h"

/* Change overall behavior */
// #define EARLY_TX_START
// #define USE_MEMCPY

#if DEVICE_I2S

#define I2S_DPIN (PB_15)
#define I2S_SCLK (PB_13)
#define I2S_WSEL (PB_12)
#define I2S_FDPX (NC)
#define I2S_MCLK (PC_6)

#ifndef NDEBUG
/* betzw: enable debugging while using sleep modes */
#include "x-nucleo-common/DbgMCU.h"
static DbgMCU enable_dbg;
#endif // !NDEBUG

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
	dev_i2c(I2C_SDA, PullUp, 
		I2C_SCL, PullUp),
    	sta350(&dev_i2c, PA_10,
	       I2S_DPIN, I2S_SCLK,
	       I2S_WSEL, I2S_FDPX,
	       I2S_MCLK),
	recv_i2s(PC_12, PC_10, PA_4) {
    	// reset debug toggles
    	toggle1 = toggle2 = 0;

    	/* configure sound terminal */
    	if(sta350.Init(37, AUDIO_FREQ)) {
    		printf("%s(%d): sta350bw init failed!\r\n", __func__, __LINE__);
    		exit(-1);
    	}
    	sta350.dev_i2s.set_mode(MASTER_TX, true);

    	/* configure I2S in reception */
    	recv_i2s.audio_frequency(AUDIO_FREQ);
    	recv_i2s.set_mode(MASTER_RX, true);
    	recv_i2s.format(16, 32);

    	printf("\r\nTransfer test inited!\r\n");
    }
    
    void start_reception() {
        printf("Starting reception\r\n");

        int res = recv_i2s.transfer(
        		(void*)NULL, 0,
				(void*)dma_buffer_rx, sizeof(dma_buffer_rx),
				event_callback_t(this, &I2STest::reception_complete_cb),
				I2S_EVENT_ALL);

        if(res != 0) {
        	error("%s, %d: res=%d\r\n", __func__, __LINE__, res);
        }
    }

    void start_transmission() {
        printf("Starting transmission\r\n");

        int res = sta350.dev_i2s.transfer(
#ifdef USE_MEMCPY
        		(void*)dma_buffer_tx, sizeof(dma_buffer_tx),
#else
				(void*)dma_buffer_rx, sizeof(dma_buffer_rx),
#endif
				(void*)NULL, 0,
				event_callback_t(this, &I2STest::transfer_complete_cb),
				I2S_EVENT_ALL);

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
    void reception_complete_cb(int narg) {
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

    	if(!(narg & (I2S_EVENT_RX_COMPLETE | I2S_EVENT_RX_HALF_COMPLETE))) {
    		error("Unexpected rx event!\r\n");
    	}

        toggle2 = !toggle2;
    }

    void transfer_complete_cb(int narg) {
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

int main() {
    static I2STest test;

    test.start();
}

#else
int main() {
    printf("The target does not support I2S (asynch) API.\r\n");
}
#endif
