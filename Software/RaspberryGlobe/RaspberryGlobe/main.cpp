#include <iostream>
#include <stdint.h>
#include <cstdio>
#include <stdio.h>
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include "definitions.h"
#include <chrono>
#include "bcm2835.h"

void config_SPI(void);
void WriteLED(led);
void WriteUInt32(uint32_t);
void WriteFrame(int);
void ReedIR(void);

/*volatile*/ unsigned int frameIndex;
std::chrono::steady_clock::time_point lastIRTime;
/*volatile*/ int timeInterval;
std::chrono::steady_clock::time_point currentTime;

using namespace std;

//volatile int int_count = 0;

int main()
{
	//Set max priority for this program
	/*struct sched_param sp;
	memset(&sp, 0, sizeof(sp));
	sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
	sched_setscheduler(0, SCHED_FIFO, &sp);
	mlockall(MCL_CURRENT | MCL_FUTURE);*/

	int result;
	result = wiringPiSetup();	//setup GPIOs
	//result = piHiPri(60);
	bcm2835_init();	//init bcm2835 library
	//setup SPI
	config_SPI();
	
	//Reed
	pinMode(REED, INPUT);
	bool reedBefore = LOW;
	//result = wiringPiISR(REED, INT_EDGE_RISING, ReedIR);	//setup Reed-Interrupt
	//unsigned int frameIndexOld = -1;
	frameIndex = 0;
	timeInterval = 50000;

	//WriteFrame(0);
	while (true)
	{
		bool reedNow = digitalRead(REED);
		if (reedBefore == LOW && reedNow == HIGH)	//Rising Edge
		{
			ReedIR();
		}
		reedBefore = reedNow;
		if (frameIndex < POSITIONS && timeInterval > 0)	//stop showing new frames when everything was shown, but reed IR did not occour
		{
			currentTime = chrono::steady_clock::now();
			int rotTime = chrono::duration_cast<chrono::nanoseconds>(currentTime - lastIRTime).count();
			
			if (rotTime >= timeInterval * frameIndex)
			{
				WriteFrame(frameIndex);
				frameIndex++;
				/*if (frameIndex == 180)
				{
					frameIndex = 0;
					lastIRTime = chrono::steady_clock::now();
				}*/
				//frameIndex = (frameIndex + 1) % 180;
			}
		}
	}
    return 0;
}

void ReedIR()
{
	frameIndex = 0;
	auto ellapsed = chrono::steady_clock::now();
	timeInterval = chrono::duration_cast<chrono::nanoseconds>(ellapsed - lastIRTime).count();
	timeInterval /= POSITIONS;
	lastIRTime = ellapsed;
	//int_count = (int_count + 1) % 100;
}

void config_SPI()
{
	bcm2835_spi_chipSelect(BCM2835_SPI_CS_NONE);	//Don't use a chip select
	bcm2835_spi_setClockDivider(16);	//Set clock divider to 6 (250MHz/6 = ~ 40 MHz)
	bcm2835_spi_setDataMode(0);	//Data Mode: Clock idle 0, Capture on rising, output on falling edge
	bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST); //MSB first
	bcm2835_spi_begin();	//Setup Peripheral
}

inline void WriteFrame(int index)
{
	//bcm2835_spi_writenb((const char *)&test[0][0], 4 * (NUM_LEDS + ADDITIONAL_FRAMES));	//writes start(4B), end(4B) and led_data(NUM_LEDS * 4B)
	bcm2835_spi_writenb((const char *)&pic[index][0], 4 * (NUM_LEDS + ADDITIONAL_FRAMES));	//writes start(4B), end(4B) and led_data(NUM_LEDS * 4B)
}

/*void WriteLED(led l) {
	uint8_t led_frame[4];
	led_frame[0] = 0b11100000 | (0b00011111);	//&l.brightness);
	led_frame[1] = l.blue;
	led_frame[2] = l.green;
	led_frame[3] = l.red;

	wiringPiSPIDataRW(0, led_frame, 4);
}

void WriteUInt32(uint32_t data) {
	uint8_t led_frame[4];
	led_frame[0] = 0b11100000 | (0b00010000);	//&l.brightness);
	led_frame[1] = (uint8_t)((data >> 16) & 0xFF);
	led_frame[2] = (uint8_t)((data >> 8) & 0xFF);
	led_frame[3] = (uint8_t)(data & 0xFF);

	wiringPiSPIDataRW(0, led_frame, 4);
}*/

