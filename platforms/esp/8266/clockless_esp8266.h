#pragma once

FASTLED_NAMESPACE_BEGIN

#ifdef FASTLED_DEBUG_COUNT_FRAME_RETRIES
extern uint32_t _frame_cnt;
extern uint32_t _retry_cnt;
#endif

#ifdef FASTLED_RGBW

#ifdef FASTLED_HAS_PRAGMA_MESSAGE
#    pragma message "Using RGBW controller as per FASTLED_RGBW define"
#else
#    warning "Using RGBW controller as per FASTLED_RGBW define"
#endif
#endif

// Info on reading cycle counter from https://github.com/kbeckmann/nodemcu-firmware/blob/ws2812-dual/app/modules/ws2812.c
__attribute__ ((always_inline)) inline static uint32_t __clock_cycles() {
  uint32_t cyc;
  __asm__ __volatile__ ("rsr %0,ccount":"=a" (cyc));
  return cyc;
}

typedef struct {
    uint32_t c0;
    uint32_t c1;
    uint32_t c2;
} out_4px;




#define FASTLED_HAS_CLOCKLESS 1

template <int DATA_PIN, int T1, int T2, int T3, EOrder RGB_ORDER = RGB, int XTRA0 = 0, bool FLIP = false, int WAIT_TIME = 9, bool USE_RGBW = false>
class ClocklessController : public CPixelLEDController<RGB_ORDER, 1, 0xFFFFFFFF, USE_RGBW> {
	typedef typename FastPin<DATA_PIN>::port_ptr_t data_ptr_t;
	typedef typename FastPin<DATA_PIN>::port_t data_t;

	data_t mPinMask;
	data_ptr_t mPort;
	CMinWait<WAIT_TIME> mWait;
public:
	virtual void init() {
		FastPin<DATA_PIN>::setOutput();
		mPinMask = FastPin<DATA_PIN>::mask();
		mPort = FastPin<DATA_PIN>::port();
	}

	virtual uint16_t getMaxRefreshRate() const { return 400; }

protected:

	virtual void showPixels(PixelController<RGB_ORDER> & pixels) {
    // mWait.wait();
		int cnt = FASTLED_INTERRUPT_RETRY_COUNT;
    while((showRGBInternal(pixels)==0) && cnt--) {
      #ifdef FASTLED_DEBUG_COUNT_FRAME_RETRIES
      _retry_cnt++;
      #endif
      os_intr_unlock();
      delayMicroseconds(WAIT_TIME);
      os_intr_lock();
    }
    // mWait.mark();
  }


#define _ESP_ADJ (0)
#define _ESP_ADJ2 (0)

	template<int BITS> __attribute__ ((always_inline)) inline static void writeBits(register uint32_t & last_mark, register uint32_t b)  {
    b = ~b; b <<= 24;
    for(register uint32_t i = BITS; i > 0; i--) {
      while((__clock_cycles() - last_mark) < (T1+T2+T3));
			last_mark = __clock_cycles();
      FastPin<DATA_PIN>::hi();

      while((__clock_cycles() - last_mark) < T1);
      if(b & 0x80000000L) { FastPin<DATA_PIN>::lo(); }
      b <<= 1;

      while((__clock_cycles() - last_mark) < (T1+T2));
      FastPin<DATA_PIN>::lo();
		}
	}

	// This method is made static to force making register Y available to use for data on AVR - if the method is non-static, then
	// gcc will use register Y for the this pointer.
	static uint32_t ICACHE_RAM_ATTR showRGBInternal(PixelController<RGB_ORDER> pixels) {
		// Setup the pixel controller and load/scale the first byte
		pixels.preStepFirstByteDithering();
#ifdef FASTLED_RGBW
        register out_4px output;
		register  uint32_t minc;
		output.c0 = pixels.loadAndScale0();
#else
		register uint32_t b = pixels.loadAndScale0();
#endif // FASTLED_RGBW
    pixels.preStepFirstByteDithering();


		os_intr_lock();
    uint32_t start = __clock_cycles();
		uint32_t last_mark = start;
		while(pixels.has(1)) {
#ifdef FASTLED_RGBW
            output.c1 = pixels.loadAndScale1();
            output.c2 = pixels.loadAndScale2();

            minc = min(output.c0, output.c1);
            minc = min(output.c2, minc);

            writeBits<8+XTRA0>(last_mark, output.c0 - minc);

            // Write first byte, read next byte

			// Write second byte, read 3rd byte
			writeBits<8+XTRA0>(last_mark, output.c1 - minc);

			// Write third byte, read 1st byte of next pixel
			writeBits<8+XTRA0>(last_mark, output.c2 - minc);

			// write out all 0's for the 4th byte for RGBW strips - this is a
			// hack and will break your RGB WS2812 strips.
			writeBits<8+XTRA0>(last_mark, minc);

            output.c0 = pixels.advanceAndLoadAndScale0();
#else
            //Serial.printf("0 - %0x\n", b);
			// Write first byte, read next byte
			writeBits<8+XTRA0>(last_mark, b);
			b = pixels.loadAndScale1();

            //Serial.printf("1 - %0x\n", b);
			// Write second byte, read 3rd byte
			writeBits<8+XTRA0>(last_mark, b);
			b = pixels.loadAndScale2();

            //Serial.printf("2 - %0x\n", b);
			if(pixels.mUseRgbw) {
				// Write third byte, read 4rd byte
				writeBits<8 + XTRA0>(last_mark, b);
				b = pixels.loadAndScale3();
			}

            //Serial.printf("3 - %0x\n", b);
			// Write third byte, read 1st byte of next pixel
			writeBits<8+XTRA0>(last_mark, b);



			b = pixels.advanceAndLoadAndScale0();
#endif // FASTLED_RGBW


#if (FASTLED_ALLOW_INTERRUPTS == 1)
			os_intr_unlock();
			#endif

          pixels.stepDithering();

			#if (FASTLED_ALLOW_INTERRUPTS == 1)
			os_intr_lock();
			// if interrupts took longer than 45µs, punt on the current frame
			if((int32_t)(__clock_cycles()-last_mark) > 0) {
				if((int32_t)(__clock_cycles()-last_mark) > (T1+T2+T3+((WAIT_TIME-INTERRUPT_THRESHOLD)*CLKS_PER_US))) { sei(); return 0; }
			}
			#endif
		};

		os_intr_unlock();
    #ifdef FASTLED_DEBUG_COUNT_FRAME_RETRIES
    _frame_cnt++;
    #endif
		return __clock_cycles() - start;
	}
};

FASTLED_NAMESPACE_END
