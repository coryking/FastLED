#pragma once

FASTLED_NAMESPACE_BEGIN

// TODO: FIX THIS TOO!
#ifdef FASTLED_RGBW
#define BYTES_PER_ELEMENT 4
#else
#define BYTES_PER_ELEMENT 4
#endif

#ifdef FASTLED_DEBUG_COUNT_FRAME_RETRIES
extern uint32_t _frame_cnt;
extern uint32_t _retry_cnt;
#endif

#ifdef FASTLED_HAS_PRAGMA_MESSAGE
#    pragma message "Using DMA-based RGBW controller as per FASTLED_RGBW define"
#else
#    warning "Using DMA-Based RGBW controller as per FASTLED_RGBW define"
#endif


extern "C"
{
#include "Arduino.h"
#include "osapi.h"
#include "ets_sys.h"

#include "i2s_reg.h"
#include "i2s.h"
#include "eagle_soc.h"
#include "esp8266_peri.h"
#include "slc_register.h"

#include "osapi.h"
#include "ets_sys.h"
#include "user_interface.h"

void rom_i2c_writeReg_Mask(uint32_t block, uint32_t host_id, uint32_t reg_add, uint32_t Msb, uint32_t Lsb, uint32_t indata);
}

struct slc_queue_item
{
    uint32  blocksize : 12;
    uint32  datalen : 12;
    uint32  unused : 5;
    uint32  sub_sof : 1;
    uint32  eof : 1;
    uint32  owner : 1;
    uint32  buf_ptr;
    uint32  next_link_ptr;
};

class NeoEsp8266DmaSpeedWs2813
{
public:
    const static uint32_t I2sClockDivisor = 3;
    const static uint32_t I2sBaseClockDivisor = 16;
    const static uint32_t ResetTimeUs = 250;
};

class NeoEsp8266DmaSpeed800Kbps
{
public:
    const static uint32_t I2sClockDivisor = 3;
    const static uint32_t I2sBaseClockDivisor = 16;
    const static uint32_t ResetTimeUs = 50;
};

class NeoEsp8266DmaSpeed400Kbps
{
public:
    const static uint32_t I2sClockDivisor = 6;
    const static uint32_t I2sBaseClockDivisor = 16;
    const static uint32_t ResetTimeUs = 50;
};

enum NeoDmaState
{
    NeoDmaState_Idle,
    NeoDmaState_Pending,
    NeoDmaState_Sending,
};
const uint16_t c_maxDmaBlockSize = 4095;
const uint16_t c_dmaBytesPerPixelBytes = 4;
const uint8_t c_I2sPin = 3; // due to I2S hardware, the pin used is restricted to this


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

template <int DATA_PIN, int T1, int T2, int T3, EOrder RGB_ORDER = RGB, int XTRA0 = 0, bool FLIP = false, int WAIT_TIME = 9, bool USE_RGBW=false, typename T_SPEED=NeoEsp8266DmaSpeed800Kbps>
class ClocklessController : public CPixelLEDController<RGB_ORDER> {
    typedef typename FastPin<DATA_PIN>::port_ptr_t data_ptr_t;
    typedef typename FastPin<DATA_PIN>::port_t data_t;

    data_t mPinMask;
    data_ptr_t mPort;
    CMinWait<WAIT_TIME> mWait;
public:
    virtual void init() {}
    virtual void initLedBuffers() override {
        this->InitI2S();
    }

    void InitI2S()
    {
        uint16_t dmaPixelSize = c_dmaBytesPerPixelBytes * BYTES_PER_ELEMENT;

        _pixelsSize = this->size() * BYTES_PER_ELEMENT;
        _i2sBufferSize = this->size() * dmaPixelSize;

        _i2sBuffer = (uint8_t*)malloc(_i2sBufferSize);
        memset(_i2sBuffer, 0x00, _i2sBufferSize);

        memset(_i2sZeroes, 0x00, sizeof(_i2sZeroes));

        _is2BufMaxBlockSize = (c_maxDmaBlockSize / dmaPixelSize) * dmaPixelSize;

        _i2sBufDescCount = (_i2sBufferSize / _is2BufMaxBlockSize) + 1 + 2; // need two more for state/latch blocks
        _i2sBufDesc = (slc_queue_item*)malloc(_i2sBufDescCount * sizeof(slc_queue_item));

        s_this = this; // store this for the ISR

        StopDma();
        _dmaState = NeoDmaState_Sending; // start off sending empty buffer

        uint8_t* is2Buffer = _i2sBuffer;
        uint32_t is2BufferSize = _i2sBufferSize;
        uint16_t indexDesc;

        // prepare main data block decriptors that point into our one static dma buffer
        for (indexDesc = 0; indexDesc < (_i2sBufDescCount - 2); indexDesc++)
        {
            uint32_t blockSize = (is2BufferSize > _is2BufMaxBlockSize) ? _is2BufMaxBlockSize : is2BufferSize;

            _i2sBufDesc[indexDesc].owner = 1;
            _i2sBufDesc[indexDesc].eof = 0; // no need to trigger interrupt generally
            _i2sBufDesc[indexDesc].sub_sof = 0;
            _i2sBufDesc[indexDesc].datalen = blockSize;
            _i2sBufDesc[indexDesc].blocksize = blockSize;
            _i2sBufDesc[indexDesc].buf_ptr = (uint32_t)is2Buffer;
            _i2sBufDesc[indexDesc].unused = 0;
            _i2sBufDesc[indexDesc].next_link_ptr = (uint32_t)&(_i2sBufDesc[indexDesc + 1]);

            is2Buffer += blockSize;
            is2BufferSize -= blockSize;
        }

        // prepare the two state/latch descriptors
        for (; indexDesc < _i2sBufDescCount; indexDesc++)
        {
            _i2sBufDesc[indexDesc].owner = 1;
            _i2sBufDesc[indexDesc].eof = 0; // no need to trigger interrupt generally
            _i2sBufDesc[indexDesc].sub_sof = 0;
            _i2sBufDesc[indexDesc].datalen = sizeof(_i2sZeroes);
            _i2sBufDesc[indexDesc].blocksize = sizeof(_i2sZeroes);
            _i2sBufDesc[indexDesc].buf_ptr = (uint32_t)_i2sZeroes;
            _i2sBufDesc[indexDesc].unused = 0;
            _i2sBufDesc[indexDesc].next_link_ptr = (uint32_t)&(_i2sBufDesc[indexDesc + 1]);
        }

        // the first state block will trigger the interrupt
        _i2sBufDesc[indexDesc - 2].eof = 1;
        // the last state block will loop to the first state block by defualt
        _i2sBufDesc[indexDesc - 1].next_link_ptr = (uint32_t)&(_i2sBufDesc[indexDesc - 2]);

        // setup the rest of i2s DMA
        //
        ETS_SLC_INTR_DISABLE();
        SLCC0 |= SLCRXLR | SLCTXLR;
        SLCC0 &= ~(SLCRXLR | SLCTXLR);
        SLCIC = 0xFFFFFFFF;

        // Configure DMA
        SLCC0 &= ~(SLCMM << SLCM); // clear DMA MODE
        SLCC0 |= (1 << SLCM); // set DMA MODE to 1
        SLCRXDC |= SLCBINR | SLCBTNR; // enable INFOR_NO_REPLACE and TOKEN_NO_REPLACE
        SLCRXDC &= ~(SLCBRXFE | SLCBRXEM | SLCBRXFM); // disable RX_FILL, RX_EOF_MODE and RX_FILL_MODE

        // Feed DMA the 1st buffer desc addr
        // To send data to the I2S subsystem, counter-intuitively we use the RXLINK part, not the TXLINK as you might
        // expect. The TXLINK part still needs a valid DMA descriptor, even if it's unused: the DMA engine will throw
        // an error at us otherwise. Just feed it any random descriptor.
        SLCTXL &= ~(SLCTXLAM << SLCTXLA); // clear TX descriptor address
        SLCTXL |= (uint32)&(_i2sBufDesc[_i2sBufDescCount-1]) << SLCTXLA; // set TX descriptor address. any random desc is OK, we don't use TX but it needs to be valid
        SLCRXL &= ~(SLCRXLAM << SLCRXLA); // clear RX descriptor address
        SLCRXL |= (uint32)_i2sBufDesc << SLCRXLA; // set RX descriptor address

        ETS_SLC_INTR_ATTACH(i2s_slc_isr, NULL);
        SLCIE = SLCIRXEOF; // Enable only for RX EOF interrupt

        ETS_SLC_INTR_ENABLE();

        //Start transmission
        SLCTXL |= SLCTXLS;
        SLCRXL |= SLCRXLS;

        pinMode(c_I2sPin, FUNCTION_1); // I2S0_DATA

        I2S_CLK_ENABLE();
        I2SIC = 0x3F;
        I2SIE = 0;

        //Reset I2S
        I2SC &= ~(I2SRST);
        I2SC |= I2SRST;
        I2SC &= ~(I2SRST);

        I2SFC &= ~(I2SDE | (I2STXFMM << I2STXFM) | (I2SRXFMM << I2SRXFM)); // Set RX/TX FIFO_MOD=0 and disable DMA (FIFO only)
        I2SFC |= I2SDE; //Enable DMA
        I2SCC &= ~((I2STXCMM << I2STXCM) | (I2SRXCMM << I2SRXCM)); // Set RX/TX CHAN_MOD=0

        // set the rate
        uint32_t i2s_clock_div = T_SPEED::I2sClockDivisor & I2SCDM;
        uint8_t i2s_bck_div = T_SPEED::I2sBaseClockDivisor & I2SBDM;

        //!trans master, !bits mod, rece slave mod, rece msb shift, right first, msb right
        I2SC &= ~(I2STSM | (I2SBMM << I2SBM) | (I2SBDM << I2SBD) | (I2SCDM << I2SCD));
        I2SC |= I2SRF | I2SMR | I2SRSM | I2SRMS | (i2s_bck_div << I2SBD) | (i2s_clock_div << I2SCD);

        I2SC |= I2STXS; // Start transmission
    }



    virtual uint16_t getMaxRefreshRate() const { return 400; }
    ~ClocklessController()
    {
        StopDma();

        free(_i2sBuffer);
        free(_i2sBufDesc);
    }
protected:

    virtual void showPixels(PixelController<RGB_ORDER> & pixels) {
        // mWait.wait();
        // wait for not actively sending data
        while (_dmaState != NeoDmaState_Idle)
        {
            yield();
        }
        showRGBInternal((uint16_t*)_i2sBuffer, pixels);
        _dmaState = NeoDmaState_Pending;

        // mWait.mark();
    }


#define _ESP_ADJ (0)
#define _ESP_ADJ2 (0)

    __attribute__ ((always_inline)) inline static uint16_t* writeBits(uint16_t* pDma, register uint32_t b)  {
        const uint16_t bitpatterns[16] =
                {
                        0b1000100010001000, 0b1000100010001110, 0b1000100011101000, 0b1000100011101110,
                        0b1000111010001000, 0b1000111010001110, 0b1000111011101000, 0b1000111011101110,
                        0b1110100010001000, 0b1110100010001110, 0b1110100011101000, 0b1110100011101110,
                        0b1110111010001000, 0b1110111010001110, 0b1110111011101000, 0b1110111011101110,
                };

        *(pDma++) = bitpatterns[((b) & 0x0f)];
        *(pDma++) = bitpatterns[((b) >> 4) & 0x0f];

        return pDma;
    }

    // This method is made static to force making register Y available to use for data on AVR - if the method is non-static, then
    // gcc will use register Y for the this pointer.
    static uint32_t ICACHE_RAM_ATTR showRGBInternal(uint16_t* i2sBuffer, PixelController<RGB_ORDER> pixels) {
        // Setup the pixel controller and load/scale the first byte
        pixels.preStepFirstByteDithering();

        uint16_t* pDma = (uint16_t*)i2sBuffer;

        const uint16_t bitpatterns[16] =
                {
                        0b1000100010001000, 0b1000100010001110, 0b1000100011101000, 0b1000100011101110,
                        0b1000111010001000, 0b1000111010001110, 0b1000111011101000, 0b1000111011101110,
                        0b1110100010001000, 0b1110100010001110, 0b1110100011101000, 0b1110100011101110,
                        0b1110111010001000, 0b1110111010001110, 0b1110111011101000, 0b1110111011101110,
                };

#ifdef FASTLED_RGBW
        register  uint32_t minc;
        register out_4px output;
        output.c0 = pixels.loadAndScale0();

#else
        register uint32_t b = pixels.loadAndScale0();
        pixels.preStepFirstByteDithering();
#endif


        while(pixels.has(1)) {
#ifdef FASTLED_RGBW
            output.c1 = pixels.loadAndScale1();
            output.c2 = pixels.loadAndScale2();

            minc = min(output.c0, output.c1);
            minc = min(output.c2, minc);

            pDma = writeBits(pDma, output.c0 - minc);
            pDma = writeBits(pDma, output.c1 - minc);
            pDma = writeBits(pDma, output.c2 - minc);
            pDma = writeBits(pDma, minc);

            output.c0 = pixels.advanceAndLoadAndScale0();
#else

            pDma = writeBits(pDma, b);

            b = pixels.loadAndScale1();
            pDma = writeBits(pDma, b);

            b = pixels.loadAndScale2();
            pDma = writeBits(pDma, b);

            if(USE_RGBW) {
                b = pixels.loadAndScale3();
                pDma = writeBits(pDma, b);
            }

            b = pixels.advanceAndLoadAndScale0();
#endif // FASTLED_RGBW

            pixels.stepDithering();
        };
#ifdef FASTLED_DEBUG_COUNT_FRAME_RETRIES
        _frame_cnt++;
#endif
        return 0;
    }
private:
    static ClocklessController* s_this; // for the ISR

    size_t    _pixelsSize;    // Size of '_pixels' buffer

    uint32_t _i2sBufferSize; // total size of _i2sBuffer
    uint8_t* _i2sBuffer;  // holds the DMA buffer that is referenced by _i2sBufDesc

    // normally 24 bytes creates the minimum 50us latch per spec, but
    // with the new logic, this latch is used to space between three states
    // buffer size = (24 * (speed / 50)) / 3
    uint8_t _i2sZeroes[(24L * (T_SPEED::ResetTimeUs / 50L)) / 3L];

    slc_queue_item* _i2sBufDesc;  // dma block descriptors
    uint16_t _i2sBufDescCount;   // count of block descriptors in _i2sBufDesc
    uint16_t _is2BufMaxBlockSize; // max size based on size of a pixel of a single block

    volatile NeoDmaState _dmaState;

    // This routine is called as soon as the DMA routine has something to tell us. All we
    // handle here is the RX_EOF_INT status, which indicate the DMA has sent a buffer whose
    // descriptor has the 'EOF' field set to 1.
    // in the case of this code, the second to last state descriptor
    volatile static void ICACHE_RAM_ATTR i2s_slc_isr(void)
    {
        uint32_t slc_intr_status = SLCIS;

        SLCIC = 0xFFFFFFFF;

        if (slc_intr_status & SLCIRXEOF)
        {
            ETS_SLC_INTR_DISABLE();

            switch (s_this->_dmaState)
            {
                case NeoDmaState_Idle:
                    break;

                case NeoDmaState_Pending:
                {
                    slc_queue_item* finished_item = (slc_queue_item*)SLCRXEDA;

                    // data block has pending data waiting to send, prepare it
                    // point last state block to top
                    (finished_item + 1)->next_link_ptr = (uint32_t)(s_this->_i2sBufDesc);

                    s_this->_dmaState = NeoDmaState_Sending;
                }
                    break;

                case NeoDmaState_Sending:
                {
                    slc_queue_item* finished_item = (slc_queue_item*)SLCRXEDA;

                    // the data block had actual data sent
                    // point last state block to first state block thus
                    // just looping and not sending the data blocks
                    (finished_item + 1)->next_link_ptr = (uint32_t)(finished_item);

                    s_this->_dmaState = NeoDmaState_Idle;
                }
                    break;
            }


            ETS_SLC_INTR_ENABLE();
        }
    }

    void StopDma()
    {
        ETS_SLC_INTR_DISABLE();
        SLCIC = 0xFFFFFFFF;
        SLCIE = 0;
        SLCTXL &= ~(SLCTXLAM << SLCTXLA); // clear TX descriptor address
        SLCRXL &= ~(SLCRXLAM << SLCRXLA); // clear RX descriptor address

        pinMode(c_I2sPin, INPUT);
    }
};

template <int DATA_PIN, int T1, int T2, int T3, EOrder RGB_ORDER, int XTRA0, bool FLIP, int WAIT_TIME, bool USE_RGBW, typename T_SPEED>
ClocklessController<DATA_PIN, T1, T2, T3, RGB_ORDER, XTRA0, FLIP, WAIT_TIME, USE_RGBW, T_SPEED>* ClocklessController<DATA_PIN, T1, T2, T3, RGB_ORDER, XTRA0, FLIP, WAIT_TIME, USE_RGBW, T_SPEED>::s_this;

FASTLED_NAMESPACE_END
