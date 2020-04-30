
/*
 * I2S Driver
 *
 *
 */

#pragma once

#pragma message "NOTE: ESP32 support using I2S parallel driver. All strips must use the same chipset"

FASTLED_NAMESPACE_BEGIN

#ifdef __cplusplus
extern "C" {
#endif
    
#include "esp_heap_caps.h"
#include "soc/soc.h"
#include "soc/gpio_sig_map.h"
#include "soc/i2s_reg.h"
#include "soc/i2s_struct.h"
#include "soc/io_mux_reg.h"
#include "driver/gpio.h"
#include "driver/periph_ctrl.h"
#include "rom/lldesc.h"
#include "esp_intr.h"
#include "esp_log.h"
#include <soc/rtc.h>
    
    
#ifdef __cplusplus
}
#endif

__attribute__ ((always_inline)) inline static uint32_t __clock_cycles() {
    uint32_t cyc;
    __asm__ __volatile__ ("rsr %0,ccount":"=a" (cyc));
    return cyc;
}


#define NUM_COLOR_CHANNELS 3

// -- Choose which I2S device to use
#ifndef I2S_DEVICE
#define I2S_DEVICE 0
#endif


// -- Max number of controllers we can support
#ifndef FASTLED_I2S_MAX_CONTROLLERS
#define FASTLED_I2S_MAX_CONTROLLERS 24
#endif

// -- I2S clock
#define I2S_BASE_CLK (80000000L)
#define I2S_MAX_CLK (20000000L) //more tha a certain speed and the I2s looses some bits
#define I2S_MAX_PULSE_PER_BIT 20 //put it higher to get more accuracy but it could decrease the refresh rate without real improvement
// -- Convert ESP32 cycles back into nanoseconds
#define ESPCLKS_TO_NS(_CLKS) (((long)(_CLKS) * 1000L) / F_CPU_MHZ)
#define NUM_VIRT_PINS 7
#ifndef NBIS2SERIALPINS
#define NBIS2SERIALPINS 1
#endif
#ifndef NUM_LEDS_PER_STRIP
#define NUM_LEDS_PER_STRIP 256
#endif
#define OFFSET NUM_VIRT_PINS + 1
#define I2S_OFF (NUM_VIRT_PINS + 1 )* NUM_LEDS_PER_STRIP
#define I2S_OFF2 I2S_OFF * NBIS2SERIALPINS - NUM_LEDS_PER_STRIP
#define AA (0x00AA00AAL)
#define CC (0x0000CCCCL)
#define FF (0xF0F0F0F0L)
#define FF2 (0x0F0F0F0FL)




#ifdef STATIC_COLOR_PER_PIN
#ifdef COLOR_1_RGB
#define C1_R 0
#define C1_G 1
#define C1_B 2
#else
#ifdef COLOR_1_RBG
#define C1_R 0
#define C1_G 2
#define C1_B 1
#else
#ifdef COLOR_1_GBR
#define C1_R 2
#define C1_G 0
#define C1_B 1
#else
#ifdef COLOR_1_BGR
#define C1_R 2
#define C1_G 1
#define C1_B 0
#else
#ifdef COLOR_1_BRG
#define C1_R 1
#define C1_G 2
#define C1_B 0
#else
#define C1_G 0
#define C1_R 1
#define C1_B 2
#endif
#endif
#endif
#endif
#endif
#ifdef COLOR_2_RGB
#define C2_R 0
#define C2_G 1
#define C2_B 2
#else
#ifdef COLOR_2_RBG
#define C2_R 0
#define C2_G 2
#define C2_B 1
#else
#ifdef COLOR_2_GBR
#define C2_R 2
#define C2_G 0
#define C2_B 1
#else
#ifdef COLOR_2_BGR
#define C2_R 2
#define C2_G 1
#define C2_B 0
#else
#ifdef COLOR_2_BRG
#define C2_R 1
#define C2_G 2
#define C2_B 0
#else
#define C2_G 0
#define C2_R 1
#define C2_B 2
#endif
#endif
#endif
#endif
#endif
#ifdef COLOR_3_RGB
#define C3_R 0
#define C3_G 1
#define C3_B 2
#else
#ifdef COLOR_3_RBG
#define C3_R 0
#define C3_G 2
#define C3_B 1
#else
#ifdef COLOR_3_GBR
#define C3_R 2
#define C3_G 0
#define C3_B 1
#else
#ifdef COLOR_3_BGR
#define C3_R 2
#define C3_G 1
#define C3_B 0
#else
#ifdef COLOR_3_BRG
#define C3_R 1
#define C3_G 2
#define C3_B 0
#else
#define C3_G 0
#define C3_R 1
#define C3_B 2
#endif
#endif
#endif
#endif
#endif
#ifdef COLOR_4_RGB
#define C4_R 0
#define C4_G 1
#define C4_B 2
#else
#ifdef COLOR_4_RBG
#define C4_R 0
#define C4_G 2
#define C4_B 1
#else
#ifdef COLOR_4_GBR
#define C4_R 2
#define C4_G 0
#define C4_B 1
#else
#ifdef COLOR_4_BGR
#define C4_R 2
#define C4_G 1
#define C4_B 0
#else
#ifdef COLOR_4_BRG
#define C4_R 1
#define C4_G 2
#define C4_B 0
#else
#define C4_G 0
#define C4_R 1
#define C4_B 2
#endif
#endif
#endif
#endif
#endif
#ifdef COLOR_5_RGB
#define C5_R 0
#define C5_G 1
#define C5_B 2
#else
#ifdef COLOR_5_RBG
#define C5_R 0
#define C5_G 2
#define C5_B 1
#else
#ifdef COLOR_5_GBR
#define C5_R 2
#define C5_G 0
#define C5_B 1
#else
#ifdef COLOR_5_BGR
#define C5_R 2
#define C5_G 1
#define C5_B 0
#else
#ifdef COLOR_5_BRG
#define C5_R 1
#define C5_G 2
#define C5_B 0
#else
#define C5_G 0
#define C5_R 1
#define C5_B 2
#endif
#endif
#endif
#endif
#endif
#ifdef COLOR_6_RGB
#define C6_R 0
#define C6_G 1
#define C6_B 2
#else
#ifdef COLOR_6_RBG
#define C6_R 0
#define C6_G 2
#define C6_B 1
#else
#ifdef COLOR_6_GBR
#define C6_R 2
#define C6_G 0
#define C6_B 1
#else
#ifdef COLOR_6_BGR
#define C6_R 2
#define C6_G 1
#define C6_B 0
#else
#ifdef COLOR_6_BRG
#define C6_R 1
#define C6_G 2
#define C6_B 0
#else
#define C6_G 0
#define C6_R 1
#define C6_B 2
#endif
#endif
#endif
#endif
#endif
#ifdef COLOR_7_RGB
#define C7_R 0
#define C7_G 1
#define C7_B 2
#else
#ifdef COLOR_7_RBG
#define C7_R 0
#define C7_G 2
#define C7_B 1
#else
#ifdef COLOR_7_GBR
#define C7_R 2
#define C7_G 0
#define C7_B 1
#else
#ifdef COLOR_7_BGR
#define C7_R 2
#define C7_G 1
#define C7_B 0
#else
#ifdef COLOR_7_BRG
#define C7_R 1
#define C7_G 2
#define C7_B 0
#else
#define C7_G 0
#define C7_R 1
#define C7_B 2
#endif
#endif
#endif
#endif
#endif
#ifdef COLOR_8_RGB
#define C8_R 0
#define C8_G 1
#define C8_B 2
#else
#ifdef COLOR_8_RBG
#define C8_R 0
#define C8_G 2
#define C8_B 1
#else
#ifdef COLOR_8_GBR
#define C8_R 2
#define C8_G 0
#define C8_B 1
#else
#ifdef COLOR_8_BGR
#define C8_R 2
#define C8_G 1
#define C8_B 0
#else
#ifdef COLOR_8_BRG
#define C8_R 1
#define C8_G 2
#define C8_B 0
#else
#define C8_G 0
#define C8_R 1
#define C8_B 2
#endif
#endif
#endif
#endif
#endif
#ifdef COLOR_9_RGB
#define C9_R 0
#define C9_G 1
#define C9_B 2
#else
#ifdef COLOR_9_RBG
#define C9_R 0
#define C9_G 2
#define C9_B 1
#else
#ifdef COLOR_9_GBR
#define C9_R 2
#define C9_G 0
#define C9_B 1
#else
#ifdef COLOR_9_BGR
#define C9_R 2
#define C9_G 1
#define C9_B 0
#else
#ifdef COLOR_9_BRG
#define C9_R 1
#define C9_G 2
#define C9_B 0
#else
#define C9_G 0
#define C9_R 1
#define C9_B 2
#endif
#endif
#endif
#endif
#endif
#ifdef COLOR_10_RGB
#define C10_R 0
#define C10_G 1
#define C10_B 2
#else
#ifdef COLOR_10_RBG
#define C10_R 0
#define C10_G 2
#define C10_B 1
#else
#ifdef COLOR_10_GBR
#define C10_R 2
#define C10_G 0
#define C10_B 1
#else
#ifdef COLOR_10_BGR
#define C10_R 2
#define C10_G 1
#define C10_B 0
#else
#ifdef COLOR_10_BRG
#define C10_R 1
#define C10_G 2
#define C10_B 0
#else
#define C10_G 0
#define C10_R 1
#define C10_B 2
#endif
#endif
#endif
#endif
#endif
#ifdef COLOR_11_RGB
#define C11_R 0
#define C11_G 1
#define C11_B 2
#else
#ifdef COLOR_11_RBG
#define C11_R 0
#define C11_G 2
#define C11_B 1
#else
#ifdef COLOR_11_GBR
#define C11_R 2
#define C11_G 0
#define C11_B 1
#else
#ifdef COLOR_11_BGR
#define C11_R 2
#define C11_G 1
#define C11_B 0
#else
#ifdef COLOR_11_BRG
#define C11_R 1
#define C11_G 2
#define C11_B 0
#else
#define C11_G 0
#define C11_R 1
#define C11_B 2
#endif
#endif
#endif
#endif
#endif
#ifdef COLOR_12_RGB
#define C12_R 0
#define C12_G 1
#define C12_B 2
#else
#ifdef COLOR_12_RBG
#define C12_R 0
#define C12_G 2
#define C12_B 1
#else
#ifdef COLOR_12_GBR
#define C12_R 2
#define C12_G 0
#define C12_B 1
#else
#ifdef COLOR_12_BGR
#define C12_R 2
#define C12_G 1
#define C12_B 0
#else
#ifdef COLOR_12_BRG
#define C12_R 1
#define C12_G 2
#define C12_B 0
#else
#define C12_G 0
#define C12_R 1
#define C12_B 2
#endif
#endif
#endif
#endif
#endif
#ifdef COLOR_13_RGB
#define C13_R 0
#define C13_G 1
#define C13_B 2
#else
#ifdef COLOR_13_RBG
#define C13_R 0
#define C13_G 2
#define C13_B 1
#else
#ifdef COLOR_13_GBR
#define C13_R 2
#define C13_G 0
#define C13_B 1
#else
#ifdef COLOR_13_BGR
#define C13_R 2
#define C13_G 1
#define C13_B 0
#else
#ifdef COLOR_13_BRG
#define C13_R 1
#define C13_G 2
#define C13_B 0
#else
#define C13_G 0
#define C13_R 1
#define C13_B 2
#endif
#endif
#endif
#endif
#endif
#ifdef COLOR_14_RGB
#define C14_R 0
#define C14_G 1
#define C14_B 2
#else
#ifdef COLOR_14_RBG
#define C14_R 0
#define C14_G 2
#define C14_B 1
#else
#ifdef COLOR_14_GBR
#define C14_R 2
#define C14_G 0
#define C14_B 1
#else
#ifdef COLOR_14_BGR
#define C14_R 2
#define C14_G 1
#define C14_B 0
#else
#ifdef COLOR_14_BRG
#define C14_R 1
#define C14_G 2
#define C14_B 0
#else
#define C14_G 0
#define C14_R 1
#define C14_B 2
#endif
#endif
#endif
#endif
#endif
#ifdef COLOR_15_RGB
#define C15_R 0
#define C15_G 1
#define C15_B 2
#else
#ifdef COLOR_15_RBG
#define C15_R 0
#define C15_G 2
#define C15_B 1
#else
#ifdef COLOR_15_GBR
#define C15_R 2
#define C15_G 0
#define C15_B 1
#else
#ifdef COLOR_15_BGR
#define C15_R 2
#define C15_G 1
#define C15_B 0
#else
#ifdef COLOR_15_BRG
#define C15_R 1
#define C15_G 2
#define C15_B 0
#else
#define C15_G 0
#define C15_R 1
#define C15_B 2
#endif
#endif
#endif
#endif
#endif
#else
#ifdef STATIC_COLOR_RGB
#define C_R 0
#define C_G 1
#define C_B 2
#else
#ifdef STATIC_COLOR_RBG
#define C_R 0
#define C_G 2
#define C_B 1
#else
#ifdef STATIC_COLOR_GBR
#define C_R 2
#define C_G 0
#define C_B 1
#else
#ifdef STATIC_COLOR_BGR
#define C_R 2
#define C_G 1
#define C_B 0
#else
#ifdef STATIC_COLOR_BRG
#define C_R 1
#define C_G 2
#define C_B 0
#else
#ifdef STATIC_COLOR_GRB
#define C_G 0
#define C_R 1
#define C_B 2

#else
static byte C_G,C_R,C_B;

#endif
#endif
#endif
#endif
#endif
#endif
#endif


// -- Array of all controllers
//static CLEDController * gControllers[FASTLED_I2S_MAX_CONTROLLERS];
static int gNumControllers = 0;
static int gNumStarted = 0;

// -- Global semaphore for the whole show process
//    Semaphore is not given until all data has been sent
static xSemaphoreHandle gTX_sem = NULL;

// -- One-time I2S initialization
static bool gInitialized = false;

// -- Interrupt handler
static intr_handle_t gI2S_intr_handle = NULL;

// -- A pointer to the memory-mapped structure: I2S0 or I2S1
static i2s_dev_t * i2s;

// -- I2S goes to these pins until we remap them using the GPIO matrix
static int i2s_base_pin_index;

// --- I2S DMA buffers
struct DMABuffer {
    lldesc_t descriptor;
    uint8_t * buffer;
};

#define NUM_DMA_BUFFERS 4
static DMABuffer * dmaBuffers[NUM_DMA_BUFFERS];

// -- Bit patterns
//    For now, we require all strips to be the same chipset, so these
//    are global variables.

static int      gPulsesPerBit = 0;
//static uint32_t gOneBit[40] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//static uint32_t gZeroBit[40]  = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

// -- Counters to track progress
static int gCurBuffer = 0;
static bool gDoneFilling = false;
static int ones_for_one;
static int ones_for_zero;

// -- Temp buffers for pixels and bits being formatted for DMA
static uint8_t gPixelRow[NUM_COLOR_CHANNELS][32];
static uint8_t gPixelBits[NUM_COLOR_CHANNELS][8][4];
static int CLOCK_DIVIDER_N;
static int CLOCK_DIVIDER_A;
static int CLOCK_DIVIDER_B;
static int dmaBufferActive;
static volatile long countfps;
static volatile int nbp;
static volatile bool stopSignal;
static volatile bool runningPixel=false;

//CRGB  pixelg[8][5]; //volatile uint8_t pixelg[8][5];
//uint8_t pixelg[16][8][4] ;
//volatile uint8_t pixelr[8][5];
//volatile uint8_t pixelb[8][5];
static volatile  int ledToDisplay;
//CRGB *int_leds;
static  int dmaBufferCount=2; //we use two buffers
/* typedef union {
 uint8_t bytes[16];
 uint16_t shorts[8];
 uint32_t raw[2];
 } Lines;*/

typedef union {
    uint8_t bytes[16];
    uint32_t shorts[8];
    uint32_t raw[2];
} Lines;
static volatile  int num_strips;
static volatile  int nun_led_per_strip;
// int *Pins;
static  int brightness_b;
static  int brightness_r;
static  int brightness_g;
static uint8_t green_map[256];
static uint8_t blue_map[256];
static uint8_t red_map[256];

static int ledType;
static volatile CRGB *int_leds;
static CRGB m_scale;
//static Lines firstPixel[3];
//template <int DATA_PIN, int T1, int T2, int T3, EOrder RGB_ORDER = RGB, int XTRA0 = 0, bool FLIP = false, int WAIT_TIME = 5>
//<Pins,CLOCK_PIN,LATCH_PIN, GRB>
template<int *Pins,int CLOCK_PIN,int LATCH_PIN, EOrder RGB_ORDER = GRB>
class ClocklessController : public CPixelLEDController<RGB_ORDER>
{
    
    //int *Pins;
    const int deviceBaseIndex[2] = {I2S0O_DATA_OUT0_IDX, I2S1O_DATA_OUT0_IDX};
    const int deviceClockIndex[2] = {I2S0O_BCK_OUT_IDX, I2S1O_BCK_OUT_IDX};
    const int deviceWordSelectIndex[2] = {I2S0O_WS_OUT_IDX, I2S1O_WS_OUT_IDX};
    const periph_module_t deviceModule[2] = {PERIPH_I2S0_MODULE, PERIPH_I2S1_MODULE};
public:
    
    void init()
    {
        // brigthness=10;
        
        //Serial.printf("%d %d %d\n",CLOCK_PIN,LATCH_PIN,NUM_LED_PER_STRIP);
        
        
        for (int i = 0; i < NBIS2SERIALPINS; i++)
            if (Pins[i] > -1)
            {
                PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[Pins[i]], PIN_FUNC_GPIO);
                gpio_set_direction((gpio_num_t)Pins[i], (gpio_mode_t)GPIO_MODE_DEF_OUTPUT);
                pinMode(Pins[i],OUTPUT);
                gpio_matrix_out(Pins[i], deviceBaseIndex[I2S_DEVICE] + i+8, false, false);
            }
        
        //latch pin
        PIN_FUNC_SELECT(GPIO_PIN_MUX_REG[LATCH_PIN], PIN_FUNC_GPIO);
        gpio_set_direction((gpio_num_t)LATCH_PIN, (gpio_mode_t)GPIO_MODE_DEF_OUTPUT);
        pinMode(LATCH_PIN,OUTPUT);
        gpio_matrix_out(LATCH_PIN, deviceBaseIndex[I2S_DEVICE] + 23, false, false);
        //if (baseClock > -1)
        //clock pin
        gpio_matrix_out(CLOCK_PIN, deviceClockIndex[I2S_DEVICE], false, false);

#ifndef STATIC_COLOR_PER_PIN
#ifndef STATIC_COLOR_GRB
#ifndef STATIC_COLOR_RGB
#ifndef STATIC_COLOR_RBG
#ifndef STATIC_COLOR_GBR
#ifndef STATIC_COLOR_BRG
#ifndef STATIC_COLOR_BGR
        C_R=(byte)(RGB_ORDER/64);
        C_G=(byte)((RGB_ORDER-C_R*64)/8);
        C_B=(byte)(RGB_ORDER-C_R*64-C_G*8);
#endif
#endif
#endif
#endif
#endif
#endif
#endif
        
        
        // gpio_matrix_out(26, deviceWordSelectIndex[I2S_DEVICE], false, false);
        i2sInit();
        
    }
    
    virtual uint16_t getMaxRefreshRate() const { return 800; }
    
protected:
    
    
    
    static DMABuffer * allocateDMABuffer(int bytes)
    {
        DMABuffer * b = (DMABuffer *)heap_caps_malloc(sizeof(DMABuffer), MALLOC_CAP_DMA);
        
        b->buffer = (uint8_t *)heap_caps_malloc(bytes, MALLOC_CAP_DMA);
        memset(b->buffer, 0, bytes);
        
        b->descriptor.length = bytes;
        b->descriptor.size = bytes;
        b->descriptor.owner = 1;
        b->descriptor.sosf = 1;
        b->descriptor.buf = b->buffer;
        b->descriptor.offset = 0;
        b->descriptor.empty = 0;
        b->descriptor.eof = 1;
        b->descriptor.qe.stqe_next = 0;
        
        return b;
    }
    
    static void i2sInit()
    {
        // -- Only need to do this once
        // if (gInitialized) return;
        
        // -- Construct the bit patterns for ones and zeros
        //   initBitPatterns();
        
        // -- Choose whether to use I2S device 0 or device 1
        //    Set up the various device-specific parameters
        int interruptSource;
        if (I2S_DEVICE == 0) {
            i2s = &I2S0;
            periph_module_enable(PERIPH_I2S0_MODULE);
            interruptSource = ETS_I2S0_INTR_SOURCE;
            i2s_base_pin_index = I2S0O_DATA_OUT0_IDX;
        } else {
            i2s = &I2S1;
            periph_module_enable(PERIPH_I2S1_MODULE);
            interruptSource = ETS_I2S1_INTR_SOURCE;
            i2s_base_pin_index = I2S1O_DATA_OUT0_IDX;
        }
        
        // -- Reset everything
        i2sReset();
        i2sReset_DMA();
        i2sReset_FIFO();
        
        // -- Main configuration
        // i2s->conf.tx_msb_right = 1;
        //i2s->conf.tx_mono = 0;
        //i2s->conf.tx_short_sync = 0;
        //i2s->conf.tx_msb_shift = 0;
        i2s->conf.tx_right_first = 0; // 0;//1;
        //i2s->conf.tx_slave_mod = 0;
        
        // -- Set parallel mode
        i2s->conf2.val = 0;
        i2s->conf2.lcd_en = 1;
        i2s->conf2.lcd_tx_wrx2_en = 1; // 0 for 16 or 32 parallel output
        i2s->conf2.lcd_tx_sdx2_en = 0; // HN
        
        // -- Set up the clock rate and sampling
        i2s->sample_rate_conf.val = 0;
        i2s->sample_rate_conf.tx_bits_mod = 16; // Number of parallel bits/pins
        i2s->sample_rate_conf.tx_bck_div_num = 1;
        i2s->clkm_conf.val = 0;
#ifdef DL_CLK
        Serial.println("norml clock");
                i2s->clkm_conf.clka_en = 0;
                //rtc_clk_apll_enable(true, 31, 133,7, 1); //19.2Mhz 7 pins +1 latchrtc_clk_apll_enable(true, 31, 133,7, 1); //19.2Mhz 7 pins +1 latch
                
                // -- Data clock is computed as Base/(div_num + (div_b/div_a))
                //    Base is 80Mhz, so 80/(3+ 7/6) = 19.2Mhz
                //   
                i2s->clkm_conf.clkm_div_a =6;// CLOCK_DIVIDER_A;
                i2s->clkm_conf.clkm_div_b = 7;//CLOCK_DIVIDER_B;
                i2s->clkm_conf.clkm_div_num = 3;//CLOCK_DIVIDER_N;
        
#else
         Serial.println("precise clock");
        i2s->clkm_conf.clka_en = 1;
        
        //rtc_clk_apll_enable(true, 215, 163,1, 20);
        //rtc_clk_apll_enable(true, 215, 163,4, 1); //14.4Mhz 5pins +1 latch
        //rtc_clk_apll_enable(true, 123, 20,6, 1); //16.8Mhz 6 pins +1 latchtch
        //rtc_clk_apll_enable(true, 164, 112,9, 2); //16.8Mhz 6 pins +1 latchtch
        rtc_clk_apll_enable(true, 31, 133,7, 1); //19.2Mhz 7 pins +1 latchrtc_clk_apll_enable(true, 31, 133,7, 1); //19.2Mhz 7 pins +1 latch
        
        // -- Data clock is computed as Base/(div_num + (div_b/div_a))
        //    Base is 80Mhz, so 80/(10 + 0/1) = 8Mhz
        //    One cycle is 125ns
        i2s->clkm_conf.clkm_div_a =1;// CLOCK_DIVIDER_A;
        i2s->clkm_conf.clkm_div_b = 0;//CLOCK_DIVIDER_B;
        i2s->clkm_conf.clkm_div_num = 1;//CLOCK_DIVIDER_N;
#endif
        i2s->fifo_conf.val = 0;
        i2s->fifo_conf.tx_fifo_mod_force_en = 1;
        i2s->fifo_conf.tx_fifo_mod = 1;  // 16-bit single channel data
        i2s->fifo_conf.tx_data_num = 32;//32; // fifo length
        i2s->fifo_conf.dscr_en = 1;      // fifo will use dma
        
        i2s->conf1.val = 0;
        i2s->conf1.tx_stop_en = 0;
        i2s->conf1.tx_pcm_bypass = 1;
        
        i2s->conf_chan.val = 0;
        i2s->conf_chan.tx_chan_mod = 1; // Mono mode, with tx_msb_right = 1, everything goes to right-channel
        
        i2s->timing.val = 0;
        
        // -- Allocate two DMA buffers
        dmaBuffers[0] = allocateDMABuffer((NUM_VIRT_PINS+1)*3*8*3*2);
        dmaBuffers[1] = allocateDMABuffer((NUM_VIRT_PINS+1)*3*8*3*2);
        dmaBuffers[2] = allocateDMABuffer((NUM_VIRT_PINS+1)*3*8*3*2);
        
        // -- Arrange them as a circularly linked list
        dmaBuffers[0]->descriptor.qe.stqe_next = &(dmaBuffers[1]->descriptor);
        dmaBuffers[1]->descriptor.qe.stqe_next = &(dmaBuffers[0]->descriptor);
        pu((uint16_t*)dmaBuffers[0]->buffer); //latch
        pu((uint16_t*)dmaBuffers[1]->buffer);
        //pu((uint32_t*)this->dmaBuffers[2]->buffer);
        pu2((uint16_t*)dmaBuffers[0]->buffer); //first pulse
        pu2((uint16_t*)dmaBuffers[1]->buffer);
        
        // -- Allocate i2s interrupt
        SET_PERI_REG_BITS(I2S_INT_ENA_REG(I2S_DEVICE), I2S_OUT_EOF_INT_ENA_V, 1, I2S_OUT_EOF_INT_ENA_S);
        esp_err_t e = esp_intr_alloc(interruptSource, ESP_INTR_FLAG_INTRDISABLED  , // ESP_INTR_FLAG_INTRDISABLED | ESP_INTR_FLAG_LEVEL3,
                                     &interruptHandler, 0, &gI2S_intr_handle);
        
        // -- Create a semaphore to block execution until all the controllers are done
        /* if (gTX_sem == NULL) {
         gTX_sem = xSemaphoreCreateBinary();
         xSemaphoreGive(gTX_sem);
         }
         */
        // Serial.println("Init I2S");
        gInitialized = true;
    }
    
    
    static void pu(uint16_t* buff)
    {
        memset((uint8_t*)buff,0,(NUM_VIRT_PINS+1)*8*3*3*2);
        for (int i=0;i<24*3;i++)
        {
            buff[NUM_VIRT_PINS+i*(NUM_VIRT_PINS+1)-1-5]=0x8000;
            //buff[NUM_VIRT_PINS+i*(NUM_VIRT_PINS+1)]=0x02;
        }
    }
    static void pu3(uint16_t* buff)
    {
        // memset((uint8_t*)buff,0,(NUM_VIRT_PINS+1)*8*3*3*2);
        for (int i=0;i<24*3;i++)
        {
            buff[NUM_VIRT_PINS+i*(NUM_VIRT_PINS+1)-1-5]+=0x8000;
            //buff[NUM_VIRT_PINS+i*(NUM_VIRT_PINS+1)]=0x02;
        }
    }
    
    static  void pu2(uint16_t* buff)
    {
        
        
        for (int j=0;j<24;j++)
        {
            // for (int i=0;i<NUM_VIRT_PINS;i++)
            // {
            //*buff=0x7FFF;
            // buff++;
            buff[1+j*(3*(NUM_VIRT_PINS+1))]=0xFFFF;
            buff[0+j*(3*(NUM_VIRT_PINS+1))]=0x7FFF;
            buff[3+j*(3*(NUM_VIRT_PINS+1))]=0x7FFF;
            buff[2+j*(3*(NUM_VIRT_PINS+1))]=0x7FFF;
            buff[5+j*(3*(NUM_VIRT_PINS+1))]=0x7FFF;
            buff[4+j*(3*(NUM_VIRT_PINS+1))]=0x7FFF;
            buff[7+j*(3*(NUM_VIRT_PINS+1))]=0x7FFF;
            buff[6+j*(3*(NUM_VIRT_PINS+1))]=0x7FFF;
            //buff[NUM_VIRT_PINS-2+j*(3*(NUM_VIRT_PINS+1))]=0x00FF;
            //buff[NUM_VIRT_PINS-3+j*(3*(NUM_VIRT_PINS+1))]=0x00FF;
            // buff[NUM_VIRT_PINS-4+j*(3*(NUM_VIRT_PINS+1))]=0x00FF;
            //buff[NUM_VIRT_PINS-5+j*(3*(NUM_VIRT_PINS+1))]=0x00FF;
            //buff[NUM_VIRT_PINS-6+j*(3*(NUM_VIRT_PINS+1))]=0x00FF;
            //buff[NUM_VIRT_PINS-4+j*(3*(NUM_VIRT_PINS+1))]=0x00FF;
            // }
            //buff+=3*(NUM_VIRT_PINS+1)-NUM_VIRT_PINS; //13
        }
    }
    
    /** Clear DMA buffer
     *
     *  Yves' clever trick: initialize the bits that we know must be 0
     *  or 1 regardless of what bit they encode.
     */
    static void empty( uint32_t *buf)
    {
        for(int i=0;i<8*NUM_COLOR_CHANNELS;i++)
        {
            int offset=gPulsesPerBit*i;
            for(int j=0;j<ones_for_zero;j++)
                buf[offset+j]=0xffffffff;
            
            for(int j=ones_for_one;j<gPulsesPerBit;j++)
                buf[offset+j]=0;
        }
    }
    
    // -- Show pixels
    //    This is the main entry point for the controller.
    virtual void showPixels(PixelController<RGB_ORDER> & pixels)
    {
        
        //Serial.println("Show");
        nbp=0;
        countfps=0;
        int_leds=(CRGB*)pixels.mData;
        m_scale=pixels.mScale;
        nun_led_per_strip=  pixels.mLen;
        //Serial.printf("led - r:%d v:%d b%d\n",m_scale.r,m_scale.g,m_scale.b);
        /*
         brightness_b=256/(m_scale.b+1);
         brightness_r=256/(m_scale.r+1);
         brightness_g=256/(m_scale.g+1);
         */
        
        for(int i=0;i<256;i++)
        {
            green_map[i]=(uint8_t)((int)(i*m_scale.g)/255);
            blue_map[i]=(uint8_t)((int)(i*m_scale.b)/255);
            red_map[i]=(uint8_t)((int)(i*m_scale.r)/255);
            // Serial.printf("led -  i:%d scale:%d r:%d v:%d b:%d\n",i,m_scale.r,green_map[i],red_map[i],blue_map[i]);
        }
        //for(int j=0;j<50;j++)
        //{
        //Serial.printf("led n:%d r:%d v:%d b%d\n",j,int_leds[j].r,int_leds[j].g,int_leds[j].b);
        //    }
        ledToDisplay=0;
        stopSignal=false;
        
        pu((uint16_t*)dmaBuffers[0]->buffer); //latch
        pu((uint16_t*)dmaBuffers[1]->buffer);
        pu((uint16_t*)dmaBuffers[2]->buffer);
        //pu((uint32_t*)this->dmaBuffers[3]->buffer);
        pu2((uint16_t*)dmaBuffers[0]->buffer); //first pulse
        pu2((uint16_t*)dmaBuffers[1]->buffer);
        //pu2((uint32_t*)this->dmaBuffers[2]->buffer);
        fillbuffer6((uint16_t*)dmaBuffers[0]->buffer);
        ledToDisplay++;
        //fillbuffer2((uint32_t*)dmaBuffers[1]->buffer);
        //ledToDisplay++;
        
        dmaBufferActive=1;
        // this->dmaBuffers[3]->next(this->dmaBuffers[0]);
        /*
         dmaBuffers[1]->next(this->dmaBuffers[0]);
         dmaBuffers[2]->next(this->dmaBuffers[0]);
         dmaBuffers[0]->next(this->dmaBuffers[1]); //on utilise le dernier buffer*/
        dmaBuffers[1]->descriptor.qe.stqe_next = &(dmaBuffers[0]->descriptor);
        dmaBuffers[2]->descriptor.qe.stqe_next = &(dmaBuffers[0]->descriptor);
        dmaBuffers[0]->descriptor.qe.stqe_next = &(dmaBuffers[1]->descriptor);
        runningPixel=true;
        //startTX();
        i2sStart();
        //long time3=ESP.getCycleCount();
        while(runningPixel==true);
        //time3=ESP.getCycleCount()-time3;
        // Serial.printf("fps:%f\n",(float)240000000L/time3);
        delayMicroseconds(50);
        
    }
    
    // -- Custom interrupt handler
    static IRAM_ATTR void interruptHandler(void *arg)
    {
        
        /*
         if (i2s->int_st.out_eof) {
         i2s->int_clr.val = i2s->int_raw.val;
         
         if ( ! gDoneFilling) {
         fillBuffer6();
         } else {
         portBASE_TYPE HPTaskAwoken = 0;
         xSemaphoreGiveFromISR(gTX_sem, &HPTaskAwoken);
         if(HPTaskAwoken == pdTRUE) portYIELD_FROM_ISR();
         }
         }*/
        
        //Lines pixel[3];
        if (!i2s->int_st.out_eof)
            return;
        i2s->int_clr.val = i2s->int_raw.val;
        
        
        if(stopSignal)
        {
            // Serial.println("stop");
            i2sStop();
            runningPixel=false;
            return;
        }
        if(ledToDisplay<=NUM_LEDS_PER_STRIP)
        {
            
            
            
            
            
            
            if(ledToDisplay==NUM_LEDS_PER_STRIP)
            {
                pu( (uint16_t*)dmaBuffers[dmaBufferActive]->buffer);
                
                
                /*i2sStop();
                 runningPixel=false;
                 return;*/
                stopSignal=true;
            }
            //  long time3=ESP.getCycleCount();
            fillbuffer6((uint16_t*)dmaBuffers[dmaBufferActive]->buffer);
            //   time3=ESP.getCycleCount()-time3;
            //   if(countfps<time3)
            //      countfps=time3;
            //  nbp++;
            ledToDisplay++;
            dmaBufferActive = (dmaBufferActive + 1)% 2;
            //if(ledToDisplay)
        }
        else
        {
            //if no more pixels then we will read the other buffer and stop
            // if(ledToDisplay==nun_led_per_strip)
            //   ledToDisplay++;
            //if(ledToDisplay==nun_led_per_strip+1)
            stopSignal=true;
        }
    }
    
    
    
    static    void transpose16x1_noinline2(unsigned char *A, uint8_t *B) {
        uint32_t  x, y, x1,y1,t;
        
        
        
        y = *(unsigned int*)(A);
        x = *(unsigned int*)(A+4);
        y1 = *(unsigned int*)(A+8);
        //x1=0;
        x1 = *(unsigned int*)(A+12);
        
        
        
        
        // pre-transform x
        t = (x ^ (x >> 7)) & AA;  x = x ^ t ^ (t << 7);
        t = (x ^ (x >>14)) & CC;  x = x ^ t ^ (t <<14);
        t = (x1 ^ (x1 >> 7)) & AA;  x1 = x1 ^ t ^ (t << 7);
        t = (x1 ^ (x1 >>14)) & CC;  x1 = x1 ^ t ^ (t <<14);
        // pre-transform y
        t = (y ^ (y >> 7)) & AA;  y = y ^ t ^ (t << 7);
        t = (y ^ (y >>14)) & CC;  y = y ^ t ^ (t <<14);
        t = (y1 ^ (y1 >> 7)) & AA;  y1 = y1 ^ t ^ (t << 7);
        t = (y1 ^ (y1 >>14)) & CC;  y1 = y1 ^ t ^ (t <<14);
        
        
        // final transform
        t = (x & FF) | ((y >> 4) & FF2);
        y = ((x << 4) & FF) | (y & FF2);
        x = t;
        
        t= (x1 & FF) | ((y1 >> 4) & FF2);
        y1 = ((x1 << 4) & FF) | (y1 & FF2);
        x1 = t;
        
        
        /*
         *((uint16_t*)B) = (uint16_t)((y & 0xff) |  (  (y1 & 0xff) << 8 ) )   ;
         B-=offset;
         //B-=offset;
         *((uint16_t*)(B)) = (uint16_t)(((y & 0xff00) |((y1&0xff00) <<8))>>8);
         B-=offset;
         *((uint16_t*)(B)) = (uint16_t)(((y & 0xff0000) |((y1&0xff0000) <<8))>>16);
         B-=offset;
         *((uint16_t*)(B)) = (uint16_t)(((y & 0xff000000) >>8 |((y1&0xff000000) ))>>16);
         B-=offset;
         *((uint16_t*)B) =(uint16_t)( (x & 0xff) |((x1&0xff) <<8));
         B-=offset;
         *((uint16_t*)(B)) = (uint16_t)(((x & 0xff00) |((x1&0xff00) <<8))>>8);
         B-=offset;
         *((uint16_t*)(B)) = (uint16_t)(((x & 0xff0000) |((x1&0xff0000) <<8))>>16);
         B-=offset;
         *((uint16_t*)(B)) = (uint16_t)(((x & 0xff000000) >>8 |((x1&0xff000000) ))>>16);
         */
        
        // *((uint16_t*)(B)) = (uint16_t)(   (   (x & 0xff000000) >>24 |  (  (x1 & 0xff000000) >>16)   )  );
        // *((uint16_t*)(B+4*48)) = (uint16_t)(((x & 0xff000000) >>8 |((x1&0xff000000) ))>>16);
        *((uint16_t*)(B)) = (uint16_t)(((x & 0xff000000) >>8 |((x1&0xff000000) ))>>16);
        //*((uint8_t*)(B))=*((uint8_t*)(&x)+3);
        //*((uint8_t*)(B+1))=*((uint8_t*)(&x1)+2);
        
        
        
        // B[0]= (uint16_t)(   (   (x & 0xff000000) >>24 |  (  (x1&0xff000000) >>16)   )  );
        //B+=48;//offset;
        *((uint16_t*)(B+48)) = (uint16_t)( ((x & 0xff0000) >>16|((x1&0xff0000) >>8)));
        // B[48] = (uint16_t)( ((x & 0xff0000) >>16|((x1&0xff0000) >>8)));
        //+=48;//offset;
        *((uint16_t*)(B+2*48)) = (uint16_t)(((x & 0xff00) |((x1&0xff00) <<8))>>8);
        // B[2*48]  = (uint16_t)(((x & 0xff00) |((x1&0xff00) <<8))>>8);
        // B+=48;//offset;
        *((uint16_t*)(B+3*48)) =(uint16_t)( (x & 0xff) |((x1&0xff) <<8));
        // B[3*48] =(uint16_t)( (x & 0xff) |((x1&0xff) <<8));
        //B+=48;//offset;
        
        
        *((uint16_t*)(B+4*48)) = (uint16_t)(((y & 0xff000000) >>8 |((y1&0xff000000) ))>>16);
        // B[4*48]= (uint16_t)(((y & 0xff000000) >>8 |((y1&0xff000000) ))>>16);
        
        *((uint16_t*)(B+5*48)) = (uint16_t)(((y & 0xff0000) |((y1&0xff0000) <<8))>>16);
        
        
        *((uint16_t*)(B+6*48)) = (uint16_t)(((y & 0xff00) |((y1&0xff00) <<8))>>8);
        //  *((uint16_t*)(B+6*48)) = (uint16_t)(( ((y & 0xff00)>>8) |((y1&0xff00) )));
        
        // B[6*48]=    (uint16_t)(((y & 0xff00) |((y1&0xff00) <<8))>>8);
        
        
        *((uint16_t*)(B+7*48)) = (uint16_t)((y & 0xff) |  (  (y1 & 0xff) << 8 ) )   ;
        //  B[7*48]  = (uint16_t)((y & 0xff) |  (  (y1 & 0xff) << 8 ) )   ;
        
    }
    
    
    
    static void fillbuffer6(uint16_t *buff)
    {
        //return;
        //return;
        //uint16_t *g;
        //  g=buff;
        Lines firstPixel[3];
        
        volatile CRGB * poli;
        // Lines secondPixel[3];
        //    int nblines=5;
        
        //  int nbpins=20;//    this->nbpins;
        
        
        // uint32_t l2=ledToDisplay;
        // poli=int_leds+ledToDisplay;
        //Serial.println(ledToDisplay);
        // uint32_t offset=OFFSET;//(NUM_VIRT_PINS+1)+1-1;//(7)*(NUM_VIRT_PINS+1)*3+2*NUM_VIRT_PINS+1;
        buff+=OFFSET;
        poli=int_leds+ledToDisplay;
        //uint32_t off=nun_led_per_strip*NUM_VIRT_PINS;
#ifndef STATIC_COLOR_PER_PIN
        for(int pin=0;pin<NBIS2SERIALPINS;pin++) {
            
            //uint32_t l=ledToDisplay+nun_led_per_strip*line+pin*nun_led_per_strip*5;
            
            
            firstPixel[C_G].bytes[pin] = green_map[(*poli).g];//(*poli).g/brightness_g; //scale8(int_leds[l].g,brightness_g);
            firstPixel[C_R].bytes[pin] = red_map[(*poli).r];//(*poli).r/brightness_r;
            firstPixel[C_B].bytes[pin] =blue_map[(*poli).b];//(*poli).b/brightness_b;
            //l+=nun_led_per_strip*NUM_VIRT_PINS;
            poli+=I2S_OFF;
            
            
        }
#else
#if NBIS2SERIALPINS >=1
        firstPixel[C1_G].bytes[0] = green_map[(*poli).g];
        firstPixel[C1_R].bytes[0] = red_map[(*poli).r];
        firstPixel[C1_B].bytes[0] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        
#if NBIS2SERIALPINS >=2
        firstPixel[C2_G].bytes[1] = green_map[(*poli).g];
        firstPixel[C2_R].bytes[1] = red_map[(*poli).r];
        firstPixel[C2_B].bytes[1] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        
#if NBIS2SERIALPINS >=3
        firstPixel[C3_G].bytes[2] = green_map[(*poli).g];
        firstPixel[C3_R].bytes[2] = red_map[(*poli).r];
        firstPixel[C3_B].bytes[2] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        
#if NBIS2SERIALPINS >=4
        firstPixel[C4_G].bytes[3] = green_map[(*poli).g];
        firstPixel[C4_R].bytes[3] = red_map[(*poli).r];
        firstPixel[C4_B].bytes[3] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=5
        firstPixel[C5_G].bytes[4] = green_map[(*poli).g];
        firstPixel[C5_R].bytes[4] = red_map[(*poli).r];
        firstPixel[C5_B].bytes[4] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=6
        firstPixel[C6_G].bytes[5] = green_map[(*poli).g];
        firstPixel[C6_R].bytes[5] = red_map[(*poli).r];
        firstPixel[C6_B].bytes[5] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        //  poli+=I2S_OFF;
#if NBIS2SERIALPINS >=7
        firstPixel[C7_G].bytes[6] = green_map[(*poli).g];
        firstPixel[C7_R].bytes[6] = red_map[(*poli).r];
        firstPixel[C7_B].bytes[6] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=8
        firstPixel[C8_G].bytes[7] = green_map[(*poli).g];
        firstPixel[C8_R].bytes[7] = red_map[(*poli).r];
        firstPixel[C8_B].bytes[7] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        //  poli+=I2S_OFF;
#if NBIS2SERIALPINS >=9
        firstPixel[C9_G].bytes[8] = green_map[(*poli).g];
        firstPixel[C9_R].bytes[8] = red_map[(*poli).r];
        firstPixel[C9_B].bytes[8] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=10
        firstPixel[C10_G].bytes[9] = green_map[(*poli).g];
        firstPixel[C10_R].bytes[9] = red_map[(*poli).r];
        firstPixel[C10_B].bytes[9] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=11
        firstPixel[C11_G].bytes[10] = green_map[(*poli).g];
        firstPixel[C11_R].bytes[10] = red_map[(*poli).r];
        firstPixel[C11_B].bytes[10] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=12
        firstPixel[C12_G].bytes[11] = green_map[(*poli).g];
        firstPixel[C12_R].bytes[11] = red_map[(*poli).r];
        firstPixel[C12_B].bytes[11] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        //poli+=I2S_OFF;
#if NBIS2SERIALPINS >=13
        firstPixel[C13_G].bytes[12] = green_map[(*poli).g];
        firstPixel[C13_R].bytes[12] = red_map[(*poli).r];
        firstPixel[C13_B].bytes[12] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=14
        firstPixel[C14_G].bytes[13] = green_map[(*poli).g];
        firstPixel[C14_R].bytes[13] = red_map[(*poli).r];
        firstPixel[C14_B].bytes[13] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=15
        firstPixel[C15_G].bytes[14] = green_map[(*poli).g];
        firstPixel[C15_R].bytes[14] = red_map[(*poli).r];
        firstPixel[C15_B].bytes[14] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
#endif
        //l2+=nun_led_per_strip;
        
        //firstPixel[0].bytes[15]=255;
        //firstPixel[1].bytes[15]=255;
        //firstPixel[2].bytes[15]=255;
        firstPixel[0].bytes[15]=0;
        firstPixel[1].bytes[15]=0;
        firstPixel[2].bytes[15]=0;
        transpose16x1_noinline2(firstPixel[0].bytes,(uint8_t*)(buff));
        transpose16x1_noinline2(firstPixel[1].bytes,(uint8_t*)(buff+192));
        transpose16x1_noinline2(firstPixel[2].bytes,(uint8_t*)(buff+384));
        //l2+=NUM_LEDS_PER_STRIP;
        poli-=I2S_OFF2;
        
        
        
        buff++;
#ifndef STATIC_COLOR_PER_PIN
        for(int pin=0;pin<NBIS2SERIALPINS;pin++) {
            
            //uint32_t l=ledToDisplay+nun_led_per_strip*line+pin*nun_led_per_strip*5;
            
            
            firstPixel[C_G].bytes[pin] = green_map[(*poli).g];//(*poli).g/brightness_g; //scale8(int_leds[l].g,brightness_g);
            firstPixel[C_R].bytes[pin] = red_map[(*poli).r];//(*poli).r/brightness_r;
            firstPixel[C_B].bytes[pin] =blue_map[(*poli).b];//(*poli).b/brightness_b;
            //l+=nun_led_per_strip*NUM_VIRT_PINS;
            poli+=I2S_OFF;
            
            
        }
#else
#if NBIS2SERIALPINS >=1
        firstPixel[C1_G].bytes[0] = green_map[(*poli).g];
        firstPixel[C1_R].bytes[0] = red_map[(*poli).r];
        firstPixel[C1_B].bytes[0] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        
#if NBIS2SERIALPINS >=2
        firstPixel[C2_G].bytes[1] = green_map[(*poli).g];
        firstPixel[C2_R].bytes[1] = red_map[(*poli).r];
        firstPixel[C2_B].bytes[1] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        
#if NBIS2SERIALPINS >=3
        firstPixel[C3_G].bytes[2] = green_map[(*poli).g];
        firstPixel[C3_R].bytes[2] = red_map[(*poli).r];
        firstPixel[C3_B].bytes[2] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        
#if NBIS2SERIALPINS >=4
        firstPixel[C4_G].bytes[3] = green_map[(*poli).g];
        firstPixel[C4_R].bytes[3] = red_map[(*poli).r];
        firstPixel[C4_B].bytes[3] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=5
        firstPixel[C5_G].bytes[4] = green_map[(*poli).g];
        firstPixel[C5_R].bytes[4] = red_map[(*poli).r];
        firstPixel[C5_B].bytes[4] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=6
        firstPixel[C6_G].bytes[5] = green_map[(*poli).g];
        firstPixel[C6_R].bytes[5] = red_map[(*poli).r];
        firstPixel[C6_B].bytes[5] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        //  poli+=I2S_OFF;
#if NBIS2SERIALPINS >=7
        firstPixel[C7_G].bytes[6] = green_map[(*poli).g];
        firstPixel[C7_R].bytes[6] = red_map[(*poli).r];
        firstPixel[C7_B].bytes[6] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=8
        firstPixel[C8_G].bytes[7] = green_map[(*poli).g];
        firstPixel[C8_R].bytes[7] = red_map[(*poli).r];
        firstPixel[C8_B].bytes[7] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        //  poli+=I2S_OFF;
#if NBIS2SERIALPINS >=9
        firstPixel[C9_G].bytes[8] = green_map[(*poli).g];
        firstPixel[C9_R].bytes[8] = red_map[(*poli).r];
        firstPixel[C9_B].bytes[8] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=10
        firstPixel[C10_G].bytes[9] = green_map[(*poli).g];
        firstPixel[C10_R].bytes[9] = red_map[(*poli).r];
        firstPixel[C10_B].bytes[9] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=11
        firstPixel[C11_G].bytes[10] = green_map[(*poli).g];
        firstPixel[C11_R].bytes[10] = red_map[(*poli).r];
        firstPixel[C11_B].bytes[10] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=12
        firstPixel[C12_G].bytes[11] = green_map[(*poli).g];
        firstPixel[C12_R].bytes[11] = red_map[(*poli).r];
        firstPixel[C12_B].bytes[11] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        //poli+=I2S_OFF;
#if NBIS2SERIALPINS >=13
        firstPixel[C13_G].bytes[12] = green_map[(*poli).g];
        firstPixel[C13_R].bytes[12] = red_map[(*poli).r];
        firstPixel[C13_B].bytes[12] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=14
        firstPixel[C14_G].bytes[13] = green_map[(*poli).g];
        firstPixel[C14_R].bytes[13] = red_map[(*poli).r];
        firstPixel[C14_B].bytes[13] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=15
        firstPixel[C15_G].bytes[14] = green_map[(*poli).g];
        firstPixel[C15_R].bytes[14] = red_map[(*poli).r];
        firstPixel[C15_B].bytes[14] =blue_map[(*poli).b];
        poli+=I2S_OFF;
#endif
        // poli+=I2S_OFF;
#endif
        //l2+=nun_led_per_strip;
        
        firstPixel[0].bytes[15]=255;
        firstPixel[1].bytes[15]=255;
        firstPixel[2].bytes[15]=255;
        transpose16x1_noinline2(firstPixel[0].bytes,(uint8_t*)(buff));
        transpose16x1_noinline2(firstPixel[1].bytes,(uint8_t*)(buff+192));
        transpose16x1_noinline2(firstPixel[2].bytes,(uint8_t*)(buff+384));
        //l2+=NUM_LEDS_PER_STRIP;
        poli-=I2S_OFF2;
        
        
        
        buff++;
        firstPixel[0].bytes[15]=0;
        firstPixel[1].bytes[15]=0;
        firstPixel[2].bytes[15]=0;
        
        for (int line=2;line<=NUM_VIRT_PINS;line++){
            
#ifndef STATIC_COLOR_PER_PIN
            for(int pin=0;pin<NBIS2SERIALPINS;pin++) {
                
                //uint32_t l=ledToDisplay+nun_led_per_strip*line+pin*nun_led_per_strip*5;
                
                
                firstPixel[C_G].bytes[pin] = green_map[(*poli).g];//(*poli).g/brightness_g; //scale8(int_leds[l].g,brightness_g);
                firstPixel[C_R].bytes[pin] = red_map[(*poli).r];//(*poli).r/brightness_r;
                firstPixel[C_B].bytes[pin] =blue_map[(*poli).b];//(*poli).b/brightness_b;
                //l+=nun_led_per_strip*NUM_VIRT_PINS;
                poli+=I2S_OFF;
                
                
            }
#else
#if NBIS2SERIALPINS >=1
            firstPixel[C1_G].bytes[0] = green_map[(*poli).g];
            firstPixel[C1_R].bytes[0] = red_map[(*poli).r];
            firstPixel[C1_B].bytes[0] =blue_map[(*poli).b];
            poli+=I2S_OFF;
#endif
            
#if NBIS2SERIALPINS >=2
            firstPixel[C2_G].bytes[1] = green_map[(*poli).g];
            firstPixel[C2_R].bytes[1] = red_map[(*poli).r];
            firstPixel[C2_B].bytes[1] =blue_map[(*poli).b];
            poli+=I2S_OFF;
#endif
            
#if NBIS2SERIALPINS >=3
            firstPixel[C3_G].bytes[2] = green_map[(*poli).g];
            firstPixel[C3_R].bytes[2] = red_map[(*poli).r];
            firstPixel[C3_B].bytes[2] =blue_map[(*poli).b];
            poli+=I2S_OFF;
#endif
            
#if NBIS2SERIALPINS >=4
            firstPixel[C4_G].bytes[3] = green_map[(*poli).g];
            firstPixel[C4_R].bytes[3] = red_map[(*poli).r];
            firstPixel[C4_B].bytes[3] =blue_map[(*poli).b];
            poli+=I2S_OFF;
#endif
            // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=5
            firstPixel[C5_G].bytes[4] = green_map[(*poli).g];
            firstPixel[C5_R].bytes[4] = red_map[(*poli).r];
            firstPixel[C5_B].bytes[4] =blue_map[(*poli).b];
            poli+=I2S_OFF;
#endif
            // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=6
            firstPixel[C6_G].bytes[5] = green_map[(*poli).g];
            firstPixel[C6_R].bytes[5] = red_map[(*poli).r];
            firstPixel[C6_B].bytes[5] =blue_map[(*poli).b];
            poli+=I2S_OFF;
#endif
            //  poli+=I2S_OFF;
#if NBIS2SERIALPINS >=7
            firstPixel[C7_G].bytes[6] = green_map[(*poli).g];
            firstPixel[C7_R].bytes[6] = red_map[(*poli).r];
            firstPixel[C7_B].bytes[6] =blue_map[(*poli).b];
            poli+=I2S_OFF;
#endif
            // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=8
            firstPixel[C8_G].bytes[7] = green_map[(*poli).g];
            firstPixel[C8_R].bytes[7] = red_map[(*poli).r];
            firstPixel[C8_B].bytes[7] =blue_map[(*poli).b];
            poli+=I2S_OFF;
#endif
            //  poli+=I2S_OFF;
#if NBIS2SERIALPINS >=9
            firstPixel[C9_G].bytes[8] = green_map[(*poli).g];
            firstPixel[C9_R].bytes[8] = red_map[(*poli).r];
            firstPixel[C9_B].bytes[8] =blue_map[(*poli).b];
            poli+=I2S_OFF;
#endif
            // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=10
            firstPixel[C10_G].bytes[9] = green_map[(*poli).g];
            firstPixel[C10_R].bytes[9] = red_map[(*poli).r];
            firstPixel[C10_B].bytes[9] =blue_map[(*poli).b];
            poli+=I2S_OFF;
#endif
            // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=11
            firstPixel[C11_G].bytes[10] = green_map[(*poli).g];
            firstPixel[C11_R].bytes[10] = red_map[(*poli).r];
            firstPixel[C11_B].bytes[10] =blue_map[(*poli).b];
            poli+=I2S_OFF;
#endif
            // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=12
            firstPixel[C12_G].bytes[11] = green_map[(*poli).g];
            firstPixel[C12_R].bytes[11] = red_map[(*poli).r];
            firstPixel[C12_B].bytes[11] =blue_map[(*poli).b];
            poli+=I2S_OFF;
#endif
            //poli+=I2S_OFF;
#if NBIS2SERIALPINS >=13
            firstPixel[C13_G].bytes[12] = green_map[(*poli).g];
            firstPixel[C13_R].bytes[12] = red_map[(*poli).r];
            firstPixel[C13_B].bytes[12] =blue_map[(*poli).b];
            poli+=I2S_OFF;
#endif
            // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=14
            firstPixel[C14_G].bytes[13] = green_map[(*poli).g];
            firstPixel[C14_R].bytes[13] = red_map[(*poli).r];
            firstPixel[C14_B].bytes[13] =blue_map[(*poli).b];
            poli+=I2S_OFF;
#endif
            // poli+=I2S_OFF;
#if NBIS2SERIALPINS >=15
            firstPixel[C15_G].bytes[14] = green_map[(*poli).g];
            firstPixel[C15_R].bytes[14] = red_map[(*poli).r];
            firstPixel[C15_B].bytes[14] =blue_map[(*poli).b];
            poli+=I2S_OFF;
#endif
#endif
            //l2+=nun_led_per_strip;
            
            //firstPixel[0].bytes[15]=255;
            //firstPixel[1].bytes[15]=255;
            //firstPixel[2].bytes[15]=255;
            firstPixel[0].bytes[15]=0;
            firstPixel[1].bytes[15]=0;
            firstPixel[2].bytes[15]=0;
            transpose16x1_noinline2(firstPixel[0].bytes,(uint8_t*)(buff));
            transpose16x1_noinline2(firstPixel[1].bytes,(uint8_t*)(buff+192));
            transpose16x1_noinline2(firstPixel[2].bytes,(uint8_t*)(buff+384));
            //l2+=NUM_LEDS_PER_STRIP;
            poli-=I2S_OFF2;
            
            
            
            buff++;
            
            
            
        }
        
        
        
    }
    
    
    /** Start I2S transmission
     */
    static void i2sStart()
    {
        // esp_intr_disable(gI2S_intr_handle);
        // Serial.println("I2S start");
        i2sReset();
        //Serial.println(dmaBuffers[0]->sampleCount());
        i2s->lc_conf.val=I2S_OUT_DATA_BURST_EN | I2S_OUTDSCR_BURST_EN | I2S_OUT_DATA_BURST_EN;
        //i2s.rx_eof_num = dmaBuffers[2]->sampleCount();
        i2s->out_link.addr = (uint32_t) & (dmaBuffers[2]->descriptor);
        //i2s.in_link.addr = (uint32_t) & (dmaBuffers[2]->descriptor);
        i2s->out_link.start = 1;
        ////vTaskDelay(5);
        i2s->int_clr.val = i2s->int_raw.val;
        // //vTaskDelay(5);
        i2s->int_ena.out_dscr_err = 1;
        //enable interrupt
        ////vTaskDelay(5);
        esp_intr_enable(gI2S_intr_handle);
        // //vTaskDelay(5);
        i2s->int_ena.val = 0;
        i2s->int_ena.out_eof = 1;
        
        //start transmission
        i2s->conf.tx_start = 1;
    }
    
    static void i2sReset()
    {
        // Serial.println("I2S reset");
        const unsigned long lc_conf_reset_flags = I2S_IN_RST_M | I2S_OUT_RST_M | I2S_AHBM_RST_M | I2S_AHBM_FIFO_RST_M;
        i2s->lc_conf.val |= lc_conf_reset_flags;
        i2s->lc_conf.val &= ~lc_conf_reset_flags;
        
        const uint32_t conf_reset_flags = I2S_RX_RESET_M | I2S_RX_FIFO_RESET_M | I2S_TX_RESET_M | I2S_TX_FIFO_RESET_M;
        i2s->conf.val |= conf_reset_flags;
        i2s->conf.val &= ~conf_reset_flags;
    }
    
    static void i2sReset_DMA()
    {
        i2s->lc_conf.in_rst=1; i2s->lc_conf.in_rst=0;
        i2s->lc_conf.out_rst=1; i2s->lc_conf.out_rst=0;
    }
    
    static void i2sReset_FIFO()
    {
        i2s->conf.rx_fifo_reset=1; i2s->conf.rx_fifo_reset=0;
        i2s->conf.tx_fifo_reset=1; i2s->conf.tx_fifo_reset=0;
    }
    
    static void i2sStop()
    {
        // Serial.println("I2S stop");
        esp_intr_disable(gI2S_intr_handle);
        i2sReset();
        i2s->conf.rx_start = 0;
        i2s->conf.tx_start = 0;
    }
};

FASTLED_NAMESPACE_END
