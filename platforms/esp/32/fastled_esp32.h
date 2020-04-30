#pragma once

#include "fastpin_esp32.h"
#ifdef ESP32_VIRTUAL_DRIVER
        #ifdef ESP_VIRTUAL_DRIVER_8
            #include "clockless_i2s_virtual_esp32_84_2.h" //the 8 pîns with the order
        #else
            #ifdef ESP_VIRTUAL_DRIVER_82_2
                #include "clockless_i2s_virtual_esp32_82_2.h" //the 8n pins reveertse order
            #else
                #include "clockless_i2s_virtual_esp32.h" //the 5 pins version
            #endif
        #endif
#else
	#ifdef FASTLED_ESP32_I2S
		#include "clockless_i2s_esp32.h"
	#else
		#include "clockless_rmt_esp32.h"
	#endif
#endif
// #include "clockless_block_esp32.h"
