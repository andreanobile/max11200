#include "delay.h"
#include "max11200.h"
#include "stm32f10x.h"

#include <stdio.h>

#define SYSTICK_INIT(frequency_hz) ((void) SysTick_Config (SystemCoreClock / (frequency_hz)))

void adc_init()
{
    max11200_init();

    max11200_config_data config;
    max11200_init_config(&config);


    config.scycle = MAX11200_CONFIG_CONVERSION_CONTINOUS;
    config.format = MAX11200_CONFIG_FORMAT_OFFSET_BINARY;
    config.sigbuf = MAX11200_CONFIG_SIGBUF_DISABLE;
    config.refbuf = MAX11200_CONFIG_REFBUF_DISABLE;
    config.extclk = MAX11200_CONFIG_CLK_INTERNAL;
    config.unipolar_bipolar = MAX11200_CONFIG_UNIPOLAR;
    config.line_filter = MAX11200_CONFIG_LINEF_50HZ;

    max11200_write_config(&config);

    uint32_t offset, gain;
    max11200_self_calibration(&offset, &gain);
}


static void data_handler(uint32_t data, uint8_t stat)
{
    printf("read value %lu stat = 0x%02x\n", data, (uint32_t)stat);
}


int main(void)
{
    adc_init();

    delay_ms(10);

    max11200_start_continuous_conversion(MAX11200_CONT_RATE_60SPS, data_handler);

    while(1) {
        delay_ms(1000);
        max11200_immediate_power_down();
        uint32_t data = max11200_convert(MAX11200_SCYCLE_RATE_1SPS);
        printf("read value %lu\n", data);
        max11200_start_continuous_conversion(MAX11200_CONT_RATE_60SPS, data_handler);
    }

    return 0;
}
