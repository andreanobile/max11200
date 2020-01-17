#ifndef MAX1200_H
#define MAX1200_H

#include <stdint.h>

#define MAX11200_CONFIG_CONVERSION_CONTINOUS 0x00
#define MAX11200_CONFIG_CONVERSION_SINGLE    0x02

#define MAX11200_CONFIG_FORMAT_2COMPLEMENT   0x00
#define MAX11200_CONFIG_FORMAT_OFFSET_BINARY 0x04

#define MAX11200_CONFIG_SIGBUF_ENABLE        0x08
#define MAX11200_CONFIG_SIGBUF_DISABLE       0x00

#define MAX11200_CONFIG_REFBUF_ENABLE        0x10
#define MAX11200_CONFIG_REFBUF_DISABLE       0x00

#define MAX11200_CONFIG_CLK_EXTERNAL         0x20
#define MAX11200_CONFIG_CLK_INTERNAL         0x00

#define MAX11200_CONFIG_UNIPOLAR             0x40
#define MAX11200_CONFIG_BIPOLAR              0x00

#define MAX11200_CONFIG_LINEF_50HZ           0x80
#define MAX11200_CONFIG_LINEF_60HZ           0x00

#define MAX11200_STAT_MEASURE_RDY            0x01
#define MAX11200_STAT_MODUATOR_BSY           0x02
#define MAX11200_STAT_MEASURE_UNDER_RANGE    0x04
#define MAX11200_STAT_MEASURE_OVER_RANGE     0x08

#define MAX11200_SCYCLE_RATE_1SPS    0x00
#define MAX11200_SCYCLE_RATE_2_5SPS  0x01
#define MAX11200_SCYCLE_RATE_5SPS    0x02
#define MAX11200_SCYCLE_RATE_10SPS   0x03
#define MAX11200_SCYCLE_RATE_15SPS   0x04
#define MAX11200_SCYCLE_RATE_30SPS   0x05
#define MAX11200_SCYCLE_RATE_60SPS   0x06
#define MAX11200_SCYCLE_RATE_120SPS  0x07

#define MAX11200_CONT_RATE_60SPS     0x04
#define MAX11200_CONT_RATE_120SPS    0x05
#define MAX11200_CONT_RATE_240SPS    0x06
#define MAX11200_CONT_RATE_480SPS    0x07



/* data and fucntions to setup adc config */
struct max11200_config_data
{
    uint8_t scycle;
    uint8_t format;
    uint8_t sigbuf;
    uint8_t refbuf;
    uint8_t extclk;
    uint8_t unipolar_bipolar;
    uint8_t line_filter;
};

typedef struct max11200_config_data max11200_config_data;

void max11200_init_config(max11200_config_data *config);
void max11200_read_config(max11200_config_data *config);
void max11200_write_config(max11200_config_data *config);


/* initilize low level interface (spi) and driver state */
int32_t max11200_init();

/* adc self calibration */
void max11200_self_calibration(uint32_t *calib_offset, uint32_t *calib_gain);

/* convert and wait */
uint32_t max11200_convert(uint8_t rate);

/* start single conversion and returns immediatley */
void max11200_start_conversion(uint8_t rate);

/* check if conversion is ready */
int32_t max11200_conversion_ready();

/* read data register (coversion result) */
uint32_t max11200_read_data();

/* read status register  */
uint8_t max11200_read_stat();

/* start cont conversion, the passed callback gets called each time a conversion is completed, uses interrupts */
void max11200_start_continuous_conversion(uint8_t rate, void (*data_handler)(uint32_t data, uint8_t stat));

/* power down, stop continuous conversion */
void max11200_immediate_power_down();


#endif //MAX1200_H
