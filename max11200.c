/******************************************************************************
MIT License

Copyright (c) 2020 Andrea Nobile

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
******************************************************************************/

#include "max11200.h"
#include "max11200_spi.h"


/* spi interface calls and functions requiring interrupt manipulation */
#define MAX11200_REG_READ(reg) (MAX11200_reg_read_spi((reg)))
#define MAX11200_REG_WRITE(reg, val) (MAX11200_reg_write_spi((reg), (val)))
#define MAX11200_REG24b_READ(reg) (MAX11200_reg24b_read_spi((reg)))
#define MAX11200_CMD(cmd) (MAX11200_cmd_spi((cmd)))
#define MAX11200_INIT() (MAX11200_spi_init())
#define MAX11200_SET_DATA_HANDLER(data_handler) (MAX11200_set_driver_data_handler_spi((data_handler)))
#define MAX11200_START_CONTINUOUS_CONVERSION(rate) (MAX11200_start_continuous_conversion_spi((rate)))
#define MAX11200_IMMEDIATE_POWER_DOWN() (MAX11200_ipd_spi())

/* system interface delay */
#include "delay.h"
#define DELAY_MS(val) delay_ms((val))


/* REGISTERS */
#define MAX11200_STAT1_REG     0x00
#define MAX11200_CTRL1_REG     0x01
#define MAX11200_CTRL2_REG     0x02
#define MAX11200_CTRL3_REG     0x03
#define MAX11200_DATA_REG      0x04
#define MAX11200_SOC_REG       0x05
#define MAX11200_SCG_REG       0x06
#define MAX11200_SCOC_REG      0x07
#define MAX11200_SCGC_REG      0x08


/* CMD BITS */
#define MAX11200_CMD_RATE0     0x01
#define MAX11200_CMD_RATE1     0x02
#define MAX11200_CMD_RATE2     0x04
#define MAX11200_CMD_IMPD      0x08
#define MAX11200_CMD_CAL1      0x20
#define MAX11200_CMD_CAL0      0x10


/* STAT1 REG BITS */
#define MAX11200_STAT1_RDY     0x01
#define MAX11200_STAT1_MSTAT   0x02
#define MAX11200_STAT1_UR      0x04
#define MAX11200_STAT1_OR      0x08
#define MAX11200_STAT1_RATE0   0x10
#define MAX11200_STAT1_RATE1   0x20
#define MAX11200_STAT1_RATE2   0x40
#define MAX11200_STAT1_SYSOR   0x80


/* CTRL1 REG BITS */
#define MAX11200_CTRL1_SCYCLE  0x02
#define MAX11200_CTRL1_FORMAT  0x04
#define MAX11200_CTRL1_SIGBUF  0x08
#define MAX11200_CTRL1_REFBUF  0x10
#define MAX11200_CTRL1_EXTCLK  0x20
#define MAX11200_CTRL1_UB      0x40
#define MAX11200_CTRL1_LINEF   0x80


/* CTRL2 REG BITS */
#define MAX11200_CTRL2_DIO1    0x01
#define MAX11200_CTRL2_DIO2    0x02
#define MAX11200_CTRL2_DIO3    0x04
#define MAX11200_CTRL2_DIO4    0x08
#define MAX11200_CTRL2_DIR1    0x10
#define MAX11200_CTRL2_DIR2    0x20
#define MAX11200_CTRL2_DIR3    0x40
#define MAX11200_CTRL2_DIR4    0x80


/* CTRL3 REG BITS */
#define MAX11200_CTRL3_NOSCO   0x02
#define MAX11200_CTRL3_NOSCG   0x04
#define MAX11200_CTRL3_NOSYSO  0x08
#define MAX11200_CTRL3_NOSYSG  0x10
/* MAX11210 ONLY */
#define MAX11200_CTRL3_DGAIN0  0x20
#define MAX11200_CTRL3_DGAIN1  0x40
#define MAX11200_CTRL3_DGAIN3  0x80


typedef struct
{
    uint8_t ctrl1;
    uint8_t ctrl2;
    uint8_t ctrl3;
    uint8_t stat1;
} max11200_ctrl_stat_regs_t;

static max11200_ctrl_stat_regs_t max11200_ctrl_stat_regs;
static void (*user_data_handler)(uint32_t data, uint8_t stat);

static void max11200_read_ctrl_stat_regs()
{
    max11200_ctrl_stat_regs.ctrl1 = MAX11200_REG_READ(MAX11200_CTRL1_REG);
    max11200_ctrl_stat_regs.ctrl2 = MAX11200_REG_READ(MAX11200_CTRL2_REG);
    max11200_ctrl_stat_regs.ctrl3 = MAX11200_REG_READ(MAX11200_CTRL3_REG);
    max11200_ctrl_stat_regs.stat1 = MAX11200_REG_READ(MAX11200_STAT1_REG);
}

void max11200_init_config(max11200_config_data *config)
{
    config->scycle = MAX11200_CONFIG_CONVERSION_SINGLE;
    config->format = MAX11200_CONFIG_FORMAT_OFFSET_BINARY;
    config->sigbuf = MAX11200_CONFIG_SIGBUF_DISABLE;
    config->refbuf = MAX11200_CONFIG_REFBUF_DISABLE;
    config->extclk = MAX11200_CONFIG_CLK_INTERNAL;
    config->unipolar_bipolar = MAX11200_CONFIG_UNIPOLAR;
    config->line_filter = MAX11200_CONFIG_LINEF_50HZ;
}

void max11200_read_config(max11200_config_data *config)
{
    uint8_t ctrl1 = MAX11200_REG_READ(MAX11200_CTRL1_REG);

    config->scycle = ctrl1 & MAX11200_CTRL1_SCYCLE;
    config->format = ctrl1 & MAX11200_CTRL1_FORMAT;
    config->sigbuf = ctrl1 & MAX11200_CTRL1_SIGBUF;
    config->refbuf = ctrl1 & MAX11200_CTRL1_REFBUF;
    config->extclk = ctrl1 & MAX11200_CTRL1_EXTCLK;
    config->unipolar_bipolar = ctrl1 & MAX11200_CTRL1_UB;
    config->line_filter = ctrl1 & MAX11200_CTRL1_LINEF;

    max11200_ctrl_stat_regs.ctrl1 = ctrl1;
}

void max11200_write_config(max11200_config_data *config)
{
    uint8_t ctrl1 = config->scycle | config->format | config->sigbuf | config->refbuf |
            config->extclk | config->unipolar_bipolar | config->line_filter;

    MAX11200_REG_WRITE(MAX11200_CTRL1_REG, ctrl1);

    max11200_ctrl_stat_regs.ctrl1 = ctrl1;
}



uint32_t max11200_read_data()
{
    return MAX11200_REG24b_READ(MAX11200_DATA_REG);
}

uint8_t max11200_read_stat()
{
    uint8_t regval = MAX11200_REG_READ(MAX11200_STAT1_REG);
    max11200_ctrl_stat_regs.stat1 = regval;
    return regval;
}

int32_t max11200_conversion_ready()
{
    uint8_t stat = max11200_read_stat();
    return (stat & MAX11200_STAT1_RDY);
}

int32_t max11200_measure_in_progress()
{
    uint8_t stat = max11200_read_stat();
    return (stat & MAX11200_STAT1_MSTAT);
}

uint32_t max11200_convert(uint8_t rate)
{
    max11200_start_conversion(rate);
    while(!max11200_conversion_ready());

    uint32_t val = MAX11200_REG24b_READ(MAX11200_DATA_REG);
    return val;
}

void max11200_start_conversion(uint8_t rate)
{
    if((max11200_ctrl_stat_regs.ctrl1 & MAX11200_CTRL1_SCYCLE) == 0) {
        uint8_t ctrl1 = MAX11200_REG_READ(MAX11200_CTRL1_REG);
        ctrl1 |= MAX11200_CTRL1_SCYCLE;
        MAX11200_REG_WRITE(MAX11200_CTRL1_REG, ctrl1);
        max11200_ctrl_stat_regs.ctrl1 = ctrl1;
    }
    MAX11200_CMD(rate);
}



/* power down, stop continuous conversion */
void max11200_immediate_power_down()
{
    MAX11200_IMMEDIATE_POWER_DOWN();
}

void max11200_data_ready_interrupt_handler(uint32_t data, uint8_t stat)
{
    if(user_data_handler) {
        user_data_handler(data, stat);
    }
}

void max11200_start_continuous_conversion(uint8_t rate, void (*data_handler)(uint32_t data, uint8_t stat))
{
    user_data_handler = data_handler;
    if(max11200_ctrl_stat_regs.ctrl1 & MAX11200_CTRL1_SCYCLE) {
        uint8_t ctrl1 = MAX11200_REG_READ(MAX11200_CTRL1_REG);
        ctrl1 &= ~MAX11200_CTRL1_SCYCLE;
        MAX11200_REG_WRITE(MAX11200_CTRL1_REG, ctrl1);
        max11200_ctrl_stat_regs.ctrl1 = ctrl1;
    }
    MAX11200_START_CONTINUOUS_CONVERSION(rate);
}



void max11200_self_calibration(uint32_t *calib_offset, uint32_t *calib_gain)
{
    /* enable self calibration registers, disable system calibration regsiters, digital gain (MAX11210) disabled */
    uint8_t ctrl3 = MAX11200_CTRL3_NOSYSO | MAX11200_CTRL3_NOSYSG;
    MAX11200_REG_WRITE(MAX11200_CTRL3_REG, ctrl3);
    max11200_ctrl_stat_regs.ctrl3 = ctrl3;

    /* start self calibration */
    MAX11200_CMD(MAX11200_CMD_CAL0);

    /* self calibration takes ~300 ms, delay for 500 */
    DELAY_MS(500);

    *calib_offset = MAX11200_REG24b_READ(MAX11200_SCOC_REG);
    *calib_gain = MAX11200_REG24b_READ(MAX11200_SCGC_REG);
}



int32_t max11200_init()
{
    uint32_t ret =  MAX11200_INIT();
    MAX11200_SET_DATA_HANDLER(max11200_data_ready_interrupt_handler);
    max11200_read_ctrl_stat_regs();
    return ret;
}

