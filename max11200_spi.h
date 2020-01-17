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

#ifndef MAX1200_SPI_H
#define MAX1200_SPI_H

#include <stdint.h>

int32_t MAX11200_spi_init();

uint8_t MAX11200_reg_read_spi(uint8_t reg);
void MAX11200_reg_write_spi(uint8_t reg, uint8_t val);
uint32_t MAX11200_reg24b_read_spi(uint8_t reg);
void MAX11200_reg24b_write_spi(uint8_t reg, uint32_t val);
void MAX11200_cmd_spi(uint8_t cmd);

void MAX11200_set_driver_data_handler_spi(void (*handler)(uint32_t data, uint8_t stat));
void MAX11200_start_continuous_conversion_spi(uint8_t rate);
void MAX11200_ipd_spi();

#endif //MAX1200_SPI_H
