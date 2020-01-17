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
