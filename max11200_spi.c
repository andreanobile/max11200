#include "max11200_spi.h"
#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include <stdint.h>

#include "profiling.h"


#define SPI_MASTER                   SPI1
#define SPI_MASTER_CLK               RCC_APB2Periph_SPI1
#define SPI_MASTER_GPIO              GPIOA
#define SPI_MASTER_GPIO_CLK          RCC_APB2Periph_GPIOA
#define SPI_MASTER_PIN_NSS           GPIO_Pin_4
#define SPI_MASTER_PIN_SCK           GPIO_Pin_5
#define SPI_MASTER_PIN_MISO          GPIO_Pin_6
#define SPI_MASTER_PIN_MOSI          GPIO_Pin_7

#define SPI_MASTER_NSS0() (SPI_MASTER_GPIO->BRR = SPI_MASTER_PIN_NSS)
#define SPI_MASTER_NSS1() (SPI_MASTER_GPIO->BSRR = SPI_MASTER_PIN_NSS)

#define MAX11200_WRITE 0x00
#define MAX11200_READ  0x01
#define MAX11200_START 0x80
#define MAX11200_MODE1 0x40

#define DISABLE_INTERRUPT_LINE(a) EXTI->IMR &= ~( (a) )
#define ENABLE_INTERRUPT_LINE(a) EXTI->IMR |= ( (a) )


/* REGISTERS */
#define MAX11200_STAT1_REG 0x00
#define MAX11200_CTRL1_REG 0x01
#define MAX11200_CTRL2_REG 0x02
#define MAX11200_CTRL3_REG 0x03
#define MAX11200_DATA_REG  0x04
#define MAX11200_SOC_REG   0x05
#define MAX11200_SCG_REG   0x06
#define MAX11200_SCOC_REG  0x07
#define MAX11200_SCGC_REG  0x08

/* CMD BITS */
#define MAX11200_CMD_RATE0     0x01
#define MAX11200_CMD_RATE1     0x02
#define MAX11200_CMD_RATE2     0x04
#define MAX11200_CMD_IMPD      0x08
#define MAX11200_CMD_CAL1      0x20
#define MAX11200_CMD_CAL0      0x10


uint32_t MAX11200_reg24b_read_spi(uint8_t reg)
{
    SPI_MASTER_NSS0();

    uint8_t cmd = MAX11200_START | MAX11200_MODE1 | MAX11200_READ;

    //insert the reg number
    cmd = cmd | (reg << 1);

    //send & recv

    //clear RXNE flag
    SPI_I2S_ReceiveData(SPI_MASTER);

    //see fig 241 of stm32f1 reference manual RM0008 rev 20
    SPI_I2S_SendData(SPI_MASTER, cmd);

    while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI_MASTER, 0);

    /* rx not empty goes up as soon as the first byte is sent
       (MISO sensed the line during the first 8 clocks)
       it contains garbage (level of MISO during cmd)
       wait for rxne to go up and read the gargabe to clear it */
    while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_RXNE) == RESET);
    uint32_t recv0 = SPI_I2S_ReceiveData(SPI_MASTER);

    while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI_MASTER, 0);

    // wait for actual value (after the second byte (0) goes out)
    while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_RXNE) == RESET);
    recv0 = SPI_I2S_ReceiveData(SPI_MASTER);

    while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI_MASTER, 0);

    // wait for actual value (after the second byte (0) goes out)
    while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_RXNE) == RESET);
    uint32_t recv1 = SPI_I2S_ReceiveData(SPI_MASTER);


    // wait for actual value (after the second byte (0) goes out)
    while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_RXNE) == RESET);
    uint32_t recv2 = SPI_I2S_ReceiveData(SPI_MASTER);

    SPI_MASTER_NSS1();

    return  (recv0 << 16) | (recv1 << 8) |  recv2 ;
}


uint8_t MAX11200_reg_read_spi(uint8_t reg)
{
    SPI_MASTER_NSS0();

    uint8_t cmd = MAX11200_START | MAX11200_MODE1 | MAX11200_READ;

    //insert the reg number
    cmd = cmd | (reg << 1);

    //see fig 241 of stm32f1 reference manual RM0008 rev 20

    //clear RXNE flag
    SPI_I2S_ReceiveData(SPI_MASTER);

    SPI_I2S_SendData(SPI_MASTER, cmd);
    while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_TXE) == RESET);

    SPI_I2S_SendData(SPI_MASTER, 0);

    /* rx not empty goes up as soon as the first byte is sent
       (MISO sensed the line during the first 8 clocks)
       it contains garbage (level of MISO during cmd)
       wait for rxne to go up and read the gargabe to clear it */
    while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_RXNE) == RESET);
    uint8_t recv = SPI_I2S_ReceiveData(SPI_MASTER);

    // wait for actual value (after the second byte (0) goes out)
    while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_RXNE) == RESET);
    recv = SPI_I2S_ReceiveData(SPI_MASTER);

    SPI_MASTER_NSS1();

    return  recv;
}


void MAX11200_reg_write_spi(uint8_t reg, uint8_t val)
{
    SPI_MASTER_NSS0();

    uint8_t cmd = MAX11200_START | MAX11200_MODE1; // write bit == 0

    cmd = cmd | (reg << 1);

    SPI_I2S_SendData(SPI_MASTER, cmd);
    while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_TXE) == RESET);

    SPI_I2S_SendData(SPI_MASTER, val);
    while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_TXE) == RESET);
    while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_BSY) == SET);

    SPI_MASTER_NSS1();
}

void MAX11200_reg24b_write_spi(uint8_t reg, uint32_t val)
{
    SPI_MASTER_NSS0();

    uint8_t cmd = MAX11200_START | MAX11200_MODE1; // write bit == 0

    cmd = cmd | (reg << 1);

    SPI_I2S_SendData(SPI_MASTER, cmd);
    while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_TXE) == RESET);

    uint8_t v0 = (val & 0x00FF0000) >> 16;
    SPI_I2S_SendData(SPI_MASTER, v0);
    while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_TXE) == RESET);

    uint8_t v1 = (val & 0x0000FF00) >> 8;
    SPI_I2S_SendData(SPI_MASTER, v1);
    while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_TXE) == RESET);

    uint8_t v2 = (val & 0x000000FF);
    SPI_I2S_SendData(SPI_MASTER, v2);
    while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_TXE) == RESET);

    while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_BSY) == SET);

    SPI_MASTER_NSS1();
}


void MAX11200_cmd_spi(uint8_t cmd)
{
    SPI_MASTER_NSS0();

    cmd = MAX11200_START | (cmd & 0x3F);
    SPI_I2S_SendData(SPI_MASTER, cmd);

    while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_TXE) == RESET);
    while (SPI_I2S_GetFlagStatus(SPI_MASTER, SPI_I2S_FLAG_BSY) == SET);

    SPI_MASTER_NSS1();
}



void MAX11200_start_continuous_conversion_spi(uint8_t rate)
{
    MAX11200_cmd_spi(rate);
    SPI_MASTER_NSS0();
    ENABLE_INTERRUPT_LINE(EXTI_Line6);
}


void (*driver_data_handler)(uint32_t data, uint8_t stat);

/* called by EXTI9_5_IRQHandler in stm32f10x_it.c  */
static void cont_conv_data_ready_ith()
{
    profile_start(0);
    DISABLE_INTERRUPT_LINE(EXTI_Line6);
    uint32_t data = MAX11200_reg24b_read_spi(MAX11200_DATA_REG);
    uint8_t stat = MAX11200_reg_read_spi(MAX11200_STAT1_REG);

    if (driver_data_handler) {
        driver_data_handler(data, stat);
    }

    SPI_MASTER_NSS0();
    ENABLE_INTERRUPT_LINE(EXTI_Line6);
    profile_stop(0);
}


void MAX11200_set_driver_data_handler_spi(void (*handler)(uint32_t data, uint8_t stat))
{
    driver_data_handler = handler;
}


int32_t MAX11200_spi_init()
{
    /* Enable SPI_MASTER clock and GPIO clock for SPI_MASTER */
    RCC_APB2PeriphClockCmd(SPI_MASTER_GPIO_CLK | SPI_MASTER_CLK, ENABLE);

    /* Configure SPI_MASTER pins: NSS, SCK and MOSI */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = SPI_MASTER_PIN_SCK | SPI_MASTER_PIN_MOSI;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(SPI_MASTER_GPIO, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = SPI_MASTER_PIN_NSS;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(SPI_MASTER_GPIO, &GPIO_InitStructure);
    GPIOA->BSRR = SPI_MASTER_PIN_NSS;

    GPIO_InitStructure.GPIO_Pin =  SPI_MASTER_PIN_MISO;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(SPI_MASTER_GPIO, &GPIO_InitStructure);


    /* SPI_MASTER configuration ------------------------------------------------------*/
    SPI_InitTypeDef  SPI_InitStructure;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_16; // 32 := sysclk @48Mhz  -> spi @1.5Mhz
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(SPI_MASTER, &SPI_InitStructure);

    SPI_Cmd(SPI_MASTER, ENABLE);



    /* interrupt init */

    /* setup interrupt on exti line 6 */
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    /* Connect EXTI0 Line to PA.6 pin */
    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource6);

    EXTI_InitStructure.EXTI_Line = EXTI_Line6;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;

    EXTI_Init(&EXTI_InitStructure);
    DISABLE_INTERRUPT_LINE(EXTI_Line6);

    /* Enable and set EXTI0 Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI9_5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    /* set EXTI handler, called by EXTI9_5_IRQHandler in stm32f10x_it.c */
    exti6_handler = cont_conv_data_ready_ith;

    profile_enable();
    profile_init(0);

    return 0;
}

void MAX11200_ipd_spi()
{
    DISABLE_INTERRUPT_LINE(EXTI_Line6);
    MAX11200_cmd_spi(MAX11200_CMD_IMPD);
}
