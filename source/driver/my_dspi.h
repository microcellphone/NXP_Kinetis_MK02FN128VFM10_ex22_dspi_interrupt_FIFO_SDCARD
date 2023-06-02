#ifndef __MY_DSPI_H__
#define __MY_DSPI_H__

#include <stdint.h>

#define POLLING_MODE        0
#if POLLING_MODE
#define INTERRUPT_MODE      0
#else
#define INTERRUPT_MODE      1
#endif

enum SSP_SPEED {SSP_SLOW, SSP_FAST};
enum CS_TYPE  {CS_AUTO, CS_GPIO};

extern void SPI_Config_Request(uint32_t bitlen, uint32_t bitrate, uint32_t spi_cs_type);
extern void  SPI_Tx_Rx_Data(uint8_t *txdata, uint8_t tx_length, uint8_t *rxdata, uint8_t rx_length);
extern uint32_t SPI_TxRxData(uint32_t txdata);
extern void SPI_TxData(uint32_t txdata);
extern uint32_t SPI_RxData(void);
extern void SSP0_Send_Request(uint8_t dat);
extern void SSP0_Send_Request16(uint16_t dat);

#endif // __MY_DSPI_H__
