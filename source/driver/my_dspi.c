#include <my_dspi.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK02F12810.h"
#include "fsl_dspi.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define TRANSFER_SIZE     8U    /*! Transfer dataSize */

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t masterRxData[TRANSFER_SIZE] = {0U};
uint8_t masterTxData[TRANSFER_SIZE] = {0U};

volatile uint32_t masterTxCount;
volatile uint32_t masterRxCount;
volatile uint32_t TxCount;
volatile uint32_t RxCount;
volatile uint32_t masterCommand;
uint32_t masterFifoSize;

dspi_master_handle_t g_m_handle;

volatile bool isTransferCompleted = false;

/*******************************************************************************
 * Code
 ******************************************************************************/
void SPI0_IRQHandler(void)
{
    if (masterRxCount < RxCount) {
        while (DSPI_GetStatusFlags(SPI0) & kDSPI_RxFifoDrainRequestFlag) {
            masterRxData[masterRxCount] = DSPI_ReadData(SPI0);
            ++masterRxCount;

            DSPI_ClearStatusFlags(SPI0, kDSPI_RxFifoDrainRequestFlag);

            if (masterRxCount == RxCount) {
            	  isTransferCompleted = true;
                DSPI_DisableInterrupts(SPI0, kDSPI_RxFifoDrainRequestInterruptEnable);
                break;
            }
        }
    }

    if (masterTxCount < TxCount) {
        while ((DSPI_GetStatusFlags(SPI0) & kDSPI_TxFifoFillRequestFlag) &&
               ((masterTxCount - masterRxCount) < masterFifoSize))
        {
            if (masterTxCount < TxCount) {
                SPI0->PUSHR = masterCommand | masterTxData[masterTxCount];
                ++masterTxCount;
            } else {
            	isTransferCompleted = true;
                DSPI_DisableInterrupts(SPI0, kDSPI_TxFifoFillRequestInterruptEnable);
                break;
              }

            /* Try to clear the TFFF; if the TX FIFO is full this will clear */
            DSPI_ClearStatusFlags(SPI0, kDSPI_TxFifoFillRequestFlag);
        }
    }

    /* Check if we're done with this transfer.*/
    if ((masterTxCount == TRANSFER_SIZE) && (masterRxCount == TRANSFER_SIZE)) {
        /* Complete the transfer and disable the interrupts */
        DSPI_DisableInterrupts(SPI0, kDSPI_RxFifoDrainRequestInterruptEnable | kDSPI_TxFifoFillRequestInterruptEnable);
    }
    SDK_ISR_EXIT_BARRIER;
}

void SPI_Config_Request(uint32_t bitlen, uint32_t speed, uint32_t spi_cs_type)
{
	return;
}

void  SPI_Tx_Rx_Data(uint8_t *txdata, uint8_t tx_length, uint8_t *rxdata, uint8_t rx_length)
{
    uint32_t i;

	TxCount = tx_length;
	RxCount = rx_length;
	isTransferCompleted = false;

    for (i = 0U; i < tx_length; i++) {
        masterTxData[i] = txdata[i];
        masterRxData[i] = 0U;
    }

    /* Start master transfer*/
    dspi_command_data_config_t commandData;
    commandData.isPcsContinuous    = false;
    commandData.whichCtar          = kDSPI_Ctar0;
    commandData.whichPcs           = kDSPI_Pcs0;
    commandData.isEndOfQueue       = false;
    commandData.clearTransferCount = false;

    masterCommand = DSPI_MasterGetFormattedCommand(&commandData);

    masterFifoSize = FSL_FEATURE_DSPI_FIFO_SIZEn(SPI0);
    masterTxCount  = 0;
    masterRxCount  = 0;

    DSPI_StopTransfer(SPI0);
    DSPI_FlushFifo(SPI0, true, true);
    DSPI_ClearStatusFlags(SPI0, (uint32_t)kDSPI_AllStatusFlag);

    /*Enable master RX interrupt*/
    DSPI_EnableInterrupts(SPI0, kDSPI_RxFifoDrainRequestInterruptEnable);

    /*Fill up the master Tx data*/
    while (DSPI_GetStatusFlags(SPI0) & kDSPI_TxFifoFillRequestFlag) {
        if (masterTxCount < tx_length) {
            DSPI_MasterWriteData(SPI0, &commandData, masterTxData[masterTxCount]);
            ++masterTxCount;
        } else {
            break;
         }

        /* Try to clear the TFFF; if the TX FIFO is full this will clear */
        DSPI_ClearStatusFlags(SPI0, kDSPI_TxFifoFillRequestFlag);
    }

    /* Start DSPI transafer.*/
    DSPI_StartTransfer(SPI0);

    /* Wait slave received all data. */
    while (!isTransferCompleted){}

    for (i = 0U; i < rx_length; i++) {
        rxdata[i] = masterRxData[i];
    }

	return;
}


uint32_t SPI_TxRxData(uint32_t txdata)
{
  uint32_t rxdata;

  SPI_Tx_Rx_Data((uint8_t*)&txdata, 1, (uint8_t*)&rxdata, 1);

  return rxdata;
}


void SPI_TxData(uint32_t txdata)
{
    SPI_TxRxData(txdata);
}


uint32_t SPI_RxData(void)
{
    uint32_t rxdata;

    rxdata = SPI_TxRxData(0xff);
    return rxdata;
}


void SSP0_Send_Request(uint8_t dat)
{
  SPI_TxData(dat);
}

void SSP0_Send_Request16(uint16_t dat)
{
  SSP0_Send_Request((uint8_t)(dat>>8));
  SSP0_Send_Request((uint8_t)dat);
}
