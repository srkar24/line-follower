/**
 * @file EUSCI_A3_SPI.h
 * @brief Header file for the EUSCI_A3_SPI driver.
 *
 * This file contains the function definitions for the EUSCI_A3_SPI driver.
 *
 * @note This function assumes that the necessary pin configurations for SPI communication have been performed
 *       on the corresponding pins. The output from the pins will be observed using an oscilloscope.
 *       - P9.4 (SCE, Slave Chip Enable)
 *       - P9.5 (SCLK, Slave Clock)
 *       - P9.6 (MISO, Master In Slave Out)
 *       - P9.7 (MOSI, Master Out Slave In)
 *
 * @note For more information regarding the eUSCI_SPI registers used, refer to the eUSCI_A SPI Registers section (25.4)
 * of the MSP432Pxx Microcontrollers Technical Reference Manual
 *
 * @author Aaron Nanas
 *
 */

#ifndef INC_EUSCI_A3_SPI_H_
#define INC_EUSCI_A3_SPI_H_

#include <stdio.h>
#include <stdint.h>
#include "msp.h"
#include "Clock.h"

#define SPI_CLK_1_MHZ       1000000
#define AUTO_CHIP_SELECT    0x01
#define MANUAL_CHIP_SELECT  0x00

/**
 * @brief Initializes the SPI module EUSCI_A3 for communication.
 *
 * This function configures the EUSCI_A3 module to enable SPI communication
 * with the following configuration:
 *
 * - CTLW0 Register Configuration:
 *
 *  Bit(s)      Field       Value       Description
 *  -----       -----       -----       -----------
 *   15         UCCKPH       0x1        Data is captured on the first edge and changed on the following edge
 *   14         UCCKPL       0x0        Clock is low when inactive
 *   13         UCMSB        0x1        MSB first
 *   12         UC7BIT       0x0        8-bit data
 *   11         UCMST        0x1        Master mode is selected
 *   10-9       UCMODEx      0x2        4-pin SPI with active low UCSTE
 *   8          UCSYNC       0x1        Synchronous mode
 *   7-6        UCSSELx      0x2        eUSCI clock source is SMCLK
 *   5-2        Reserved     0x0        Reserved
 *   1          UCSTEM       0x1        UCSTE pin is used to generate signal for 4-wire slave
 *   0          UCSWRST      0x1        eUSCI logic held in reset state
 *
 * @param uint8_t chip_select_mode Selects between automatic toggle mode (0x01)
 * manual toggle mode (0x00) for the chip select signal.
 *
 * @param uint32_t clock_frequency Configures the frequency of the SPI clock.
 *
 * @return None
 */
void EUSCI_A3_SPI_Init(uint8_t ucstem_select, uint32_t clock_frequency);

/**
 * @brief Receives a single character over SPI using the EUSCI_A3 module.
 *
 * This function receives a single character over SPI using the EUSCI_A3 module.
 * It waits until a character is available in the SPI receive buffer and then reads
 * the received data.
 *
 * @param None
 *
 * @return The received unsigned 8-bit data from the MISO line.
 */
uint8_t EUSCI_A3_SPI_Data_Read();

/**
 * @brief Transmits a single byte over SPI using the EUSCI_A3 module.
 *
 * This function transmits a single character over SPI using the EUSCI_A3 module.
 * It waits until the transmit buffer is ready to accept new data and then writes the provided data
 * to the transmit buffer for transmission.
 *
 * @param data The unsigned 8-bit data to be transmitted over the MOSI line.
 *
 * @return None
 */
void EUSCI_A3_SPI_Data_Write(uint8_t data);

#endif /* INC_EUSCI_A3_SPI_H_ */
