/**
 * @file PMOD_JSTK2.c
 * @brief Source code for the PMOD_JSTK2 driver.
 *
 * This file contains the function definitions for the PMOD_JSTK2 driver.
 *
 * It interfaces with the PMOD JSTK2 module, which uses the SPI communication protocol.
 *  - Product Link: https://digilent.com/shop/pmod-jstk2-two-axis-joystick/
 *  - Reference Manual: https://digilent.com/reference/pmod/pmodjstk2/reference-manual
 *
 * The PMOD JSTK2 module uses the following SPI configuration:
 *  - SPI Mode 0
 *  - Chip Select Active Low
 *  - SCLK Frequency: 1 MHz
 *  - MSB First
 *
 * The following connections must be made:
 *  - PMOD JSTK2 CS     (Pin 1)     <-->  MSP432 LaunchPad Pin P9.4 (CS)
 *  - PMOD JSTK2 MOSI   (Pin 2)     <-->  MSP432 LaunchPad Pin P9.7 (MOSI)
 *  - PMOD JSTK2 MISO   (Pin 3)     <-->  MSP432 LaunchPad Pin P9.6 (MISO)
 *  - PMOD JSTK2 SCLK   (Pin 4)     <-->  MSP432 LaunchPad Pin P9.5 (SCLK)
 *  - PMOD JSTK2 GND    (Pin 5)     <-->  MSP432 LaunchPad GND
 *  - PMOD JSTK2 VCC    (Pin 6)     <-->  MSP432 LaunchPad VCC (3.3V)
 *
 * @note For more information regarding the Enhanced Universal Serial Communication Interface (eUSCI),
 * refer to the MSP432Pxx Microcontrollers Technical Reference Manual
 *
 * @author Aaron Nanas
 *
 */

#include "../inc/PMOD_JSTK2.h"

// Initialize global buffer to store received data from the PMOD JSTK2 module
uint8_t rx_buffer[10] = {0};

// Declare global variables to store the fsButtons status:
//  - Joystick Center Button (Bit 0)
//  - Trigger Button (Bit 1)
//
//  1 = Pressed
//  0 = Not pressed
uint8_t Center_Button_Status;
uint8_t Trigger_Button_Status;

// Declare global variables to store 10-bit X and Y position values
int X_Position_10_Bit;
int Y_Position_10_Bit;

// Declare global variables to store 8-bit X and Y position values
uint8_t X_Position_8_Bit;
uint8_t Y_Position_8_Bit;

void PMOD_JSTK2_Init()
{
    EUSCI_A3_SPI_Init(MANUAL_CHIP_SELECT, SPI_CLK_1_MHZ);
}

void PMOD_JSTK2_Chip_Select(uint8_t chip_select_enable)
{
    // Clear P9.4 to 0 when chip select is enabled
    if (chip_select_enable == 0x00)
    {
        P9->OUT &= ~0x10;
    }

    // Set P9.4 to 1 when chip select is disabled
    else
    {
        P9->OUT |= 0x10;
    }
}

void PMOD_JSTK2_Write_SPI_Data(uint8_t data)
{
    EUSCI_A3_SPI_Data_Write(data);
}

uint8_t PMOD_JSTK2_Read_SPI_Data()
{
    uint8_t received_data = EUSCI_A3_SPI_Data_Read();

    return received_data;
}

void PMOD_JSTK2_Transfer_and_Receive_Packet(uint8_t* cmd_buffer, uint8_t buffer_length)
{
    // Enable the chip select line
    PMOD_JSTK2_Chip_Select(0x00);

    // 5 us delay between chip select low to the first byte
    Clock_Delay1us(5);

    for (int i = 0; i < buffer_length; i++)
    {
        // An interbyte delay of at lesat 10 us is required when transferring multiple bytes
        Clock_Delay1us(10);

        // Write to the TX buffer and transmit the data from cmd_buffer to the PMOD JSTK2 module
        PMOD_JSTK2_Write_SPI_Data(cmd_buffer[i]);

        // Receive data from the PMOD JSTK2 module and write it to the RX buffer
        rx_buffer[i] = PMOD_JSTK2_Read_SPI_Data();
    }

    // Disable the chip select line
    PMOD_JSTK2_Chip_Select(0x01);

    // At least 25 us is required before chip select can be driven low to initiate another communication session
    Clock_Delay1us(25);
}

void PMOD_JSTK2_Get_Basic_Packet()
{
    // Initialize cmd_buffer with zeros
    uint8_t cmd_buffer[5] = {0};

    // Write the command to get the standard 5 bytes of data to the first index of cmd_buffer
    cmd_buffer[0] = JSTK2_CMD_GET_RAW;

    // Transmit 5 bytes of data to the PMOD JSTK2 (MOSI) and receive the packet (MISO)
    PMOD_JSTK2_Transfer_and_Receive_Packet(cmd_buffer, 5);
}

void PMOD_JSTK2_Set_RGB_LED(uint8_t red, uint8_t green, uint8_t blue)
{
    // Initialize cmd_buffer with zeros
    uint8_t cmd_buffer[5] = {0};

    // Write the command to set the PMOD JSTK2's RGB LED to the first index of cmd_buffer
    // Note: The RGB LED is capable of 24-bit color
    cmd_buffer[0] = JSTK2_CMD_SET_RGB_LED;

    // Write the 8-bit "red" portion of the RGB color
    cmd_buffer[1] = red;

    // Write the 8-bit "green" portion of the RGB color
    cmd_buffer[2] = green;

    // Write the 8-bit "blue" portion of the RGB color
    cmd_buffer[3] = blue;

    // Write the last byte as a dummy byte (PMOD JSTK2 ignores this byte)
    cmd_buffer[4] = JSTK2_DUMMY_BYTE;

    // Transmit 5 bytes of data to the PMOD JSTK2 (MOSI) and receive the packet (MISO)
    PMOD_JSTK2_Transfer_and_Receive_Packet(cmd_buffer, 5);
}

uint8_t PMOD_JSTK2_Get_X_Position()
{
    // Initialize cmd_buffer with zeros
    uint8_t cmd_buffer[7] = {0};

    // Write the command to get the XY position of the PMOD JSTK2 to the first index of cmd_buffer
    cmd_buffer[0] = JSTK2_CMD_GET_POSITION;

    // Transmit 7 bytes of data to the PMOD JSTK2 (MOSI) and receive the packet (MISO)
    PMOD_JSTK2_Transfer_and_Receive_Packet(cmd_buffer, 7);

    // Only return the 6th byte of the received data (which indicates the X position) from the PMOD JSTK2
    // From the reference manual: "The X position is transferred to the master following the byte containing the button state."
    return rx_buffer[5];
}

uint8_t PMOD_JSTK2_Get_Y_Position()
{
    // Initialize cmd_buffer with zeros
    uint8_t cmd_buffer[7] = {0};

    // Write the command to get the XY position of the PMOD JSTK2 to the first index of cmd_buffer
    cmd_buffer[0] = JSTK2_CMD_GET_POSITION;

    // Transmit 7 bytes of data to the PMOD JSTK2 (MOSI) and receive the packet (MISO)
    PMOD_JSTK2_Transfer_and_Receive_Packet(cmd_buffer, 7);

    // Only return the 7th byte of the received data (which indicates the Y position) from the PMOD JSTK2
    // From the reference manual: "The Y position is transferred to the master immediately following the X position."
    return rx_buffer[6];
}

void PMOD_JSTK2_Get_XY_Position()
{
    // Initialize cmd_buffer with zeros
    uint8_t cmd_buffer[7] = {0};

    // Write the command to get the XY position of the PMOD JSTK2 to the first index of cmd_buffer
    cmd_buffer[0] = JSTK2_CMD_GET_POSITION;

    // Transmit 7 bytes of data to the PMOD JSTK2 (MOSI) and receive the packet (MISO)
    PMOD_JSTK2_Transfer_and_Receive_Packet(cmd_buffer, 7);

    // Assign the 6th byte of the received data (which indicates the X position) from the PMOD JSTK2 to a global variable
    X_Position_8_Bit = rx_buffer[5];

    // Assign the 7th byte of the received data (which indicates the Y position) from the PMOD JSTK2 to a global variable
    Y_Position_8_Bit = rx_buffer[6];
}
