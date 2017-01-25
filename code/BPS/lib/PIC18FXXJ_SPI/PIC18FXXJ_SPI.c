/*
 Linear Technology DC2100A Demonstration Board.
 PIC18 Driver File for SPI port.
 All datasheet references in this file refer to Microchip Technology Inc. document 39964B.pdf.

 @verbatim
 This code provides an interface to the SPI port on the PIC18 used to communicate with the LTC6804-2.
 DMA is used so that the other software modules can perform other tasks (such as CRC calculation) as
 bytes as sent to and received from the SPI bus.  The SPI driver can be configured to different baud
 rates, so that communication for each IC can occur at its maximum rate.
 @endverbatim

 http://www.linear.com/solutions/5126

 REVISION HISTORY
 $Revision: 750 $
 $Date: 2014-09-17 19:27:28 -0400 (Wed, 17 Sep 2014) $

 Copyright (c) 2013, Linear Technology Corp.(LTC)
 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

 The views and conclusions contained in the software and documentation are those
 of the authors and should not be interpreted as representing official policies,
 either expressed or implied, of Linear Technology Corp.

*/

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"
#include "DC2100A.h"
#include "PIC18F47J53_registers.h"
#include "PIC18FXXJ_SPI.h"

#ifndef __PCH__
#error This file was built for PCH compiler from Custom Computer Services (CCS).
#error www.ccsinfo.com
#endif

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define SPI_BAUD_KHZ_TO_PR2_TABLE_SIZE (sizeof(spi_baud_khz_to_pr2_table)/sizeof(SPI_BAUD_KHZ_TO_PR2_TABLE_TYPE))

typedef struct
{
    int16 baud_khz;     // baud rate in kHz, min value of 100kHz
    int8 pr2_value;     // value of PIC18 PR2 that achieves baud rate without going over.
} SPI_BAUD_KHZ_TO_PR2_TABLE_TYPE;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#if DC2100A_INSTRUCTION_CLOCK != 12000000
#error This code module is designed for an PIC18 instruction clock of 12MHz (FCY) = 12MHz
#endif
// See “PIC18 SPI” worksheet in the file DC2100A_Design.xlsm for design details for this table.
const SPI_BAUD_KHZ_TO_PR2_TABLE_TYPE spi_baud_khz_to_pr2_table[]= { 100, 59,
                                                                    200, 29,
                                                                    353, 16,
                                                                    500, 11,
                                                                    600, 9,
                                                                    750, 7,
                                                                    857, 6,
                                                                    1000, 5};

int8 spi_dummy_tx; // Cosmetic only, byte is only necessary to ensure MOSI is low while reading from SPI
int8 spi_baud_khz_to_pr2_table_idx; // Memory of last selected index, to make selection of baud rates as quick as possible when they aren't changing.

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
int8 spi_lookup_pr2(int16 baud_khz);

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Initializes SPI pins, SPI module, and timer used for SPI baud rate generator
void PIC18FXXJ_SPI_Init(int16 baud_khz, PIC18FXXJ_SPI_MODE_TYPE mode)
{

    spi_baud_khz_to_pr2_table_idx = 0;

    SSP2CON1_SSPEN = 0; // Turn off SPI port, so that it can be reconfigured after init at power-up.

    // Init I/O according to datasheet section 10
    LATA_LATA5 = 1; // CS level start high
    LATB_LATB4 = 1; // SCK level start high
    LATC_LATC7 = 1; // SDO level start high
    TRISA_TRISA5 = 0;   // CS output
    TRISB_TRISB4 = 0;   // SCK output
    TRISB_TRISB5 = 1;   // SDI input
    TRISC_TRISC7 = 0;   // SDO output
    RPINR21 = 8;     // Set SDI2 as input RP8 (RB5)
    RPOR7 = 11;      // Set SCK2 as output RP7 (RB4)
    RPOR18 = 10;     // Set SDO2 as output RP18 (RC7)

    // Setup SPI according to datasheet section 20
    // todo - this is specific to the way the 6820 is configured on the DC2100A
    setup_spi2(SPI_MASTER | SPI_CLK_T2); // CCS compiler SPI setup macro
    SSP2CON1_CKP = mode.CKP;
    SSP2STAT_CKE = mode.CKE;
    SSP2STAT_SMP = mode.SMP;

    // Setup Timer used for Baud Rate according to datasheet section 14
    T2CON_TMR2ON = 0;
    T2CON_T2CKPS0 = 0;
    T2CON_T2CKPS1 = 0;
    T2CON_T2OUTPS0 = 0;
    T2CON_T2OUTPS1 = 0;
    T2CON_T2OUTPS2 = 0;
    T2CON_T2OUTPS3 = 0;

    // DMA is initialized according to datasheet section 20-3
    DMACON1_DMAEN = 0;          // Start with DMA disabled.
    DMACON1_TXINC = 1;          // Allow Transmit Buffer Address to Automatically Increment.
    DMACON1_RXINC = 1;          // Allow Receive Buffer Address to Automatically Increment.
    DMACON1_DUPLEX0 = 1;        // Start in half duplex mode, transmit.
    DMACON1_DUPLEX1 = 0;        //
    DMACON1_DLYINTEN = 0;       // Disable the Interrupt.
    DMACON2 = 0;                // No delay between bytes or watermark interrupt
    DMABCH = 0;                 // Set up high byte to be 0, as more than 256 bytes will never be transferred.

    spi_dummy_tx = 0x00;        // Set low so that logic analyzer pictures will be pretty during read

    // Set the baud rate, which will also start the timer and SPI module
    PIC18FXXJ_SPI_Set_Baud(baud_khz);
}

// Initializes SPI baud rate and starts the SPI module
// This is called regularly during code operation, as several different baud rates are used by the peripheral ICs on DC2100A.
void PIC18FXXJ_SPI_Set_Baud(int16 baud_khz)
{
    // Turn off SPI and Timer, so that it can they can reconfigured after init at power-up.
    SSP2CON1_SSPEN = 0;
    T2CON_TMR2ON = 0;

    TMR2 = 0;
    PR2 = spi_lookup_pr2(baud_khz);

    // Start the SPI Module
    T2CON_TMR2ON = 1;
    SSP2CON1_SSPEN = 1;
}

// Checks if the current SPI transmission or reception is complete.
inline BOOLEAN PIC18FXXJ_SPI_Buffer_Done(void)
{
    if(DMACON1_DMAEN)
    {
        return FALSE;
    }
    else
    {
        return TRUE;
    }
}

// Begins the transmission of number of bytes to the SPI via DMA.
// Does not wait for transmission to complete.
inline void PIC18FXXJ_SPI_Buffer_Send_Start(int8* write_buffer, int8 num_bytes)
{
    // If not currently sending anything, start the SPI transmission
    while(PIC18FXXJ_SPI_Buffer_Done() == FALSE);

    TXADDRH = UPPER_BYTE(write_buffer);
    TXADDRL = LOWER_BYTE(write_buffer);
    DMABCL = num_bytes - 1;

    DMACON1_DUPLEX0 = 1;        // Use half duplex mode, transmit
    DMACON1_DUPLEX1 = 0;        //
    DMACON1_TXINC = 1;          // Allow Transmit Buffer Address to Automatically Increment.
    DMACON1_DMAEN = 1;          // Start the transfer

    return;
}

// Begins the reception of a number of bytes to the SPI via DMA.
// Does not wait for reception to complete.
inline void PIC18FXXJ_SPI_Buffer_Receive_Start(int8* read_buffer, int8 num_bytes)
{
    // If not currently receiving anything, start the SPI reception
    while(PIC18FXXJ_SPI_Buffer_Done() == FALSE);

    RXADDRH = UPPER_BYTE(read_buffer);
    RXADDRL = LOWER_BYTE(read_buffer);
    DMABCL = num_bytes - 1;

    DMACON1_DUPLEX0 = 0;                    // Use full duplex mode, just to ensure the output of transmit line
    DMACON1_DUPLEX1 = 1;
    DMACON1_TXINC = 0;                      // Do not allow Transmit Buffer Address to Automatically Increment.
    TXADDRH = UPPER_BYTE(&spi_dummy_tx);    // Set up to transmit 0x00 during reception.
    TXADDRL = LOWER_BYTE(&spi_dummy_tx);
    DMACON1_DMAEN = 1;                      // Start the reception

    return;
}

// Returns the number of bytes left to be sent out the SPI port.
inline int8 PIC18FXXJ_SPI_Buffer_Receive_Bytes_Available(int8* write_buffer_ptr)
{
    int8* read_buffer_ptr;

    // Get the current DMA read address while protecting against overflow.
    // todo - if we dedicated a buffer in low memory (or at least in an area where we could be sure high byte couldn't overflow),
    //        this function simplifies quite a bit.
    do
    {
        read_buffer_ptr = ((int8*) RXADDRH << 8) + RXADDRL;
    } while((read_buffer_ptr >> 8) != RXADDRH);

    return (int8)(read_buffer_ptr - write_buffer_ptr);
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// binary search table of baud rates, starting with the current value, and stop when larger than the desired baud rate is found.
int8 spi_lookup_pr2(int16 baud_khz)
{
    int16 test_baud_khz;
    int8 idx_min = 0;
    int8 idx_max = SPI_BAUD_KHZ_TO_PR2_TABLE_SIZE;

    while (idx_min < idx_max)
    {
        test_baud_khz = spi_baud_khz_to_pr2_table[spi_baud_khz_to_pr2_table_idx].baud_khz;

        if(test_baud_khz == baud_khz)
        {
            return spi_baud_khz_to_pr2_table[spi_baud_khz_to_pr2_table_idx].pr2_value;;
        }
        else if(test_baud_khz > baud_khz)
        {
            idx_max = spi_baud_khz_to_pr2_table_idx;
        }
        else
        {
            idx_min = spi_baud_khz_to_pr2_table_idx + 1;
        }

        spi_baud_khz_to_pr2_table_idx = (idx_max + idx_min) >> 1;
    }

    // value is one below the value in the table, unless baud rate was requested that's off of the table
    if(spi_baud_khz_to_pr2_table_idx != 0)
    {
        spi_baud_khz_to_pr2_table_idx -= 1;
    }

    return spi_baud_khz_to_pr2_table[spi_baud_khz_to_pr2_table_idx].pr2_value;
}
