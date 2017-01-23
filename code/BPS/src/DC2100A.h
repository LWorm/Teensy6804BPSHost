/*
 Linear Technology DC2100A Demonstration Board.
 Reference Application File for Controlling the LTC3300-1 Battery Balancers through the LTC6804-2 Battery Monitor on the DC2100A PCB.
 All datasheet references in this file refer to Microchip Technology Inc. document 39964B.pdf.

 @verbatim
 The App FW identification and HW definition are contained in these files.  The I/O between the PIC18 and J21 is defined in this code.
 @endverbatim

 http://www.linear.com/solutions/5126

 REVISION HISTORY
 $Revision: 1395 $
 $Date: 2015-05-28 16:12:35 -0400 (Thu, 28 May 2015) $

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

#ifndef __DC2100A_H__
#define __DC2100A_H__

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#include "Typedefs.h"
#include "18F47J53.h"
#include "PIC18F47J53_registers.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Definitions specific to the DC2100A board
#define APP_FW_REV_MAJOR            0x01
#define APP_FW_REV_MINOR            0x02
#define APP_FW_REV_SUB_MINOR        0x04
#define APP_FW_REV_BETA             0x00
#define APP_FW_STRING_FORMAT        "xx.yy.zzbaa"     // xx = major revision, yy = minor revision, zz = sub-revision, aa = beta number, . = '.', b = 'b'
#define APP_FW_STRING_SIZE          (sizeof(APP_FW_STRING_FORMAT) - 1)
#define APP_FW_STRING_DEFAULT       "N/A        "

// IDSTRING that the board sends back to the host
#define DC2100A_IDSTRING            "DC2100A-A,LTC3300-1 demonstration board"
#define DC2100A_IDSTRING_SIZE       (sizeof(DC2100A_IDSTRING) - 1)
#define DC2100A_COMPANY_STRING      "Linear Technology Inc."
#define DC2100A_COMPANY_STRING_SIZE (sizeof(DC2100A_COMPANY_STRING) - 1)

#define DC2100A_MODEL_NUM_DEFAULT   "DC2100A-?"
#define DC2100A_MODEL_NUM_SIZE      (sizeof(DC2100A_MODEL_NUM_DEFAULT) - 1)  // The size of the model number string (non-unicode), no null terminator
#define DC2100A_CAP_DEMO_DEFAULT    '?'
#define DC2100A_CAP_DEMO_SIZE       1                                        // The size of the cap demo character
#define DC2100A_SERIAL_NUM_DEFAULT  "None             "
#define DC2100A_SERIAL_NUM_SIZE     (sizeof(DC2100A_SERIAL_NUM_DEFAULT) - 1) // The size of the serial number string (non-unicode), no null terminator

// DC2100A HW definition
#define DC2100A_OSCILLATOR_FREQUENCY    48000000                             // Oscillator frequency for DC2100A board
#define DC2100A_INSTRUCTION_CLOCK       (DC2100A_OSCILLATOR_FREQUENCY / 4)   // Instruction frequency derived from oscillator
#define DC2100A_MAX_BOARDS              8           // The maximum number of DC2100A boards that can be stacked together into one system.
                                                    // Note: the number of boards that can be stacked, is limited by the voltage rating on transformer T15.
#define DC2100A_NUM_CELLS               12          // The number of cells on one DC2100A board
#define DC2100A_NUM_MUXES               2           // The number of LTC1380 Muxes on a DC2100A board
#define DC2100A_NUM_TEMPS               12          // The number of thermistor inputs on one DC2100A board
#define DC2100A_NUM_LTC3300             2           // The number of LTC3300 Balancers on a DC2100A board
#define DC2100A_PIC_BOARD_NUM           0           // It makes a lot of things simpler if the board with the PIC always has the same address.

// DC2100A I/O definition
// JP21 definitions
#define DISCHARGER_OUT_PIN          LATA_LATA1   // Output to control discharger in SuperCap Demo System
#define CHARGER_OUT_PIN             LATA_LATA0   // Output to control charger in SuperCap Demo System
#define DEBUG0_OUT_PIN              LATA_LATA3   // Toggled in Task_Status() at same rate as Green LED D15
#define DEBUG1_OUT_PIN              LATE_LATE0   // Lowered in Task_Parser() as USB Communication is Processed
#define DEBUG2_OUT_PIN              LATA_LATA2   // Lowered in Task_Voltage() when voltage sampling is started.
#define DEBUG3_OUT_PIN              LATE_LATE2   // Lowered in Task_Temperature() when temperature sampling is started.
#define DEBUG4_OUT_PIN              LATD_LATD5   // Lowered in Task_Balancer() when balancer state machine is started.
#define DEBUG5_OUT_PIN              LATD_LATD7   // Lowered in Task_Error() when error detection is started.
#define PACK_CURRENT_AIN_PIN        LATE_LATE1   // Monitors one A/D input referenced between VDD and VSS.
#define DISCHARGER_IN_PIN           PORTD_RD4    // Input to indicate discharge mode.
#define CHARGER_IN_PIN              PORTD_RD6    // Input to indicate charge mode.
#define DISCHARGER_OUT_PIN_SETUP    {DISCHARGER_OUT_PIN = 1; TRISA_TRISA1 = 0;}
#define CHARGER_OUT_PIN_SETUP       {CHARGER_OUT_PIN = 0; TRISA_TRISA0 = 0;}
#define DEBUG0_OUT_PIN_SETUP        {DEBUG0_OUT_PIN = 1; TRISA_TRISA3 = 0;}
#define DEBUG1_OUT_PIN_SETUP        {DEBUG1_OUT_PIN = 1; TRISE_TRISE0 = 0;}
#define DEBUG2_OUT_PIN_SETUP        {DEBUG2_OUT_PIN = 1; TRISA_TRISA2 = 0;}
#define DEBUG3_OUT_PIN_SETUP        {DEBUG3_OUT_PIN = 1; TRISE_TRISE2 = 0;}
#define DEBUG4_OUT_PIN_SETUP        {DEBUG4_OUT_PIN = 1; TRISD_TRISD5 = 0;}
#define DEBUG5_OUT_PIN_SETUP        {DEBUG5_OUT_PIN = 1; TRISD_TRISD7 = 0;}
#define PACK_CURRENT_AIN_PIN_SETUP  {PACK_CURRENT_AIN_PIN = 1; TRISE_TRISE1 = 1;}
#define DISCHARGER_IN_PIN_SETUP     {DISCHARGER_IN_PIN = 1; TRISD_TRISD4 = 1;}
#define CHARGER_IN_PIN_SETUP        {CHARGER_IN_PIN = 1; TRISD_TRISD6 = 1;}

// D15 and D16 definitions
#define LED_STATE_PIN               LATD_LATD0
#define LED_COMM_PIN                LATD_LATD1
#define LED_STATE_PIN_SETUP         {LED_STATE_PIN = 0; TRISD_TRISD0 = 0;}
#define LED_COMM_PIN_SETUP          {LED_COMM_PIN = 0; TRISD_TRISD1 = 0;}

// Analog channel used for Pack Current Measurements
#define DC2100A_PACK_CURRENT_CHANNEL    6       // AN6, corresponds to PACK_CURRENT_AIN_PIN

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

#endif
