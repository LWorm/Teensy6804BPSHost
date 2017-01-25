/*
 Linear Technology DC2100A Demonstration Board.
 Reference Application File for Controlling the LTC3300-1 Battery Balancers through the LTC6804-2 Battery Monitor on the DC2100A PCB.
 All datasheet references in this file refer to Microchip Technology Inc. document 39964B.pdf.

 @verbatim
 This code schedules contains the main() function for the App FW.
 The tasks are initialized and scheduled for periodic execution before starting the CCS RTOS.
 Note that much of the USB Parser is located in these files, as the CCS compiler requires code to be
 located in the file with main() in order to use the rtos_await() function.  It is also a convention of the
 CCS compiler to have one file included in a project, with the called code modules included as .c files.
 @endverbatim

 http://www.linear.com/solutions/5126

 REVISION HISTORY
 $Revision: 771 $
 $Date: 2014-09-26 10:42:21 -0400 (Fri, 26 Sep 2014) $

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




// Define the hardware.  CCS must have these included first.
#include "Typedefs.h"
#include "18F47J53.h"
#include "PIC18F47J53_registers.h"
#include "DC2100A.h"

// Settings for the CCS compiler
//#DEVICE ANSI      // todo - our code all seems to be ANSI compliant, but the USB driver doesn't work if this is set.
//#DEVICE CONST=ROM // If set for ANSI, specify for const tables to be in ROM instead of RAM
#DEVICE HIGH_INTS=TRUE
#ifdef __DEBUG
#DEVICE ICD=TRUE
#endif

// Fuse configuration for Debug and Release versions
#FUSES HSPLL, NOWDT, NOIESO, NOFCMEN, NOPROTECT, NODSWDT, PLL3, NOCPUDIV, NOXINST, STVREN, ADC12, WPFP=01, WPCFG, NOWPDIS, WPDIS
#ifdef __DEBUG
#FUSES DEBUG
#else
#FUSES NODEBUG
#endif

// Tell CCS to locate their reset vector and interrupt vectors in the indirect locations used by the bootloader.
#build (reset=APP_RESET_VECTOR, interrupt=APP_HIGH_INTERRUPT_VECTOR)

// Manually create reset vector and interrupt vectors in application that are normally in the bootloader (so that application can be run independently).
#ROM BOOT_RESET_VECTOR = {
 0xEF00+((APP_RESET_VECTOR >> 1) & 0xFF),
 0xF000+(APP_RESET_VECTOR >> 9),
 0x0000,
 0x0000
}

#ROM BOOT_HIGH_INTERRUPT_VECTOR = {
 0xEF00+((APP_HIGH_INTERRUPT_VECTOR >> 1) & 0xFF),
 0xF000+(APP_HIGH_INTERRUPT_VECTOR >> 9),
 0x0000,
 0x0000,
 0x0000,
 0x0000,
 0x0000,
 0x0000
}

#ROM BOOT_LOW_INTERRUPT_VECTOR = {
 0xEF00+((APP_LOW_INTERRUPT_VECTOR >> 1) & 0xFF),
 0xF000+(APP_LOW_INTERRUPT_VECTOR >> 9),
 0x0000,
 0x0000
}

// Tell CCS to reserve flash memory for the bootloader.
#ORG BOOT_LOW_INTERRUPT_VECTOR + 8, BOOT_ROM_END {}

// Create key at end of memory so that application presence can be detected by the bootloader.
#ROM APP_ROM_KEY_LOCATION = {
 APP_ROM_KEY_BYTE_0 + (APP_ROM_KEY_BYTE_1 << 8),
 APP_ROM_KEY_BYTE_2 + (APP_ROM_KEY_BYTE_3 << 8)
}

// Put FW in fixed place in memory so that bootloader can find it if launched directly through the jumpers.
#ROM APP_FW_REV_LOCATION = {
 APP_FW_REV_MAJOR + (APP_FW_REV_MINOR << 8),
 APP_FW_REV_SUB_MINOR + (APP_FW_REV_BETA << 8)
}

// Tell the compiler what the clock speed is for their delay function (used by the USB driver)
#USE delay(clock=DC2100A_OSCILLATOR_FREQUENCY)

// Use the CCS Cyclic Executive RTOS with timer 1
// todo - some way to automatically set minor cycle to the smallest task time would be preferable.
#USE RTOS(timer=1, minor_cycle=25ms)

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Includes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// PIC and Compiler Driver Modules
// Note - There is order dependence here, where "pic18_usb.h" and "USB_Descriptors.c" must be included before "USB.c".
#include "pic18_usb.h"                                  // Microchip PIC18Fxx5x Hardware layer for CCS's PIC USB driver
#include "USB_Descriptors.c"                            // Descriptors and USB configuration
#include "USB.c"                                        // CCS's USB driver handles USB setup tokens and get descriptor reports
#include "PIC18FXXJ_SPI.c"
#include "PIC18FXXJ_Timer.c"
#include "PIC18FXXJ_ADC.c"

// IC interface Modules
#include "LTC6804-2.c"
#include "LTC3300-1.c"
#include "LTC1380.c"
#include "24AA64.c"

// High Level Modules
#include "System.c"
#include "Eeprom.c"
#include "Voltage.c"
#include "Temperature.c"
#include "SOC.c"
#include "Balancer.c"
#include "Pack_Current.c"
#include "USB_Parser.c"
#include "Error.c"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

 // These bits allow tasks to be skipped while data is being shipped out USB.
 // This allows all of the data samples sent out USB to be from the same timestamp, at the expense of the regular sample period.
 struct {
    int voltage : 1;
    int temperature : 1;
} dc2100a_task_skip;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Define the tasks for the CCS RTOS
// todo - Fill in more realistic estimates for the worst case execution time
#task(rate=STRINGIZE(BALANCER_TASK_RATE)ms, max=10ms)
void Task_Balancer(void);                       // Controls the balancers in the DC2100A system

#task(rate=STRINGIZE(VOLTAGE_TASK_RATE)ms, max=20ms)
void Task_Voltage(void);                        // Monitors the voltages in the DC2100A system

#task(rate=STRINGIZE(PACK_CURRENT_TASK_RATE)ms, max=20ms)
void Task_Pack_Current(void);                   // Monitors the charger, discharger, and pack current in the DC2100A system

#task(rate=STRINGIZE(TEMPERATURE_TASK_RATE)ms, max=10ms)
void Task_Temperature(void);                    // Monitors the temperatures in the DC2100A system

#task(rate=STRINGIZE(USB_TASK_RATE)ms,max=1ms)
void Task_Get_USB(void);                        // Receives bytes from the USB bulk endpoint.  Must be its own task to receive bytes while Task_Parser is pending.

#task(rate=STRINGIZE(USB_TASK_RATE)ms,max=10ms)
void Task_Parser(void);                         // Parses bytes received from the USB bulk endpoints for commands, and sends the responses to the commands

#task(rate=STRINGIZE(STATUS_TASK_RATE)ms,max=1ms)
void Task_Status(void);                         // Indicates the status of the DC2100A system

#task(rate=STRINGIZE(DETECT_TASK_RATE)ms, max=1ms)
void Task_Detect(void);                         // Detect new boards in the DC2100A system

#task(rate=STRINGIZE(ERROR_TASK_RATE)ms, max=1ms)
void Task_Error(void);                          // Detect errors in the DC2100A system

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void main(void)
{

    // Init PIC oscillator PLL
    {
        unsigned int pll_startup_counter = 600;  // todo - where does 600 come from?
        OSCTUNE_PLLEN = 1;  //Enable the PLL and wait 2+ms until the PLL locks before enabling USB module
        while (pll_startup_counter--) ;
    }

    // Init PIC hardware
    DISCHARGER_OUT_PIN_SETUP;       // Init I/O
    CHARGER_OUT_PIN_SETUP;
    DEBUG0_OUT_PIN_SETUP;
    DEBUG1_OUT_PIN_SETUP;
    DEBUG2_OUT_PIN_SETUP;
    DEBUG3_OUT_PIN_SETUP;
    DEBUG4_OUT_PIN_SETUP;
    DEBUG5_OUT_PIN_SETUP;
    PACK_CURRENT_AIN_PIN_SETUP;
    DISCHARGER_IN_PIN_SETUP;
    CHARGER_IN_PIN_SETUP;
    LED_STATE_PIN_SETUP;
    LED_COMM_PIN_SETUP;
    PIC18FXXJ_SPI_Init(LTC6804_BAUD_RATE, (PIC18FXXJ_SPI_MODE_TYPE) 0b010); // Init SPI for the LTC6820 clock polarity/phase, and the 6804 baud rate.
    PIC18FXXJ_Timer_Init();

    // Init the Error handler first, so that HW issues upon startup can be recorded
    Error_Init();

    // Initialize the Hardware Modules
    LTC6804_Init();                                 // Init after SPI, as LTC6804 commincation is through SPI
    LTC3300_Init();                                 // Init after LTC6804, as LTC3300 communication passes through the LTC6804

    // Initialize the Higher Level Functions Necessary to Identify System Via USB
    Eeprom_Init();                                  // Init after LTC6804, as EEPROM communication passes through the LTC6804
    System_Init();                                  // Init after EEPROM, as system config info is contained in the EEPROM
    USB_Init();                                     // Init after EEPROM and System, as USB enumeration strings depend upon system config info contained in the EEPROM

    // Initialize the Other Higher Level Functions
    Temperature_Init();                             // Init after LTC6804
    Balancer_Init();                                // Init after LTC6804 and LTC3300
    Voltage_Init();                                 // Init after LTC6804
    Pack_Current_Init();                            // Init after EEPROM and System
    USB_Parser_Init();                              // Initialize Last

    // Allow all tasks to run initially
    dc2100a_task_skip.temperature = 1;
    dc2100a_task_skip.voltage = 1;

    while (TRUE)
    {
        rtos_run();                                     // Start the RTOS
    }
}

// Detects DC2100A-D boards attached to DC2100A-C, manages the DC2100A system state, and indicates via LED.
void Task_Status(void)
{
    System_Status_Task();
}

// Reads Cell Voltages and Monitors for UV and OV
void Task_Voltage(void)
{
    if(System_State == SYSTEM_STATE_AWAKE)
    {

        DEBUG2_OUT_PIN = 0;
        if(dc2100a_task_skip.voltage == 1)
        {
            Voltage_Monitor_Task();
        }
        DEBUG2_OUT_PIN = 1;
    }

}

// Monitors the charger, discharger, and pack current in the DC2100A system
void Task_Pack_Current(void)
{
    if(System_State == SYSTEM_STATE_AWAKE)
    {
        DEBUG4_OUT_PIN = 0;
        Pack_Current_Monitor_Task();
        DEBUG4_OUT_PIN = 1;
    }
}

// Reads Temperatures
void Task_Temperature(void)
{
    if(System_State == SYSTEM_STATE_AWAKE)
    {
        DEBUG3_OUT_PIN = 0;
        if(dc2100a_task_skip.temperature == 1)
        {
            Temperature_Monitor_Task();
        }
        DEBUG3_OUT_PIN = 1;
    }
}

// Controls the Balancers
void Task_Balancer(void)
{
    if(System_State == SYSTEM_STATE_AWAKE)
    {
        //DEBUG4_OUT_PIN = 0;
        Balancer_Control_Task();
        //DEBUG4_OUT_PIN = 1;
    }
}

// Detect new boards as they wake up in the system.  Necessary when the PIC is powered up before the cells so that they can not all be detected at init time.
void Task_Detect(void)
{
    if(System_State == SYSTEM_STATE_AWAKE)
    {
        System_Detect_Task();
    }
}

// Detect errors in the DC2100A system and report through USB.
void Task_Error(void)
{
    //DEBUG5_OUT_PIN = 0;
    Error_Monitor_Task();
    //DEBUG5_OUT_PIN = 1;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Receives bytes from the USB bulk endpoint
void Task_Get_USB(void)
{
    if(usb_enumerated() == FALSE)
    {
        // Do not try to use the USB peripheral for
        // sending and receiving packets until you
        // are enumerated.
        return;
    }

    // Receive bytes from the USB if they are available
    USB_Parser_Check_Incoming();
}

// Note - this must be its own task, in order to use rtos await.
// Parse commands and invoke lower level routines.
void Task_Parser(void)
{

    if((usb_enumerated() == FALSE))
    {
        // Do not try to use the USB peripheral for
        // sending and receiving packets until you
        // are enumerated.
        return;
    }

    // Prevent a new response from being started before the old one has been sent
    if (usb_parser_transmit_queue.Length != 0)
    {
    }
    // If async responses are pending, process them now
    else if (usb_parser_async_queue.Length != 0)
    {
        // Send asyncs if any are pending
        USB_Parser_Async_Response();
    }
    // Otherwise, process responses for the incoming USB
    else if (usb_parser_receive_queue.Length != 0)
    {
        LED_COMM_PIN = 1;
        DEBUG1_OUT_PIN = 0;

        // Get the command to process.
        usb_parser_command = USB_Parser_Buffer_Get_Char(&usb_parser_receive_queue);

        // Parse the command
        switch (usb_parser_command)
        {
            default:                                                /* By default anything not specified is a no-op */
                printf(USB_Parser_Buffer_Put_Char, USB_PARSER_DEFAULT_STRING);
                break;

            case USB_PARSER_HELLO_COMMAND:                          /*  Reply with Hello String.  Mostly useful for testing. */
                USB_Parser_Hello_Response();
                break;

            case USB_PARSER_IDSTRING_COMMAND:                       /*  Read controller ID and firmware rev, this supports legacy functions */
            case USB_PARSER_IDSTRING_COMMAND_2:
                USB_Parser_IDString_Response();
                break;

            case USB_PARSER_BOOT_MODE_COMMAND:                      /* Enter Bootload Mode */
                rtos_await(usb_parser_receive_queue.Length >= SYSTEM_BOOTLOAD_KEY_SIZE);
                USB_Parser_Bootload_Command();
                break;

            case USB_PARSER_ERROR_COMMAND:                          /* Read Error Data*/
                // Get the board number
                USB_Parser_Error_Data_Response();
                break;

            case USB_PARSER_SYSTEM_COMMAND:                         /* Read System Data*/
                USB_Parser_System_Data_Response();
                break;

            case USB_PARSER_MFG_COMMAND:                            /* Read/Write Board Manufacturing Data */
                // Get the read/write character and board number
                rtos_await(usb_parser_receive_queue.Length >= 3);
                usb_parser_subcommand = USB_Parser_Buffer_Get_Char(&usb_parser_receive_queue);
                usb_parser_board_num = getUSBint8_ASCII();

                // If a write, handle the write
                if(usb_parser_subcommand == 'W')
                {
                    // Get the mfg data
                    rtos_await(usb_parser_receive_queue.Length >= (DC2100A_MODEL_NUM_SIZE + DC2100A_CAP_DEMO_SIZE + DC2100A_SERIAL_NUM_SIZE));
                    USB_Parser_Board_Mfg_Data_Command(usb_parser_board_num);
                }
                // If a write, handle the write
                else if(usb_parser_subcommand == 'D')
                {
                    rtos_await(usb_parser_receive_queue.Length >= (EEPROM_RESET_KEY_SIZE + SYSTEM_RESET_KEY_SIZE));
                    USB_Parser_Board_Mfg_Data_Reset(usb_parser_board_num);
                }

                // Always send a response, whether it's a read or a write
                USB_Parser_Board_Mfg_Data_Response(usb_parser_board_num);
                break;

            case USB_PARSER_UVOV_COMMAND:                           /* Read Board Over-Voltage and Under-Voltage Conditions */
                // Get the board number
                rtos_await(usb_parser_receive_queue.Length >= (sizeof(usb_parser_board_num)*ASCII_PER_BYTE));
                usb_parser_board_num = getUSBint8_ASCII();

                USB_Parser_Board_Vov_Vuv_Response(usb_parser_board_num);
                break;

            case USB_PARSER_VOLTAGE_COMMAND:                        /* Read Board Voltage Data */
                dc2100a_task_skip.voltage = 0;
                for(usb_parser_board_num = 0; usb_parser_board_num < System_Num_Boards; usb_parser_board_num++)
                {
                    USB_Parser_Board_Voltage_Data_Response(usb_parser_board_num);

                    // Wait for USB driver to be ready for data, and then send out one board at time
                    rtos_await(usb_tbe(1));
                    sendUSBmessage();
                }
                dc2100a_task_skip.voltage = 1;
                break;

            case USB_PARSER_TEMPERATURE_COMMAND:                    /* Read Board Temperature Data */
                dc2100a_task_skip.temperature = 0;
                for(usb_parser_board_num = 0; usb_parser_board_num < System_Num_Boards; usb_parser_board_num++)
                {
                    USB_Parser_Board_Temperature_Data_Response(usb_parser_board_num);

                    // Wait for USB driver to be ready for data, and then send out one board at time
                    rtos_await(usb_tbe(1));
                    sendUSBmessage();
                }
                dc2100a_task_skip.temperature = 1;
                break;

            case USB_PARSER_TEMP_ADC_COMMAND:                       /* Read Board Temperature Adc Values */
                // Get the board number
                rtos_await(usb_parser_receive_queue.Length >= (sizeof(usb_parser_board_num)*ASCII_PER_BYTE));
                usb_parser_board_num = getUSBint8_ASCII();

                USB_Parser_Board_Temperature_Adc_Value_Response(usb_parser_board_num);
                break;

            case USB_PARSER_PACK_CURRENT_COMMAND:
                USB_Parser_Pack_Current_Data_Response();
                break;

            case USB_PARSER_LTC3300_COMMAND:                        /* LTC3300 Raw Write via LTC6804 */
                // Get the read/write character, board number, and number of bytes in command.
                rtos_await(usb_parser_receive_queue.Length >= 4);
                usb_parser_subcommand = USB_Parser_Buffer_Get_Char(&usb_parser_receive_queue);
                usb_parser_board_num = getUSBint8_ASCII();
                usb_parser_num_bytes = ASCIItonybble(USB_Parser_Buffer_Get_Char(&usb_parser_receive_queue));

                // If a write, handle the write
                if(usb_parser_subcommand == 'W')
                {
                    // Wait for the bytes that need to be sent.
                    rtos_await(usb_parser_receive_queue.Length >= (usb_parser_num_bytes*ASCII_PER_BYTE));
                    USB_Parser_Board_LTC3300_Write_Command(usb_parser_board_num, usb_parser_num_bytes);
                }
                else
                {
                    // Wait for the one bytes that needs to be sent, before receiving num bytes.
                    rtos_await(usb_parser_receive_queue.Length >= (1*ASCII_PER_BYTE));
                    USB_Parser_Board_LTC3300_Read_Response(usb_parser_board_num, usb_parser_num_bytes);
                }
                break;

            case USB_PARSER_TIMED_BALANCE_COMMAND:                  /* Board Timed Balance */
                // Get the usb_parser_subcommand and board number
                // Get the read/write character and board number
                rtos_await(usb_parser_receive_queue.Length >= 3);
                usb_parser_subcommand = USB_Parser_Buffer_Get_Char(&usb_parser_receive_queue);
                usb_parser_board_num = getUSBint8_ASCII();

                // Write a balancing sequence for one board.
                if(usb_parser_subcommand == 'W')
                {
                    rtos_await(usb_parser_receive_queue.Length >= (sizeof(BALANCER_ACTIVE_STATE_TYPE)*ASCII_PER_BYTE));
                    Balancer_Set();
                    USB_Parser_Board_Active_Balancer_Command(usb_parser_board_num);
                }
                // Begin the Balancing.
                else if(usb_parser_subcommand == 'B')
                {
                    Balancer_Start();
                }
                // Suspend the Balancing.
                else if(usb_parser_subcommand == 'S')
                {
                    Balancer_Suspend();
                }
                // End the Balancing.
                else if(usb_parser_subcommand == 'E')
                {
                    Balancer_Stop();
                }

                // Always send a response, whether it's a read or a write
                USB_Parser_Board_Active_Balancer_Response(usb_parser_board_num);
                break;

            case USB_PARSER_PASSIVE_BALANCE_COMMAND:                /* Board Passive Balancers */
                rtos_await(usb_parser_receive_queue.Length >= 3);
                usb_parser_subcommand = USB_Parser_Buffer_Get_Char(&usb_parser_receive_queue);
                usb_parser_board_num = getUSBint8_ASCII();

                // If a write, handle the write
                if(usb_parser_subcommand == 'W')
                {
                    rtos_await(usb_parser_receive_queue.Length >= (sizeof(BALANCER_PASSIVE_STATE_TYPE)*ASCII_PER_BYTE));
                    USB_Parser_Board_Passive_Balancer_Command(usb_parser_board_num);
                }

                // Always send a response, whether it's a read or a write
                USB_Parser_Board_Passive_Balancer_Response(usb_parser_board_num);
                break;

            case USB_PARSER_CELL_PRESENT_COMMAND:                   /* Board Cell Present */
                rtos_await(usb_parser_receive_queue.Length >= 3);
                usb_parser_subcommand = USB_Parser_Buffer_Get_Char(&usb_parser_receive_queue);
                usb_parser_board_num = getUSBint8_ASCII();

                // If a write, handle the write
                if(usb_parser_subcommand == 'W')
                {
                    rtos_await(usb_parser_receive_queue.Length >= (sizeof(VOLTAGE_CELL_PRESENT_TYPE)*ASCII_PER_BYTE));
                    USB_Parser_Board_Cell_Present_Command(usb_parser_board_num);
                }

                // Always send a response, whether it's a read or a write
                USB_Parser_Board_Cell_Present_Response(usb_parser_board_num);
                break;

            case USB_PARSER_EEPROM_COMMAND:                         /* Read/Write/Default EEPROM */
                // Get the read/write/default character, board number, and EEPROM_data_id.
                rtos_await(usb_parser_receive_queue.Length >= 4);
                usb_parser_subcommand = USB_Parser_Buffer_Get_Char(&usb_parser_receive_queue);
                usb_parser_board_num = getUSBint8_ASCII();
                usb_parser_item_num = ASCIItonybble(USB_Parser_Buffer_Get_Char(&usb_parser_receive_queue));

                if(usb_parser_item_num >= USB_PARSER_EEPROM_NUM_ITEMS)
                {
                    break;
                }

                // If a writing/loading/defaulting, handle that before the response
                if((usb_parser_subcommand == 'L') || (usb_parser_subcommand == 'l'))
                {
                    USB_Parser_Board_EEPROM_Default_Load_Command(usb_parser_board_num, usb_parser_item_num, (usb_parser_subcommand == 'L') ? 0 : EEPROM_MFG_KEY);
                }
                else if((usb_parser_subcommand == 'D') || (usb_parser_subcommand == 'd'))
                {
                    USB_Parser_Board_EEPROM_Default_Save_Command(usb_parser_board_num, usb_parser_item_num, (usb_parser_subcommand == 'D') ? 0 : EEPROM_MFG_KEY);
                }
                else if((usb_parser_subcommand == 'W') || (usb_parser_subcommand == 'w'))
                {
                    usb_parser_num_bytes = usb_parser_eeprom_num_bytes[usb_parser_item_num];

                    // Wait for the one bytes that needs to be sent, before receiving num bytes.
                    rtos_await(usb_parser_receive_queue.Length >= usb_parser_num_bytes);
                    USB_Parser_Board_EEPROM_Write_Command(usb_parser_board_num, usb_parser_item_num, (usb_parser_subcommand == 'W') ? 0 : EEPROM_MFG_KEY);
                }

                USB_Parser_Board_EEPROM_Read_Response(usb_parser_board_num, usb_parser_item_num);
                break;

            case USB_PARSER_UVOV_THRESHOLDS_COMMAND:                /* Over and Under Voltage Thresholds */
                rtos_await(usb_parser_receive_queue.Length >= (2*sizeof(int16)*ASCII_PER_BYTE) );

                USB_Parser_System_UVOV_Command();
                USB_Parser_System_Data_Response();
                break;

            // todo - rename this command
            case USB_PARSER_CAP_DEMO_COMMAND:                       /* Charge/Discharge the Cap Board */
                rtos_await(0 != usb_parser_receive_queue.Length);
                switch (USB_Parser_Buffer_Get_Char(&usb_parser_receive_queue))
                {
                    case 'C':   //start charging
                        System_Cap_Demo.charging = 1;
                        System_Cap_Demo.discharging = 0;
                        USB_Parser_System_Data_Response();
                        break;
                    case 'N':   //suspend charging
                        System_Cap_Demo.charging = 0;
                        System_Cap_Demo.discharging = 0;
                        USB_Parser_System_Data_Response();
                        break;
                    case 'D':   //start discharging
                        System_Cap_Demo.charging = 0;
                        System_Cap_Demo.discharging = 1;
                        USB_Parser_System_Data_Response();
                        break;
                    case 'c':   //toggle charging
                        if(Pack_Current_IO.output_enabled)
                        {
                            Pack_Current_IO.charging_output = 1 - Pack_Current_IO.charging_output;
                        }
                        else
                        {
                            Pack_Current_IO.charging_output = 0;
                        }
                        USB_Parser_Pack_Current_Data_Response();
                        break;
                    case 'd':   //toggle discharging
                        if(Pack_Current_IO.output_enabled)
                        {
                            Pack_Current_IO.discharging_output = 1 - Pack_Current_IO.discharging_output;
                        }
                        else
                        {
                            Pack_Current_IO.discharging_output = 0;
                        }
                        USB_Parser_Pack_Current_Data_Response();
                        break;
                }
                break;

                case USB_PARSER_ALGORITHM_COMMAND:                  /* Timed Balance Incorporating Algorithm */
                    // Get the usb_parser_subcommand and board number
                    rtos_await(usb_parser_receive_queue.Length >= 1);
                    usb_parser_subcommand = USB_Parser_Buffer_Get_Char(&usb_parser_receive_queue);
                    usb_parser_board_num = DC2100A_PIC_BOARD_NUM;   //  Balance algorithm only implemented for 1 board with 12 cells.

                    // Write a balancing sequence for one board.
                    if(usb_parser_subcommand == 'W')
                    {
                        USB_Parser_Balancer_Algorithm_Command();
                    }

                    // Always send a response, whether it's a read or a write
                    USB_Parser_Board_Active_Balancer_Response(usb_parser_board_num);
                    break;
           }  //end of outer switch statement

    } // end of while statement

    LED_COMM_PIN = 0;
    DEBUG1_OUT_PIN = 1;

    USB_Parser_Check_Outgoing();

}
