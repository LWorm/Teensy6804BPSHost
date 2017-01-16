/*
 Linear Technology DC2100A Demonstration Board.
 PIC18 Driver File for ADC Module.
 All datasheet references in this file refer to Microchip Technology Inc. document 39964B.pdf.

 @verbatim
 This code uses the PIC18 ADC in the 12bit mode.
 @endverbatim

 http://www.linear.com/solutions/5126

 REVISION HISTORY
 $Revision: 434 $
 $Date: 2014-06-16 10:32:37 -0400 (Mon, 16 Jun 2014) $

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
#include "PIC18FXXJ_ADC.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define PIC18FXXJ_ADC_TAD_MIN       (0.8e-6)    // Min A/D Clock Period from datasheet Table 31-32 as parameter number 130.
#define PIC18FXXJ_ADC_TAD_MAX       (12.5e-6)   // Max A/D Clock Period from datasheet Table 31-32 as parameter number 130.

#define PIC18FXXJ_ADC_TAD_DIVISOR   64         // Smallest divisor that satisfies limitations on Min/Max TAD
#if (PIC18FXXJ_ADC_TAD_DIVISOR != 0)
    #if ((1.0/(DC2100A_OSCILLATOR_FREQUENCY/PIC18FXXJ_ADC_TAD_DIVISOR) < PIC18FXXJ_ADC_TAD_MIN) || \
        (1.0/(DC2100A_OSCILLATOR_FREQUENCY/PIC18FXXJ_ADC_TAD_DIVISOR) > PIC18FXXJ_ADC_TAD_MAX))
    #error ADC Divisor outside of TAD limitations.
    #endif
#endif

#define PIC18FXXJ_ADC_NUM_TAD_TO_SAMPLE_TIME_US(num_tad)                                        \
        (int8)((1.0*num_tad*US_PER_S)/(DC2100A_OSCILLATOR_FREQUENCY/PIC18FXXJ_ADC_TAD_DIVISOR))

#define PIC18FXXJ_SAMPLE_TIME_US_TO_ACQT_TABLE_IDX_MAX (sizeof(pic18fxxj_sample_time_us_to_acqt_table)/sizeof(int8) - 1)

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// See “PIC18 ADC” worksheet in the file DC2100A_Design.xlsm for design details for this table.
const int8 pic18fxxj_sample_time_us_to_acqt_table[] = { PIC18FXXJ_ADC_NUM_TAD_TO_SAMPLE_TIME_US(0),
                                                        PIC18FXXJ_ADC_NUM_TAD_TO_SAMPLE_TIME_US(2),
                                                        PIC18FXXJ_ADC_NUM_TAD_TO_SAMPLE_TIME_US(4),
                                                        PIC18FXXJ_ADC_NUM_TAD_TO_SAMPLE_TIME_US(6),
                                                        PIC18FXXJ_ADC_NUM_TAD_TO_SAMPLE_TIME_US(8),
                                                        PIC18FXXJ_ADC_NUM_TAD_TO_SAMPLE_TIME_US(12),
                                                        PIC18FXXJ_ADC_NUM_TAD_TO_SAMPLE_TIME_US(16),
                                                        PIC18FXXJ_ADC_NUM_TAD_TO_SAMPLE_TIME_US(20)};

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
int8 pic18fxxj_lookup_acqt(int8 sample_time_us);

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// Configures PIC ADC Module specifically for measuring Pack Current in DC2100A.
// Note that Configuration register, set in bootloader/boot.asm, is where ADCSEL is set to 0 for 12 bit operation.
void PIC18FXXJ_ADC_Init(int8 sample_time_us)
{
    ADCON0_ADON = 0;    // Turn ADC off before configuring

    // Setup Timer used for Baud Rate according to datasheet section 14
    ADCON0_VCFG1 = 0;   // AVSS Voltage Reference
    ADCON0_VCFG0 = 0;   // AVDD Voltage Reference

    // Configure Analog Channel Used to measure the current
    ADCON0_CHS0 = BITVAL(DC2100A_PACK_CURRENT_CHANNEL,0);
    ADCON0_CHS1 = BITVAL(DC2100A_PACK_CURRENT_CHANNEL,1);
    ADCON0_CHS2 = BITVAL(DC2100A_PACK_CURRENT_CHANNEL,2);
    ADCON0_CHS3 = BITVAL(DC2100A_PACK_CURRENT_CHANNEL,3);

    ADCON1_ADFM = 1;    // Right Justify Results

    // Set AD Clock as per datasheet register 22-2
#if (PIC18FXXJ_ADC_TAD_DIVISOR == 64)
    ADCON1_ADCS2 = 1;
    ADCON1_ADCS1 = 1;
    ADCON1_ADCS0 = 0;
#elif (PIC18FXXJ_ADC_TAD_DIVISOR == 32)
    ADCON1_ADCS2 = 0;
    ADCON1_ADCS1 = 1;
    ADCON1_ADCS0 = 0;
#elif (PIC18FXXJ_ADC_TAD_DIVISOR == 16)
    ADCON1_ADCS2 = 1;
    ADCON1_ADCS1 = 0;
    ADCON1_ADCS0 = 1;
#elif (PIC18FXXJ_ADC_TAD_DIVISOR == 8)
    ADCON1_ADCS2 = 0;
    ADCON1_ADCS1 = 0;
    ADCON1_ADCS0 = 1;
#elif (PIC18FXXJ_ADC_TAD_DIVISOR == 4)
    ADCON1_ADCS2 = 1;
    ADCON1_ADCS1 = 0;
    ADCON1_ADCS0 = 0;
#elif (PIC18FXXJ_ADC_TAD_DIVISOR == 2)
    ADCON1_ADCS2 = 0;
    ADCON1_ADCS1 = 0;
    ADCON1_ADCS0 = 0;
#else
#warning Using FRC for ADC clock as invalid divisor specified.
    // FRC (clock derived from A/D RC OSCILLATOR
    ADCON1_ADCS2 = 0;
    ADCON1_ADCS1 = 1;
    ADCON1_ADCS0 = 1;
#endif

    PIC18FXXJ_ADC_Sample_Time_Set(sample_time_us);

// Configure the Analog Input to be an Analog Input
#if (DC2100A_PACK_CURRENT_CHANNEL < BITS_PER_BYTE)
    ANCON0 &= ~MASK(1, DC2100A_PACK_CURRENT_CHANNEL);
#else
     ANCON1 &= ~MASK(1, DC2100A_PACK_CURRENT_CHANNEL - BITS_PER_BYTE);
#endif

    ANCON1_VBGEN  = 1;  // Turn on band gap reference

    ADCON0_ADON = 1;    // Turn ADC on

}

// Sets the ADC Acquisition Time.
void PIC18FXXJ_ADC_Sample_Time_Set(int8 sample_time_us)
{
    int8 adon_set = ADCON0_ADON; // Store whether ADC was on or off
    int8 acqt = pic18fxxj_lookup_acqt(sample_time_us);

    ADCON0_ADON = 0;    // Turn ADC off before configuring

    // Change the sample time
    ADCON1_ACQT2 = BITVAL(acqt, 0);
    ADCON1_ACQT1 = BITVAL(acqt, 1);
    ADCON1_ACQT0 = BITVAL(acqt, 2);

    ADCON0_ADON = adon_set; // Restore ADC to on or off state

}

// Begins the PIC ADC conversion.
void PIC18FXXJ_ADC_Start(void)
{
    ADCON0_GO = 1;
}

// Reads the results of the ADC conversion.
// Returns FALSE if conversion was not complete.
BOOLEAN PIC18FXXJ_ADC_Read(int16* adc_result)
{
    int8 high_byte, low_byte;

    if(ADCON0_DONE)
    {
        return FALSE;
    }

    // Get the ADC value.
    low_byte = ADRESL;
    high_byte = ADRESH;

    *adc_result = ((int16) high_byte << 8) + low_byte;
    return TRUE;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

// binary search table of acquisition times and return closest ACQT value that results in the closest yet larger than the desired sample time.
int8 pic18fxxj_lookup_acqt(int8 sample_time_us)
{
    unsigned int8 idx, idx_min, idx_max;
    unsigned int8 sample_time_test;

    if(sample_time_us <= pic18fxxj_sample_time_us_to_acqt_table[0])
    {
        // Sample time is too low for the table, return min value.
        idx =  0;
    }
    else if(sample_time_us >= pic18fxxj_sample_time_us_to_acqt_table[PIC18FXXJ_SAMPLE_TIME_US_TO_ACQT_TABLE_IDX_MAX])
    {
        // Sample time is too high for the table, return max value.
        idx = PIC18FXXJ_SAMPLE_TIME_US_TO_ACQT_TABLE_IDX_MAX;
    }
    else
    {
        idx_min = 0;
        idx_max = PIC18FXXJ_SAMPLE_TIME_US_TO_ACQT_TABLE_IDX_MAX;
        idx = PIC18FXXJ_SAMPLE_TIME_US_TO_ACQT_TABLE_IDX_MAX/2;

        // Sample time is in the table.  Find the closest value that's closest yet larger.
        while (idx_min < idx_max)
        {
            sample_time_test = pic18fxxj_sample_time_us_to_acqt_table[idx];

            if(sample_time_us == sample_time_test)
            {
                // Sample time is exactly value in the table, stop searching.
                break;
            }
            else if(sample_time_us > sample_time_test)
            {
                // If sample time is higher than tested table value, adjust the min
                idx_min = idx + 1;
            }
            else
            {
                // If sample time is lower than tested table value, adjust the max
                idx_max = idx;
            }

            idx = (idx_max + idx_min) >> 1;
        }
    }

    return idx;
}
