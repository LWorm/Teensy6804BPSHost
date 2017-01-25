/*
 Linear Technology DC2100A Demonstration Board.
 PIC18 Driver File for Timer0.
 All datasheet references in this file refer to Microchip Technology Inc. document 39964B.pdf.

 @verbatim
 This code uses Timer0 of the PIC18 to provide time measurements and time stamps to the other code modules.
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
#include "PIC18FXXJ_Timer.h"

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define PIC18FXXJ_TIMER_PRESCALE    256                                 // T0PS<2:0> settings as per datasheet Register 12-1

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
unsigned int16 pic18fxxj_timer_last;
unsigned int32 pic18fxxj_timer_count;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void PIC18FXXJ_Timer_Init(void)
{
    // Setup timer according to datasheet section 12
    T0CON_TMR0ON = 0;

    // Set the prescaler for Timer0
#if (PIC18FXXJ_TIMER_PRESCALE == 256)
    T0CON_T0PS2 = 1;
    T0CON_T0PS1 = 1;
    T0CON_T0PS0 = 1;
#elif (PIC18FXXJ_TIMER_PRESCALE == 128)
    T0CON_T0PS2 = 1;
    T0CON_T0PS1 = 1;
    T0CON_T0PS0 = 0;
#elif (PIC18FXXJ_TIMER_PRESCALE == 64)
    T0CON_T0PS2 = 1;
    T0CON_T0PS1 = 0;
    T0CON_T0PS0 = 1;
#elif (PIC18FXXJ_TIMER_PRESCALE == 32)
    T0CON_T0PS2 = 1;
    T0CON_T0PS1 = 0;
    T0CON_T0PS0 = 0;
#elif (PIC18FXXJ_TIMER_PRESCALE == 16)
    T0CON_T0PS2 = 0;
    T0CON_T0PS1 = 1;
    T0CON_T0PS0 = 1;
#elif (PIC18FXXJ_TIMER_PRESCALE == 8)
    T0CON_T0PS2 = 0;
    T0CON_T0PS1 = 1;
    T0CON_T0PS0 = 0;
#elif (PIC18FXXJ_TIMER_PRESCALE == 4)
    T0CON_T0PS2 = 0;
    T0CON_T0PS1 = 0;
    T0CON_T0PS0 = 1;
#elif (PIC18FXXJ_TIMER_PRESCALE == 2)
    T0CON_T0PS2 = 0;
    T0CON_T0PS1 = 0;
    T0CON_T0PS0 = 0;
#endif

    T0CON_PSA = 0;          // timer prescaler is assigned.  Timer0 clock input comes from prescaler output.
    T0CON_T0SE = 0;         // increment on low-to-high transition on T0CKI pin
    T0CON_T0CS = 0;         // internal instruction cycle clock
    T0CON_T08BIT = 0;       // configure at 16 bit timer

    TMR0_H = 0;
    TMR0_L = 0;

    pic18fxxj_timer_last = 0;
    pic18fxxj_timer_count = 0;

    // Start the Timer
    T0CON_TMR0ON = 1;
}

// Update the 32 bit counter from the hw timer and return the updated value
unsigned int32 PIC18FXXJ_Timer_Update(void)
{
    int8 high_byte, low_byte;
    unsigned int16 timer_next;

    // Get the current value of the timer.  Must read low byte first to latch the high byte.  See Figure 12-2 of PIC datasheet for details.
    low_byte = TMR0_L;
    high_byte = TMR0_H;
    timer_next = ((int16) high_byte << 8) + low_byte;

    // Calculate the number of timer ticks since clock was last updated.
    pic18fxxj_timer_count += (timer_next - pic18fxxj_timer_last);

    pic18fxxj_timer_last = timer_next;

    return pic18fxxj_timer_count;
}

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

