/*
 Linear Technology DC2100A Demonstration Board.
 DC2100A Application File for calculating the SOC in a SuperCap Demo system and setting the LTC3300 to balance the charge.

 This code calculates the charge imbalance in the SuperCap Demo system used by the Linear Sales team.  It is very simple, using
 the relationship between charge and voltage on a capacitor:  delta Q = C * delta V. The charge imbalance is then passed to the
 Balancer module to calculate the optimal time for each balancer to be turned on for the SuperCap Demo system to balance.  See
 the section on the Balancer.c/.h code module for the details about how the charge imbalance is translated into balancer commands.

 http://www.linear.com/solutions/5126

 REVISION HISTORY
 $Revision: 748 $
 $Date: 2014-09-16 17:45:44 -0400 (Tue, 16 Sep 2014) $

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
#include "SOC.h"
#include "Voltage.h"
#include "Eeprom.h"
#include "Balancer.h"
#include <string.h>

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Definitions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
#define SOC_ALGORITHM_PASSES    2  // The number of times to recalculate the delta Q to move in each cell before starting balancing.

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Data
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Prototypes
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Global Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

void SOC_Balance(void)
{

    if(System_Cap_Demo.demo_present == 1)
    {
        // If Cap Demo system, calculate the relative SOC for each cell using the Voltage/Charge relationship for capacitors.
        int8 pass_num;
        int8 cell_num;
        unsigned int32 average_voltage;
        signed int32 temp;
        signed int32 target_charge[DC2100A_NUM_CELLS];

        // Start assuming that no charge will be moved.
        memset(target_charge, 0, sizeof(target_charge));

        // Calculate the delta Q to move SOC_ALGORITHM_PASSES times.  Since the battery balancers are not 100% efficient,
        // the balancer algorithm will evenly spread the losses resulting from the desired delta Q across each cell.
        // If all cells have the same capacity, this is the correct way to distribute the losses.  If the cells have different
        // capacities, however, then less of the losses should be applied to the cells with less capacity.
        for (pass_num = 0; pass_num < SOC_ALGORITHM_PASSES; pass_num++)
        {
            // Calculate the average cell voltage, if the target charge was moved from each cell.
            average_voltage = 0;
            for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
            {
                average_voltage += voltage_cell[DC2100A_PIC_BOARD_NUM][cell_num];
                temp = (target_charge[cell_num] * VOLTAGE_CELL_BITS_PER_MV * SOC_CAP_SCALE_FACTOR);
                average_voltage -= SIGNED_DIVIDE_BY_UNSIGNED_WITH_ROUND(temp, Eeprom_Cap_Values[DC2100A_PIC_BOARD_NUM].cap[cell_num]);
            }
            average_voltage = (average_voltage + DC2100A_NUM_CELLS / 2) / DC2100A_NUM_CELLS;

            // Calculate the amount of charge needed to bring each cell to this average cell voltage
            for (cell_num = 0; cell_num < DC2100A_NUM_CELLS; cell_num++)
            {
                target_charge[cell_num] = voltage_cell[DC2100A_PIC_BOARD_NUM][cell_num] - average_voltage;
                target_charge[cell_num] *= Eeprom_Cap_Values[DC2100A_PIC_BOARD_NUM].cap[cell_num];
                target_charge[cell_num] = SIGNED_DIVIDE_BY_UNSIGNED_WITH_ROUND(target_charge[cell_num], VOLTAGE_CELL_BITS_PER_MV * SOC_CAP_SCALE_FACTOR);
            }

            // Calculate the amount of charge actually moved when attempting to move the target_charge.
            Balancer_Set(target_charge);
        }
    }
    else
    {
        // Note - a battery balance algorithm would go here.
    }

    return;
}


//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
// Local Functions
//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
