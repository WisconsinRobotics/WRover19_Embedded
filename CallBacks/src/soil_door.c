/*
 * soil_door.c
 *
 *  Created on: May 26, 2019
 *      Author: bijan
 */

#include "FreeRTOS.h"
#include "hardware_abstraction_layer.h"
#include "hal_uart.h"
#include "ax12a.h"
#include "MechanicalControlPackets.h"
#include "task.h"
#include "bcl_port.h"
#include "soil_door.h"

#define SCI_ID              (0x02)

#define DOOR_OPEN 0x300
#define DOOR_CLOSE 0x1F0

static BCL_STATUS soil_door_callback(int bcl_inst, BclPayloadPtr payload);
static void ax12_delay(uint32_t delay);

static const uartInfo Uart_Settings = {
    .baud = 1000000,
    .wlen = 8,
    .parity = false,
    .twoStopBits = false
};

//needed because vTaskDelay isn't available when init is called
static void ax12_delay(uint32_t delay)
{
    //120000 is the number of clocks per ms
    int i;
    for(i = 0; i < (delay * 120000); i++){}
}

int init_soil_door(void)
{
    uart_init(uart4, &Uart_Settings);

    // set to position mode to initialize
    AX12A_setPositionRotation(uart4, SCI_ID, 0x3FF, 0x000);
    ax12_delay(10);

    // initialize to a good position
    AX12A_setGoalPosition(uart4, SCI_ID, DOOR_CLOSE);
    ax12_delay(50); // wait a second for the dynamixels to get into position

    BCL_pktCallbackRegister(soil_door_callback, SET_SOIL_DOOR_POS);

    return 0;
}

static BCL_STATUS soil_door_callback(int bcl_inst, BclPayloadPtr payload)
{
    DynamixelPositionPayload *pyld = (DynamixelPositionPayload *) payload;

    if(pyld->position == 0) {
        AX12A_setGoalPosition(uart4, SCI_ID, DOOR_CLOSE);
    } else {
        AX12A_setGoalPosition(uart4, SCI_ID, DOOR_CLOSE);
    }
}
