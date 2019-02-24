/*
 * camera_mast.c
 *
 *  Created on: Feb 24, 2019
 *      Author: bijan
 */

#include "FreeRTOS.h"
#include "hardware_abstraction_layer.h"
#include "hal_uart.h"
#include "ax12a.h"
#include "MechanicalControlPackets.h"
#include "task.h"
#include "bcl_port.h"
#include "camera_mast.h"

// Dynamixel device IDs
#define PAN_ID              (0x01)
#define TILT_ID             (0x00)

static BCL_STATUS camera_mast_callback(int bcl_inst, BclPayloadPtr payload);
static void camera_mast_delay(uint32_t delay);

static uartInfo Uart_Settings = {
    .baud = 1000000,
    .wlen = 8,
    .parity = false,
    .twoStopBits = false
};

static uint16_t cur_pan;
static uint16_t cur_tilt;

//needed because vTaskDelay isn't available when init is called
static void camera_mast_delay(uint32_t delay)
{
    //120000 is the number of clocks per ms
    int i;
    for(i = 0; i < (delay * 120000); i++){}
}

int init_camera_mast(void)
{
    uart_init(uart4, &Uart_Settings);

    // unregister the uart interrupt because it's borked, yo
    // from mk ii
    UARTIntUnregister(vUART[(int)uart4].uartBase);

    BCL_pktCallbackRegister(camera_mast_callback, SET_MAST_POS);

    // wait a bit then initialize the positions
    camera_mast_delay(100);

    // Only uncomment this if dynamixels are borked/new, parameter sets the new
    // ID. Only one can be uncommented at a time.
    //AX12A_unfuck(Uart, TILT_ID, Uart_Settings.baud);
    //AX12A_unfuck(Uart, PAN_ID, Uart_Settings.baud);

    // set to position mode to initialize
    AX12A_setPositionRotation(uart4, PAN_ID, 0x3FF, 0x000);
    camera_mast_delay(10);
    AX12A_setPositionRotation(uart4, TILT_ID, 0x3FF, 0x000);
    camera_mast_delay(10);

    // initialize to a good position
    AX12A_setGoalPosition(uart4, PAN_ID, 0x297);
    cur_pan = 0x297;
    camera_mast_delay(10);
    AX12A_setGoalPosition(uart4, TILT_ID, (0x3FF/2) + 95);
    cur_tilt = (0x3FF/2) + 95;

    return 0;
}

static BCL_STATUS camera_mast_callback(int bcl_inst, BclPayloadPtr payload)
{
    CameraMastPayload *pyld = payload;

    // set the new goal positions
    if(cur_pan + pyld->pan <= 0x3FF && cur_pan + pyld->pan >= 0) {
        cur_pan += pyld->pan;
    }
    //The first number seems to be the limit for the zed pointing down.
    //The second seems to be the limit for the zed pointing up
    if(cur_tilt + pyld->tilt <= 0x300 && cur_tilt + pyld->tilt >= 0x150) {
        cur_tilt += pyld->tilt;
    }

    AX12A_setGoalPosition(uart4, PAN_ID, cur_pan);
    vTaskDelay(10);
    AX12A_setGoalPosition(uart4, TILT_ID, cur_tilt);
    vTaskDelay(10);

    return BCL_OK;
}
