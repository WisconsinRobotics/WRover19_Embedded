/*
 * ride_height.c
 *
 *  Created on: Feb 18, 2019
 *      Author: bijan
 */
#include "FreeRTOS.h"
#include "hardware_abstraction_layer.h"
#include "MechanicalControlPackets.h"
#include "roboclaw_driver.h"
#include "task.h"
#include "bcl_port.h"

//The I2C bus that the ride height board is on
static BCL_STATUS ride_height_callback(int bcl_inst, BclPayloadPtr payload);

static motor_controller front_rh;
static motor_controller back_rh;

int init_ride_height(void)
{
    // initialize each joint driver
    if(roboclaw_driver_init(&front_rh, uart0, 0x83, 0, 0, true))// last arg = motor 1
            return 1;
    if(roboclaw_driver_init(&back_rh, uart0, 0x83, 0, 0, false))// last arg = motor 2
            return 1;

    BCL_pktCallbackRegister(ride_height_callback, SET_RIDE_HEIGHT_SPEED);

    return 0;
}

BCL_STATUS ride_height_callback(int bcl_inst, BclPayloadPtr payload)
{
    RideHeightPayload *pyld = (RideHeightPayload *)payload;

    if(pyld->front > 0) {
        front_rh.set_speed(&front_rh, -76);
    } else if(pyld->front < 0) {
        front_rh.set_speed(&front_rh, 76);
    } else {
        front_rh.set_speed(&front_rh, 0);
    }

    if(pyld->back > 0) {
        back_rh.set_speed(&back_rh, 127);
    } else if(pyld->back < 0) {
        back_rh.set_speed(&back_rh, -127);
    } else {
        back_rh.set_speed(&back_rh, 0);
    }

    return BCL_OK;
}
