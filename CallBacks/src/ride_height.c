/*
 * ride_height.c
 *
 *  Created on: Feb 18, 2019
 *      Author: bijan
 */
#include "FreeRTOS.h"
#include "hardware_abstraction_layer.h"
#include "MechanicalControlPackets.h"
#include "ess.h"
#include "task.h"
#include "bcl_port.h"

#define RIDE_HEIGHT_I2C_ADDR (0x0A)
#define FRONT_PAIR_SHIFT              (0x02)
#define BACK_PAIR_SHIFT             (0x00)
#define STOP                 (0x00)
#define DRIVE_UP        (0x01)
#define DRIVE_DOWN          (0x02)

//The I2C bus that the ride height board is on
eI2CController rhBus = i2c_invalid;
static BCL_STATUS ride_height_callback(int bcl_inst, BclPayloadPtr payload);

int init_ride_height(void)
{
    BCL_pktCallbackRegister(ride_height_callback, SET_RIDE_HEIGHT_SPEED);

    return 0;
}

BCL_STATUS ride_height_callback(int bcl_inst, BclPayloadPtr payload)
{
    RideHeightPayload *pyld = (RideHeightPayload *)payload;
    uint8_t data;

    if(rhBus == i2c_invalid)
        rhBus = ESS_findDevice(RIDE_HEIGHT_I2C_ADDR);

    //Tell the ride height board what to do!
    data = 0;
    //Move both the front and the back forward
    if(pyld->front > 0) data |= DRIVE_UP << FRONT_PAIR_SHIFT;
    else if(pyld->front < 0) data |= DRIVE_DOWN << FRONT_PAIR_SHIFT;
    else data |= STOP << FRONT_PAIR_SHIFT;

    if(pyld->back > 0) data |= DRIVE_UP << BACK_PAIR_SHIFT;
    else if(pyld->back < 0) data |= DRIVE_DOWN << BACK_PAIR_SHIFT;
    else data |= STOP << BACK_PAIR_SHIFT;

    //Send the stuff
    ESS_sendCommand(rhBus, RIDE_HEIGHT_I2C_ADDR, ESS_CMD_RIDE_HEIGHT, &data, 1);

    return BCL_OK;
}
