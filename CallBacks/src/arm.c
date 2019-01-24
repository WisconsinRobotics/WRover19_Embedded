/*
 * arm.c
 *
 *  Created on: Dec 10, 2018
 *      Author: bijan
 */

#include "FreeRTOS.h"
#include "hardware_abstraction_layer.h"
#include "MechanicalControlPackets.h"
#include "task.h"
#include "bcl_port.h"
#include "roboclaw_driver.h"
#include "arm.h"

static void arm_task(void *args);
static BCL_STATUS arm_callback(int bcl_inst, BclPayloadPtr payload);

static motor_controller arm_cont;

int init_arm(void)
{
    const int pulses_per_rev = 7*188*3;

    if(roboclaw_driver_init(&arm_cont, uart3, 0x80, pulses_per_rev, pulses_per_rev/360, true))
        return 1;

    if(xTaskCreate(arm_task, "arm", configMINIMAL_STACK_SIZE, NULL, 1, NULL) != pdPASS)
        return 1;

    BCL_pktCallbackRegister(arm_callback, SET_ARM_POS);

    return 0;
}

void arm_task(void *args)
{
    while(1) {
        vTaskDelay(100);
    }
}

BCL_STATUS arm_callback(int bcl_inst, BclPayloadPtr payload)
{
    ArmPayload *pyld = (ArmPayload *)payload;
    int mult = 1;

    if(pyld->claw < 0)
    {
        mult = -1;
        pyld->claw = -pyld->claw;
    }

    arm_cont.set_position(&arm_cont, 20 * mult, pyld->claw);


    return BCL_OK;
}
