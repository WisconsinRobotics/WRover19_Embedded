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
#include <stdlib.h> /// use for absolute value?

static void arm_task(void *args);
static BCL_STATUS arm_callback(int bcl_inst, BclPayloadPtr payload);

// all motor controllers for the arm joints
static motor_controller arm_claw;
static motor_controller arm_turntable;
static motor_controller arm_humerus;
static motor_controller arm_forearm;
static motor_controller arm_wrist_ud; // wrist up down
static motor_controller arm_wrist_r; // wrist rotate
static motor_controller arm_sliding;

int init_arm(void)
{
    const int pulses_per_rev = 7*188*3;

    // initialize each joint driver
    if(roboclaw_driver_init(&arm_claw, uart3, 0x80, pulses_per_rev, pulses_per_rev/360, true)) // last arg = motor 1
        return 1;
    if(roboclaw_driver_init(&arm_turntable, uart3, 0x80, pulses_per_rev, pulses_per_rev/360, false)) // last arg = motor 2
            return 1;
    if(roboclaw_driver_init(&arm_humerus, uart3, 0x81, pulses_per_rev, pulses_per_rev/360, true))
            return 1;
    if(roboclaw_driver_init(&arm_forearm, uart3, 0x81, pulses_per_rev, pulses_per_rev/360, false))
            return 1;
    if(roboclaw_driver_init(&arm_wrist_ud, uart3, 0x82, pulses_per_rev, pulses_per_rev/360, true))
            return 1;
    if(roboclaw_driver_init(&arm_wrist_r, uart3, 0x82, pulses_per_rev, pulses_per_rev/360, false))
            return 1;
    if(roboclaw_driver_init(&arm_sliding, uart3, 0x83, pulses_per_rev, pulses_per_rev/360, true))
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

/*
 * Change payload to absolute values and set multiplier to hold payload's original sign
 */

void get_direction(int8_t *payload, int *multiplier) {
    if(payload < 0) {
        *multiplier = -1;
        *payload = -(*payload);
    }
    else {
        *multiplier = 1;
    }
}

BCL_STATUS arm_callback(int bcl_inst, BclPayloadPtr payload)
{
    ArmPayload *pyld = (ArmPayload *)payload;
    int mult;

    get_direction(&pyld->claw, &mult); // change the payload to absolute value, add multiplier
    arm_claw.set_position(&arm_claw, 20 * mult, pyld->claw);

    get_direction(&pyld->turntable, &mult);
    arm_turntable.set_position(&arm_turntable, 20 * mult, pyld->turntable);

    get_direction(&pyld->humerus, &mult);
    arm_humerus.set_position(&arm_humerus, 20 * mult, pyld->humerus);

    get_direction(&pyld->forearm, &mult);
    arm_forearm.set_position(&arm_forearm, 20 * mult, pyld->forearm);

    get_direction(&pyld->wrist_up_down, &mult);
    arm_wrist_ud.set_position(&arm_wrist_ud, 20 * mult, pyld->wrist_up_down);

    get_direction(&pyld->wrist_rot, &mult);
    arm_wrist_r.set_position(&arm_wrist_r, 20 * mult, pyld->wrist_rot);

    get_direction(&pyld->sliding, &mult);
    arm_sliding.set_position(&arm_sliding, 20 * mult, pyld->sliding);

    return BCL_OK;
}
