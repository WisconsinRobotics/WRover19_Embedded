/*
 * arm.c
 *
 *  Created on: Dec 10, 2018
 *      Author: bijan
 */
#include <stdbool.h>

#include "FreeRTOS.h"
#include "hardware_abstraction_layer.h"
#include "MechanicalControlPackets.h"
#include "task.h"
#include "bcl_port.h"
#include "roboclaw_driver.h"
#include "arm.h"

#define	ARM_PAYLOAD_SIZE (14)
#define ARM_PACKET_SIZE (PACKET_MIN_SIZE + ARM_PAYLOAD_SIZE)

static void arm_task(void *args);
static BCL_STATUS arm_get_pos_callback(int bcl_inst, BclPayloadPtr payload);
static BCL_STATUS arm_pos_callback(int bcl_inst, BclPayloadPtr payload);
static BCL_STATUS arm_speed_callback(int bcl_inst, BclPayloadPtr payload);

static uint32_t last_packet_ticks = 0;
static bool last_packet_speed = false;
// all motor controllers for the arm joints
static motor_controller arm_claw;
static motor_controller arm_turntable;
static motor_controller arm_shoulder;
static motor_controller arm_forearm;
static motor_controller arm_wrist_left; // wrist up down
static motor_controller arm_wrist_right; // wrist rotate
static motor_controller arm_elbow;

int init_arm(void)
{
    const int pulses_per_rev = 2047;

    // initialize each joint driver
    if(roboclaw_driver_init(&arm_wrist_left, uart0, 0x84, pulses_per_rev, pulses_per_rev/360, true))// last arg = motor 1
            return 1;
    if(roboclaw_driver_init(&arm_wrist_right, uart0, 0x84, pulses_per_rev, pulses_per_rev/360, false))// last arg = motor 2
            return 1;
    if(roboclaw_driver_init(&arm_shoulder, uart0, 0x85, pulses_per_rev, pulses_per_rev/360, true))
            return 1;
    if(roboclaw_driver_init(&arm_forearm, uart0, 0x85, pulses_per_rev, pulses_per_rev/360, false))
            return 1;
    if(roboclaw_driver_init(&arm_elbow, uart0, 0x86, pulses_per_rev, pulses_per_rev/360, true))
            return 1;
    if(roboclaw_driver_init(&arm_turntable, uart0, 0x86, pulses_per_rev, pulses_per_rev/360, false))
            return 1;
    if(roboclaw_driver_init(&arm_claw, uart0, 0x87, pulses_per_rev, pulses_per_rev/360, true))
        return 1;


    if(xTaskCreate(arm_task, "arm", configMINIMAL_STACK_SIZE, NULL, 1, NULL) != pdPASS)
        return 1;

    BCL_pktCallbackRegister(arm_pos_callback, SET_ARM_POS);
    BCL_pktCallbackRegister(arm_speed_callback, SET_ARM_SPEED);
    BCL_pktCallbackRegister(arm_get_pos_callback, QUERY_ARM_POS);

    return 0;
}

void arm_task(void *args)
{
    uint32_t curTicks;
    while(1) {
        curTicks = xTaskGetTickCount();
        if((curTicks - last_packet_ticks > 500) && last_packet_speed) {
            arm_claw.set_speed(&arm_claw, 0);
            arm_turntable.set_speed(&arm_turntable, 0);
            arm_shoulder.set_speed(&arm_shoulder, 0);
            arm_forearm.set_speed(&arm_forearm, 0);
            arm_wrist_left.set_speed(&arm_wrist_left, 0);
            arm_wrist_right.set_speed(&arm_wrist_right, 0);
            arm_elbow.set_speed(&arm_elbow, 0);
        }

        vTaskDelay(250);
    }
}

/*
 * Change payload to absolute values and set multiplier to hold payload's original sign
 */

void get_direction(int16_t *payload, int *multiplier) {
    if(*payload < 0) {
        *multiplier = -1;
        *payload = -(*payload);
    }
    else {
        *multiplier = 1;
    }
}

BCL_STATUS arm_get_pos_callback(int bcl_inst, BclPayloadPtr payload)
{
    ArmPayload pyld; //fill a payload struct
    BclPacket pkt;
    uint8_t packetBuff[ARM_PACKET_SIZE];
  
    pyld.claw = arm_claw.get_position(&arm_claw);
    pyld.turntable = arm_turntable.get_position(&arm_turntable);
    pyld.shoulder = arm_shoulder.get_position(&arm_shoulder);
    pyld.forearm = arm_forearm.get_position(&arm_forearm);
    pyld.wrist_left = arm_wrist_left.get_position(&arm_wrist_left);
    pyld.wrist_right = arm_wrist_right.get_position(&arm_wrist_right);
    pyld.elbow = arm_elbow.get_position(&arm_elbow);

    BCL_STATUS report = InitializeReportArmPositionPacket(&pkt, &pyld);

    BCL_sendPacket(bcl_inst, &pkt, packetBuff, ARM_PACKET_SIZE);
    return BCL_OK;    
}

BCL_STATUS arm_pos_callback(int bcl_inst, BclPayloadPtr payload)
{
    ArmPayload *pyld = (ArmPayload *)payload;
    int mult;

    last_packet_speed = false;

    get_direction(&pyld->claw, &mult); // change the payload to absolute value, add multiplier
    arm_claw.set_position(&arm_claw, 20 * mult, pyld->claw);

    get_direction(&pyld->turntable, &mult);
    arm_turntable.set_position(&arm_turntable, 20 * mult, pyld->turntable);

    get_direction(&pyld->shoulder, &mult);
    arm_shoulder.set_position(&arm_shoulder, 20 * mult, pyld->shoulder);

    get_direction(&pyld->forearm, &mult);
    arm_forearm.set_position(&arm_forearm, 20 * mult, pyld->forearm);

    get_direction(&pyld->wrist_left, &mult);
    arm_wrist_left.set_position(&arm_wrist_left, 20 * mult, pyld->wrist_left);

    get_direction(&pyld->wrist_right, &mult);
    arm_wrist_right.set_position(&arm_wrist_right, 20 * mult, pyld->wrist_right);

    get_direction(&pyld->elbow, &mult);
    arm_elbow.set_position(&arm_elbow, 20 * mult, pyld->elbow);

    return BCL_OK;
}


static BCL_STATUS arm_speed_callback(int bcl_inst, BclPayloadPtr payload)
{
    ArmPayload *pyld = (ArmPayload *)payload;

    last_packet_ticks = xTaskGetTickCount();
    last_packet_speed = true;

    arm_claw.set_speed(&arm_claw, pyld->claw);
    arm_turntable.set_speed(&arm_turntable, pyld->turntable);
    arm_shoulder.set_speed(&arm_shoulder, pyld->shoulder);
    arm_forearm.set_speed(&arm_forearm, pyld->forearm);
    arm_wrist_left.set_speed(&arm_wrist_left, pyld->wrist_left);
    arm_wrist_right.set_speed(&arm_wrist_right, pyld->wrist_right);
    arm_elbow.set_speed(&arm_elbow, pyld->elbow);

    return BCL_OK;
}
