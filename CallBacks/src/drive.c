/*
 * drive.c
 *
 *  Created on: Feb 6, 2019
 *      Author: bijan
 */

#include "FreeRTOS.h"
#include "hardware_abstraction_layer.h"
#include "MechanicalControlPackets.h"
#include "task.h"
#include "bcl_port.h"
#include "roboclaw_driver.h"
#include "drive.h"

static void drive_task(void *args);
static BCL_STATUS drive_callback(int bcl_inst, BclPayloadPtr payload);

static uint32_t last_packet_ticks = 0;
static struct motor_controller back_left;
static struct motor_controller mid_left;
static struct motor_controller front_left;
static struct motor_controller back_right;
static struct motor_controller mid_right;
static struct motor_controller front_right;

int init_drive(void)
{
    const int pulses_per_rev = 7 * 188;

    if(roboclaw_driver_init(&back_left, uart0, 0x80, pulses_per_rev, pulses_per_rev/360, true))
            return 1;
    if(roboclaw_driver_init(&mid_left, uart0, 0x81, pulses_per_rev, pulses_per_rev/360, true))
            return 1;
    if(roboclaw_driver_init(&front_left, uart0, 0x82, pulses_per_rev, pulses_per_rev/360, true))
            return 1;
    if(roboclaw_driver_init(&back_right, uart0, 0x80, pulses_per_rev, pulses_per_rev/360, false))
            return 1;
    if(roboclaw_driver_init(&mid_right, uart0, 0x81, pulses_per_rev, pulses_per_rev/360, false))
            return 1;
    if(roboclaw_driver_init(&front_right, uart0, 0x82, pulses_per_rev, pulses_per_rev/360, false))
            return 1;

    if(xTaskCreate(drive_task, "arm", configMINIMAL_STACK_SIZE, NULL, 1, NULL) != pdPASS)
            return 1;

    BCL_pktCallbackRegister(drive_callback, SET_TANK_DRIVE_SPEED);

    return 0;
}

void drive_task(void *args)
{
    uint32_t curTicks;
    while(1) {
        curTicks = xTaskGetTickCount();
        if(curTicks - last_packet_ticks > 500) {
            TankDrivePayload pyld;
            pyld.left = 0;
            pyld.right = 0;

            drive_callback(0, &pyld);
        }

        vTaskDelay(250);
    }
}

BCL_STATUS drive_callback(int bcl_inst, BclPayloadPtr payload)
{
    TankDrivePayload *pyld = (TankDrivePayload *)payload;

    back_left.set_speed(&back_left, pyld->left);
    mid_left.set_speed(&mid_left, pyld->left);
    front_left.set_speed(&mid_left, pyld->left);

    back_right.set_speed(&back_right, pyld->right);
    mid_right.set_speed(&mid_right, pyld->right);
    front_right.set_speed(&front_right, pyld->right);

    return BCL_OK;
}
