#include "FreeRTOS.h"
#include "semphr.h"
#include "MechanicalControlPackets.h"
#include "bcl.h"
#include "solenoid.h"
#include "ess.h"
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "hardware_abstraction_layer.h"
#define ON_TIME (500)
#define WAIT_TIME 1000

eI2CController solenoid_bus = i2c_invalid;
static BCL_STATUS solenoidCallback(int bcl_inst, BclPayloadPtr payload);

uint32_t g_port;
uint8_t g_pin;
uint8_t base_pin;


uint32_t sol_reset_port = GPIO_PORTH_BASE;
uint32_t sol_base_port = GPIO_PORTA_BASE;
uint8_t sol_reset_pin = 0xFF;
uint8_t sol_base_pin = 0xFF;

eI2CController checkForSolenoid(){
    eI2CController solenoid_bus = i2c_invalid;
    //Find out which daughter board the solenoid board is on, if it is on one
    //It looks like only these two pins are on H, so this should be safe
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOH));

    GPIOPinTypeGPIOInput(sol_reset_port, GPIO_PIN_0 | GPIO_PIN_1);
    if(!(GPIOPinRead(sol_reset_port, GPIO_PIN_1) & GPIO_PIN_1)) {
        sol_base_pin = GPIO_PIN_5;
        solenoid_bus = i2c_7;
    }
    else if(!(GPIOPinRead(sol_reset_port, GPIO_PIN_0) & GPIO_PIN_0)) {
        sol_base_pin = GPIO_PIN_7;
        solenoid_bus = i2c_6;
    }

    GPIOPinTypeGPIOOutput(sol_reset_port, GPIO_PIN_0 | GPIO_PIN_1);

    if(solenoid_bus != i2c_invalid) initSolenoid(sol_base_port, sol_base_pin);
    return solenoid_bus;
}

void initSolenoid(uint32_t port, uint8_t pin)
{
    BCL_pktCallbackRegister(solenoidCallback, ACTIVATE_SOLENOID);

    g_port = port;
    g_pin = pin;
    uint32_t _strength, _type;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA));
    GPIOPadConfigGet(g_port, g_pin, &_strength, &_type);
    GPIOPinTypeGPIOOutput(g_port, g_pin);
    GPIOPadConfigSet(g_port, g_pin, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);
    GPIOPinWrite(g_port, g_pin, 0);
}

//uint32_t wait_time;
static BCL_STATUS solenoidCallback(int bcl_inst, BclPayloadPtr payload)
{
    int i;

    GPIOPinWrite(g_port, g_pin, 0xff);

    vTaskDelay(WAIT_TIME);

    GPIOPinWrite(g_port, g_pin, 0);

    return BCL_OK;

    //vTaskDelay(1);
}
