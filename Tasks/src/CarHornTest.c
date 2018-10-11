// C standard libs, needed for compilation. Put in every file!
#include <stdint.h>
#include <stdbool.h>

#include "bcl_port.h"
#include "CarHornTest.h"

static int HornEnabled = 0;

BCL_STATUS HonkHorn(int bcl_inst, BclPayloadPtr payload) {
    if (!HornEnabled) {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA)) {
        }
        GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE,  GPIO_PIN_1);
    }

    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_1, GPIO_PIN_1);
    vTaskDelay(1000);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_1, GPIO_PIN_0);

    return BCL_OK;
}
