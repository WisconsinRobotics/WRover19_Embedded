// C standard libs, needed for compilation. Put in every file!
#include <stdint.h>
#include <stdbool.h>

#include "bcl_port.h"
#include "CarHorn.h"

static int HornEnabled = 0;

BCL_STATUS HonkHorn(int bcl_inst, BclPayloadPtr payload) {
    if (!HornEnabled) {
        SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
        while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)) {
        }
        GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE,  GPIO_PIN_0);
    }

    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, GPIO_PIN_0);
    vTaskDelay(1000);
    GPIOPinWrite(GPIO_PORTD_BASE, GPIO_PIN_0, !GPIO_PIN_0);

    return BCL_OK;
}
