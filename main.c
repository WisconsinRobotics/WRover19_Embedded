// C standard libs, needed for compilation. Put in every file!
#include <stdint.h>
#include <stdbool.h>

//needed to initialize launchpad clock/interrupts
#include "inc/hw_ints.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "hardware_abstraction_layer.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bcl_port.h"
#include "FreeRTOS_IP.h"
#include "config_udp.h"
#include "MechanicalControlPackets.h"
#include "CarHorn.h"
#include "arm.h"
#include "drive.h"
#include "ride_height.h"
#include "solenoid.h"
#include "camera_mast.h"
#include "soil_door.h"
#include "ControlServicePackets.h"
extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);

static void InitBCLUDP(int bcl_inst);

static void InitBCLUDP(int bcl_inst)
{

    {
        static const uint8_t Ip_Address[ipIP_ADDRESS_LENGTH_BYTES]      = {192,168,1,10};
        static const uint8_t Net_Mask[ipIP_ADDRESS_LENGTH_BYTES]        = {255,255,255,0};
        static const uint8_t Gateway_Address[ipIP_ADDRESS_LENGTH_BYTES] = {192,168,1,1};
        static const uint8_t DNS_Address[ipIP_ADDRESS_LENGTH_BYTES]     = {8,8,8,8};
        static       uint8_t Mac_Address[6];
        eth_read_mac_addr(Mac_Address);
        FreeRTOS_IPInit(
            Ip_Address,
            Net_Mask,
            Gateway_Address,
            DNS_Address,
            Mac_Address);
    }

    BCL_configUDP(bcl_inst, 10000, 10000, FreeRTOS_inet_addr_quick(192,168,1,20)); // Sam: these arguments will almost certainly have to change

}

int main(void)
{
    global_sys_clock = SysCtlClockFreqSet(
            SYSCTL_XTAL_25MHZ|SYSCTL_OSC_MAIN|SYSCTL_USE_PLL|SYSCTL_CFG_VCO_480,
            120000000
    );

    //Wait for external peripherals to initialize
    {
        uint32_t i = 0;
        while (i++ < 1000000);
    }

    // Register FreeRTOS's Supervisor Call handler
    IntRegister(FAULT_SVCALL, vPortSVCHandler);
    // Register FreeRTOS's Pend Supervisor Call handler
    IntRegister(FAULT_PENDSV, xPortPendSVHandler);
    // Register FreeRTOS's SysTick handler
    IntRegister(FAULT_SYSTICK, xPortSysTickHandler);

    int main_bcl_service;
    main_bcl_service = BCL_initService();

    uartManager(0);

    i2c_initAvailableBusses(true, 0);

    init_ride_height();
    init_drive();
    init_camera_mast();
    init_soil_door();
    init_arm();
    checkForSolenoid();

    InitBCLUDP(main_bcl_service);

    vTaskStartScheduler();

	return 0;
}
