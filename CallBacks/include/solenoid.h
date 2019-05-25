/*
 * solenoid.h
 *
 *  Created on: Mar 6, 2019
 *      Author: Justin
 */

#ifndef CALLBACKS_INCLUDE_SOLENOID_H_
#define CALLBACKS_INCLUDE_SOLENOID_H_

void initSolenoid(uint32_t port, uint8_t pin);
eI2CController checkForSolenoid();

#endif /* CALLBACKS_INCLUDE_SOLENOID_H_ */
