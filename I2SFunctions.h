/*
 * IS2Play.h
 *
 *  Created on: Apr 7, 2022
 *      Author: Jake
 */

#ifndef I2SFUNCTIONS_H_
#define I2SFUNCTIONS_H_

//These functions won't build properly with C++ because the
//low-level ESP32 SDK only provides a C interface, so rather
//than fill the I2SWavPlayer function up with extern "C" {} blocks,
//they are all declared here and just called from the class.
//See definitions in I2SFunctions.c

extern "C" void I2SPlayConfigI2S(int aMckPin,
                                 int aBckPin,
                                 int aWsPin,
                                 int aDataOutPin,
								 int aDataLength);

extern "C" size_t I2SPlayI2SWrite(int32_t* aBuffer, size_t aBufferSize);

#endif /* I2SFUNCTIONS_H_ */
