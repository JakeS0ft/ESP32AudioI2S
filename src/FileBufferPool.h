/*
 * FileBufferPool.h
 *
 *  Created on: Jun 3, 2020
 *      Author: JakeSoft
 */

#ifndef _FILEBUFFERPOOL_H_
#define _FILEBUFFERPOOL_H_

#include <Arduino.h>

#define FILE_BUFFER_SIZE 512
#define MAX_FILE_BUFFERS 6

/**
 * A memory block container for file read buffers
 */
struct tFileBuffer
{
  //Keep track of the buffer number
  int mBufferNumber;
  //Block of memory to store bytes of file data
  uint8_t maData[FILE_BUFFER_SIZE];
};

namespace FileBufferPool
{

  /**
   * Reserves a buffer.
   * Return: Pointer to the reserved buffer
   */
  tFileBuffer* ReserveBuffer();

  /**
   * Releases a buffer.
   *
   * Buffers should be released so that they can be returned to the pool
   * after they are no longer needed. Callers are responsible for not
   * using the buffer anymore after the buffer is released.
   *
   * Args:
   *  apBuffer - Pointer to buffer to be released.
   */
  void ReleaseBuffer(tFileBuffer* apBuffer);
};


#endif /* LIBRARIES_NRF52AUDIO_FILEBUFFERPOOL_H_ */
