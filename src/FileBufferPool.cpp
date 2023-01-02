/*
 * FileBufferPool.cpp
 *
 *  Created on: Jun 3, 2020
 *      Author: JakeSoft
 */

//#define FILE_BUFFER_POOL_DEBUG

#include <Arduino.h>

#include "FileBufferPool.h"

//Flags if a buffer is in use or not. TRUE = used, FALSE = not used
static bool saBufferUsed[MAX_FILE_BUFFERS] = {};

//Buffers to dole out
static tFileBuffer saBuffers[MAX_FILE_BUFFERS] = {};

tFileBuffer* FileBufferPool::ReserveBuffer()
{
  tFileBuffer* lpBuff = nullptr;

  //Find available buffer
  int lFoundIdx = -1;
  for(int lIdx = 0; lIdx < MAX_FILE_BUFFERS && lFoundIdx < 0; lIdx++)
  {
    if(false == saBufferUsed[lIdx])
    {
      saBufferUsed[lIdx] = true;
      lFoundIdx = lIdx;
      lpBuff = &(saBuffers[lIdx]);
      lpBuff->mBufferNumber = lIdx;
      memset(lpBuff->maData, 0, FILE_BUFFER_SIZE);

#ifdef FILE_BUFFER_POOL_DEBUG
      Serial.print("Reserved buffer #"); Serial.print(lpBuff->mBufferNumber);
      Serial.print(" Buffers[");
      for(int lIdx = 0; lIdx < MAX_FILE_BUFFERS; lIdx++)
      {
        if(true == saBufferUsed[lIdx])
        {
          Serial.print(":1:");
        }
        else
        {
          Serial.print(":0:");
        }
      }
      Serial.println("]");
#endif
    }
  }


  return lpBuff;
}

void FileBufferPool::ReleaseBuffer(tFileBuffer* apBuffer)
{
  if(nullptr != apBuffer)
  {
    saBufferUsed[apBuffer->mBufferNumber] = false;

#ifdef FILE_BUFFER_POOL_DEBUG
    Serial.print("Release buffer #"); Serial.println(apBuffer->mBufferNumber);
#endif
  }
}


