/******************************************************************************
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 ******************************************************************************/

/*
 * I2SWavPlayer.cpp
 *
 *  Created on: Jun 30, 2019
 *      Author: JakeSoft
 */


 /* USeful Links
  * ESP32 I2S manual: 			 https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/i2s.html
  * ESP32 I2S api documentation: https://docs.espressif.com/projects/esp-idf/en/v4.2.3/esp32/api-reference/peripherals/i2s.html
  *
  *
  *
  */
#include "Arduino.h"
#include "I2SWavPlayer.h"


I2SWavPlayer::I2SWavPlayer(int32_t aPinMCK,
                           int32_t aPinBCLK,
                           int32_t aPinLRCK,
                           int32_t aPinDIN,
                           int32_t aPinSD)
{
//#if defined(NRF52) || defined(NRF52_SERIES)
// allocate 2 buffers, A and B to alternately fill them with audio data and pass them to the I2S driver
   memset(maBufferA, 0, I2S_BUF_SIZE);
   memset(maBufferB, 0, I2S_BUF_SIZE);
   mBufferASelected = true;
//#endif

   mPinMCK = aPinMCK;
   mPinBCLK = aPinBCLK;
   mPinLRCK = aPinLRCK;
   mPinDIN = aPinDIN;
   mPinSD = aPinSD;

#if defined(ESP32)
   m_pin_config.bck_io_num   = mPinBCLK;
   m_pin_config.ws_io_num    = mPinLRCK; //  wclk
   m_pin_config.data_out_num = mPinDIN;
   m_pin_config.data_in_num  = I2S_PIN_NO_CHANGE;  //not used
#if(ESP_IDF_VERSION_MAJOR >= 4 && ESP_IDF_VERSION_MINOR >= 4)
   m_pin_config.mck_io_num   = mPinMCK;
#endif
   // hard-wire the 1st (0) I2S port of the ESP32
   const esp_err_t result = i2s_set_pin((i2s_port_t) I2S_NUM_0, &m_pin_config);
   /*
    // initialize the buffer size (default is 5*1600 bytes = 8kbytes allocated in Heap
	if(!InBuff.isInitialized()) {
       size_t size = InBuff.init();
       if (size > 0) {
           AUDIO_INFO("PSRAM %sfound, inputBufferSize: %u bytes", InBuff.havePSRAM()?"":"not ", size - 1);
       }
   }
   */
#endif

   for(int lIdx = 0; lIdx < MAX_WAV_FILES; lIdx++)
   {
      mapWavFile[lIdx] = nullptr;
   }

   mSamplesMixed = 0;
   mSampleRate = ee2205;
   mVolume = 1.0;
   mMixingStrategy = eeMixingModeAlternating;
}

I2SWavPlayer::~I2SWavPlayer()
{
   StopPlayback();

   for(int lIdx = 0; lIdx < MAX_WAV_FILES; lIdx++)
   {
      if(nullptr != mapWavFile[lIdx])
      {
         mapWavFile[lIdx]->Close();
      }
   }
}

bool I2SWavPlayer::Init()
{
   Configure_I2S();
   return true;
}

void I2SWavPlayer::SetWavFile(ISDWavFile* apWavFile, int aFileIndex)
{
   if(aFileIndex < MAX_WAV_FILES && aFileIndex >= 0)
   {
      mapWavFile[aFileIndex] = apWavFile;
      if(nullptr != apWavFile)
      {
         //Force down-sampling of 44.1KHz to 22.05 KHz
         if(apWavFile->GetHeader().sampleRate == 44100 && mSampleRate == ee2205)
         {
            //Serial.println("Downsampling enabled.");
            maDownsampleParams[aFileIndex].mIsDownsample = true;
         }
         else //Don't do down-sampling
         {
            //Serial.println("Downsampling disabled.");
            maDownsampleParams[aFileIndex].mIsDownsample = false;
         }

         //Serial.print("SampleRate: ");
         //Serial.println(apWavFile->GetHeader().sampleRate);

         apWavFile->SeekStartOfData();


      }
   }
}

void  I2SWavPlayer::ClearAllWavFiles()
{
   for(int lIdx = 0; lIdx < MAX_WAV_FILES; lIdx++)
   {
      mapWavFile[lIdx] = nullptr;
   }

//#if defined(NRF52) || defined(NRF52_SERIES)
   //Flush the I2S buffers so only silence will play
   memset(maBufferA, 0, sizeof(int32_t)*I2S_BUF_SIZE);
   memset(maBufferB, 0, sizeof(int32_t)*I2S_BUF_SIZE);
   memset(maMixedI2SSamples, 0, sizeof(int32_t)*I2S_BUF_SIZE);
//#elif defined(ESP32)

//#endif

}

void I2SWavPlayer::StartPlayback()
{

   for(int lIdx = 0; lIdx < I2S_BUF_SIZE; lIdx++)
   {
      GenerateMixedI2SSample(maBufferA[lIdx]);
   }
   for(int lIdx = 0; lIdx < I2S_BUF_SIZE; lIdx++)
   {
      GenerateMixedI2SSample(maBufferB[lIdx]);
   }

   mBufferASelected = true;
#if defined(NRF52) || defined(NRF52_SERIES)

   NRF_I2S->RXTXD.MAXCNT = I2S_BUF_SIZE;
   NRF_I2S->TXD.PTR = (uint32_t) maBufferA;
   NRF_I2S->EVENTS_TXPTRUPD = 0;

   // restart the MCK generator (a TASKS_STOP will disable the MCK generator)
   // Start transmitting I2S data
   NRF_I2S->TASKS_START = 1;
#elif defined(ESP32)
   i2s_zero_dma_buffer((i2s_port_t)I2S_NUM_0);

   esp_err_t err = i2s_write((i2s_port_t)I2S_NUM_0, GetBufferA(), sizeof(int32_t)*I2S_BUF_SIZE, 0, 100);
   //esp_err_t err = i2s_write((i2s_port_t)I2S_NUM_0, (const char*) &maMixedI2SSamples[0], sizeof(int32_t)*I2S_BUF_SIZE, 0, 100);
	if(err != ESP_OK) {
       log_e("ESP32 Errorcode %i", err);
       //return false;
   }
#endif
}

void I2SWavPlayer::StopPlayback()
{
   // Stop transmitting I2S data, a TASKS_STOP will disable the MCK generator
#if defined(NRF52) || defined(NRF52_SERIES)
   NRF_I2S->TASKS_STOP = 1;
#elif defined(ESP32)
   i2s_stop((i2s_port_t)I2S_NUM_0);
#endif
}

bool I2SWavPlayer::ContinuePlayback()
{
   bool lPlaybackIsDone = false;

#if defined(NRF52) || defined(NRF52_SERIES)
   if (NRF_I2S->EVENTS_TXPTRUPD != 0) //It's time to update a buffer
   {
      if (mBufferASelected == true)
      {
        NRF_I2S->EVENTS_TXPTRUPD = 0; //Start consuming buffer B
        NRF_I2S->TXD.PTR = (uint32_t)maBufferA;

        memcpy(maBufferA, maMixedI2SSamples, sizeof(int32_t)*I2S_BUF_SIZE);


      }
      else
      {
        NRF_I2S->EVENTS_TXPTRUPD = 0; //Start consuming buffer A
        NRF_I2S->TXD.PTR = (uint32_t)maBufferB;

        }
        memcpy(maBufferB, maMixedI2SSamples, sizeof(int32_t)*I2S_BUF_SIZE);

      }
#endif

   // todo: how to check if buffer is consumed in ESP32?
#if defined(ESP32)
   	   esp_err_t err;
   	   i2s_event_t lI2SEvent;
       if (xQueueReceive(this->m_i2sQueue, &lI2SEvent, portMAX_DELAY) == pdPASS)
       {
    	   if (lI2SEvent.type == I2S_EVENT_TX_DONE)
    	   {
    		   if (mBufferASelected == true)
			   {
				 memcpy(maBufferA, maMixedI2SSamples, sizeof(int32_t)*I2S_BUF_SIZE);
				 err = i2s_write((i2s_port_t)I2S_NUM_0, GetBufferA(), sizeof(int32_t)*I2S_BUF_SIZE, 0, 100);
			   }
			   else  // start consuming Buffer B
			   {
				 memcpy(maBufferB, maMixedI2SSamples, sizeof(int32_t)*I2S_BUF_SIZE);
				 err = i2s_write((i2s_port_t)I2S_NUM_0, GetBufferB(), sizeof(int32_t)*I2S_BUF_SIZE, 0, 100);
			   }
				if(err != ESP_OK) {
					log_e("ESP32 Errorcode %i", err);
					//return false;
				}
    	   }
       }
#endif

      PopulateMixingBuffer(); //Pre-mix next set of samples

#if defined(NRF52) || defined(NRF52_SERIES)

      //Toggle buffer selector
      mBufferASelected = !mBufferASelected;
#endif


   if(0 == mSamplesMixed)
   {
      lPlaybackIsDone = true;
   }

   return lPlaybackIsDone;
}

bool I2SWavPlayer::IsEnded()
{
   bool lIsEnded = true;

   for(int lIdx = 0; lIdx < MAX_WAV_FILES && lIsEnded; lIdx++)
   {
      if(mapWavFile[lIdx] != nullptr && !mapWavFile[lIdx]->IsEnded())
      {
         lIsEnded = false;
      }
   }

   return lIsEnded;
}

void I2SWavPlayer::SetVolume(float aVolume)
{
   if(aVolume <= 0.0)
   {
      mVolume = 0.0;
   }
   else if(aVolume >= 1.0)
   {
      mVolume = 1.0;
   }
   else
   {
      mVolume = aVolume;
   }
}

void I2SWavPlayer::SetMixingMode(EMixingMode aMode)
{
   mMixingStrategy = aMode;
}

void I2SWavPlayer::ClipSample(int32_t& arSample)
{
   if(arSample > INT16_MAX)
   {
      arSample = INT16_MAX;
   }
   else if(arSample < INT16_MIN)
   {
      arSample = INT16_MIN;
   }
}

int I2SWavPlayer::Mix(int32_t& arSampleOut)
{
   int lSamplesCounter = 0;

   int32_t lMixedSample32 = 0;
   for(int lWavFileIdx = 0; lWavFileIdx < MAX_WAV_FILES; lWavFileIdx++)
   {
      ISDWavFile* lpCurFilePtr = mapWavFile[lWavFileIdx];
      if(lpCurFilePtr != nullptr
            && !lpCurFilePtr->IsPaused()
            && !lpCurFilePtr->IsEnded())
      {
         int16_t lCurSample = 0;

         //Handle down-sampling
         if(maDownsampleParams[lWavFileIdx].mIsDownsample && lpCurFilePtr->Available() >= sizeof(int32_t))
         {
            //Double-fetch to skip a sample if we have at least
            //32 bytes (2 16-bit samples)left to read
            lpCurFilePtr->Skip16BitSamples(1);
            lpCurFilePtr->Fetch16BitSamples(&lCurSample, 1);
         }
         else
         {
            //Fetch sample from current file
            lpCurFilePtr->Fetch16BitSamples(&lCurSample, 1);
         }

         //Mix new sample with already collected samples
         lMixedSample32 += (int32_t)lCurSample;

         //Keep track of how many valid samples we read
         lSamplesCounter++;

      }
   }

   arSampleOut = lMixedSample32;
   return lSamplesCounter;
}

int I2SWavPlayer::MixAlt(int32_t& arSampleOutLeft, int32_t& arSampleOutRight)
{
   int lSamplesCounter = 0;

   int32_t lMixedSampleLeft32 = 0;
   for(int lWavFileIdx = 0; lWavFileIdx < MAX_WAV_FILES; lWavFileIdx += 2)
   {
      ISDWavFile* lpCurFilePtr = mapWavFile[lWavFileIdx];
      if(lpCurFilePtr != nullptr
            && !lpCurFilePtr->IsPaused()
            && !lpCurFilePtr->IsEnded())
      {
         int16_t lCurSample = 0;

         //Handle down-sampling
         if(maDownsampleParams[lWavFileIdx].mIsDownsample && lpCurFilePtr->Available() >= sizeof(int32_t))
         {
            //Double-fetch to skip a sample if we have at least
            //32 bytes (2 16-bit samples)left to read
            lpCurFilePtr->Skip16BitSamples(1);
            lpCurFilePtr->Fetch16BitSamples(&lCurSample, 1);
         }
         else
         {
            //Fetch sample from current file
            lpCurFilePtr->Fetch16BitSamples(&lCurSample, 1);
         }

         //Mix new sample with already collected samples
         lMixedSampleLeft32 += (int32_t)lCurSample;

         //Keep track of how many valid samples we read
         lSamplesCounter++;

      }
   }

   //Fetch 16-bit samples from odd-numbered files to create right channel
   int32_t lMixedSampleRight32 = 0;
   for(int lWavFileIdx = 1; lWavFileIdx < MAX_WAV_FILES; lWavFileIdx += 2)
   {
      ISDWavFile* lpCurFilePtr = mapWavFile[lWavFileIdx];
      if(lpCurFilePtr != nullptr
            && !lpCurFilePtr->IsPaused()
            && !lpCurFilePtr->IsEnded())
      {
         //Fetch sample from current file
         int16_t lCurSample = 0;
         //Handle down-sampling
         if(maDownsampleParams[lWavFileIdx].mIsDownsample)
         {
            //Double-fetch to skip a sample
            if(lpCurFilePtr->Available() >= sizeof(int32_t))
            {
               lpCurFilePtr->Fetch16BitSamples(&lCurSample, 1);
               lpCurFilePtr->Fetch16BitSamples(&lCurSample, 1);
            }
         }
         else
         {
            lpCurFilePtr->Fetch16BitSamples(&lCurSample, 1);
         }

         //Mix new sample with already collected samples
         lMixedSampleRight32 += (int32_t)lCurSample;

         //Keep track of how many valid samples we read
         lSamplesCounter++;
      }
   }

   arSampleOutLeft = lMixedSampleLeft32;
   arSampleOutRight = lMixedSampleRight32;
   return lSamplesCounter;
}

void I2SWavPlayer::GenerateMixedI2SSample(int32_t& arSampleOut)
{

   int lSamplesCounter = 0;

   int32_t lMixedSampleLeft32 = 0;
   int32_t lMixedSampleRight32 = 0;

   //Mixing
   switch(mMixingStrategy)
   {
      case eeMixingModeLeftOnly:
         lSamplesCounter = Mix(lMixedSampleLeft32);
         break;
      case eeMixingModeRightOnly:
         lSamplesCounter = Mix(lMixedSampleRight32);
         break;
      case eeMixingModeStereo:
         lSamplesCounter = Mix(lMixedSampleLeft32); //Mix into left channel
         lMixedSampleRight32 = lMixedSampleLeft32; //Copy data to right channel
         break;
      case eeMixingModeAlternating:
      default: //Use Alternating mixing strategy as default
         lSamplesCounter = MixAlt(lMixedSampleLeft32, lMixedSampleRight32);
         break;
   }

   //Clipping
   ClipSample(lMixedSampleLeft32);
   ClipSample(lMixedSampleRight32);

   //Volume control
   if(mVolume < 1.0)
   {
      lMixedSampleLeft32 *= mVolume;
      lMixedSampleRight32 *= mVolume;
   }

   //Keep track of how many files were able to get samples from
   mSamplesMixed = lSamplesCounter;

   //16-bit left and right channel samples
   int16_t lMixedSample16Left = (int16_t)lMixedSampleLeft32;
   int16_t lMixedSample16Right = (int16_t)lMixedSampleRight32;

   //Pointer so we can address 32-bit I2S word 16-bits at a time
   int16_t* lSampleOut16Ptr = (int16_t*)&arSampleOut;

   //Copy left and right channel data into output to create 32-bit I2S word
   memcpy(lSampleOut16Ptr, &lMixedSample16Left, sizeof(int16_t));
   memcpy(lSampleOut16Ptr+1, &lMixedSample16Right, sizeof(int16_t));
}

//void I2SWavPlayer::GenerateMixedI2SSample(int32_t& arSampleOut)
//{
//
//   int lSamplesCounter = 0;
//
//   //Fetch 16-bit samples from even-numbered files to create left channel
//   int32_t lMixedSampleLeft32 = 0;
//   for(int lWavFileIdx = 0; lWavFileIdx < MAX_WAV_FILES; lWavFileIdx += 2)
//   {
//      ISDWavFile* lpCurFilePtr = mapWavFile[lWavFileIdx];
//      if(lpCurFilePtr != nullptr
//            && !lpCurFilePtr->IsPaused()
//            && !lpCurFilePtr->IsEnded())
//      {
//         int16_t lCurSample = 0;
//
//         //Handle down-sampling
//         if(maDownsampleParams[lWavFileIdx].mIsDownsample && lpCurFilePtr->Available() >= sizeof(int32_t))
//         {
//            //Double-fetch to skip a sample if we have at least
//            //32 bytes (2 16-bit samples)left to read
//            lpCurFilePtr->Skip16BitSamples(1);
//            lpCurFilePtr->Fetch16BitSamples(&lCurSample, 1);
//         }
//         else
//         {
//            //Fetch sample from current file
//            lpCurFilePtr->Fetch16BitSamples(&lCurSample, 1);
//         }
//
//         //Mix new sample with already collected samples
//         lMixedSampleLeft32 += (int32_t)lCurSample;
//
//         //Keep track of how many valid samples we read
//         lSamplesCounter++;
//
//      }
//   }
//
//   //Fetch 16-bit samples from odd-numbered files to create right channel
//   int32_t lMixedSampleRight32 = 0;
//   for(int lWavFileIdx = 1; lWavFileIdx < MAX_WAV_FILES; lWavFileIdx += 2)
//   {
//      ISDWavFile* lpCurFilePtr = mapWavFile[lWavFileIdx];
//      if(lpCurFilePtr != nullptr
//            && !lpCurFilePtr->IsPaused()
//            && !lpCurFilePtr->IsEnded())
//      {
//         //Fetch sample from current file
//         int16_t lCurSample = 0;
//         //Handle down-sampling
//         if(maDownsampleParams[lWavFileIdx].mIsDownsample)
//         {
//            //Double-fetch to skip a sample
//            if(lpCurFilePtr->Available() >= sizeof(int32_t))
//            {
//               lpCurFilePtr->Fetch16BitSamples(&lCurSample, 1);
//               lpCurFilePtr->Fetch16BitSamples(&lCurSample, 1);
//            }
//         }
//         else
//         {
//            lpCurFilePtr->Fetch16BitSamples(&lCurSample, 1);
//         }
//
//         //Mix new sample with already collected samples
//         lMixedSampleRight32 += (int32_t)lCurSample;
//
//         //Keep track of how many valid samples we read
//         lSamplesCounter++;
//      }
//   }
//   //Clipping
//   ClipSample(lMixedSampleLeft32);
//   ClipSample(lMixedSampleRight32);
//
//   //Volume control
//   if(mVolume < 1.0)
//   {
//      lMixedSampleLeft32 *= mVolume;
//      lMixedSampleRight32 *= mVolume;
//   }
//
//   //Keep track of how many files were able to get samples from
//   mSamplesMixed = lSamplesCounter;
//
//   //16-bit left and right channel samples
//   int16_t lMixedSample16Left = (int16_t)lMixedSampleLeft32;
//   int16_t lMixedSample16Right = (int16_t)lMixedSampleRight32;
//
//   //Pointer so we can address 32-bit I2S word 16-bits at a time
//   int16_t* lSampleOut16Ptr = (int16_t*)&arSampleOut;
//
//   //Copy left and right channel data into output to create 32-bit I2S word
//   memcpy(lSampleOut16Ptr, &lMixedSample16Left, sizeof(int16_t));
//   memcpy(lSampleOut16Ptr+1, &lMixedSample16Right, sizeof(int16_t));
//}

int I2SWavPlayer::PopulateMixingBuffer()
{
   for(int lIdx = 0; lIdx < I2S_BUF_SIZE; lIdx++)
   {
      GenerateMixedI2SSample(maMixedI2SSamples[lIdx]);
   }

   mAudioSampleValue=(maMixedI2SSamples[0] + maMixedI2SSamples[I2S_BUF_SIZE-1] + maMixedI2SSamples[I2S_BUF_SIZE/2] + maMixedI2SSamples[I2S_BUF_SIZE/2 + 1]);

   return 0;
}

void I2SWavPlayer::Configure_I2S()
{
   // register structure hierarchy for I2S
   // NRF_I2S is of type NRF_I2S_Type defined in nrf52.h
   // CONFIG is of type I2S_CONFIG_Type defined in nrf52.h, sub-struct of NRF_I2S_Type
   // Struct -> Element (kinda Struct.Element)

   // Position variables (_Pos) are defined in nrf52_bitfields.h

#if defined(NRF52) || defined(NRF52_SERIES)
   // Enable Tx transmission
   NRF_I2S->CONFIG.TXEN = (I2S_CONFIG_TXEN_TXEN_ENABLE << I2S_CONFIG_TXEN_TXEN_Pos);

   // Enable MCK generator
   NRF_I2S->CONFIG.MCKEN = (I2S_CONFIG_MCKEN_MCKEN_ENABLE << I2S_CONFIG_MCKEN_MCKEN_Pos);
   // 16/24/32-bit  resolution, the MAX98357A supports I2S timing only!
   // Master mode, 16Bit, left aligned
   NRF_I2S->CONFIG.MODE = I2S_CONFIG_MODE_MODE_MASTER << I2S_CONFIG_MODE_MODE_Pos;
   //	// set the sample rate to a value supported by the audio amp
   //	// LRCLK  ONLY  supports  8kHz,  16kHz,  32kHz,  44.1kHz,  48kHz, 88.2kHz, and 96kHz frequencies.
   //	// LRCLK clocks at  11.025kHz,  12kHz,  22.05kHz  and  24kHz  are  NOT supported.
   //
   //	// look for /* Register: I2S_CONFIG_MCKFREQ */ in nrf52_bitfields.h
   //	// MCKFREQ = 4 MHz
   //	//+++NRF_I2S->CONFIG.MCKFREQ =  I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV11 << I2S_CONFIG_MCKFREQ_MCKFREQ_Pos;
   //	//NRF_I2S->CONFIG.MCKFREQ =  I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV23 << I2S_CONFIG_MCKFREQ_MCKFREQ_Pos;
   //	NRF_I2S->CONFIG.MCKFREQ =  I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV10 << I2S_CONFIG_MCKFREQ_MCKFREQ_Pos;
   //
   //	// look for /* Register: I2S_CONFIG_RATIO */ in nrf52_bitfields.h
   //	// set 16kHz as default value -> Ration = 256 (4MHz/256=16kHz)
   //	//+++NRF_I2S->CONFIG.RATIO = I2S_CONFIG_RATIO_RATIO_128X << I2S_CONFIG_RATIO_RATIO_Pos;
   //	NRF_I2S->CONFIG.RATIO = I2S_CONFIG_RATIO_RATIO_32X << I2S_CONFIG_RATIO_RATIO_Pos;
   // look for /* Register: I2S_CONFIG_SWIDTH */ in nrf52_bitfields.h
   // 16 bit
   NRF_I2S->CONFIG.SWIDTH = I2S_CONFIG_SWIDTH_SWIDTH_16BIT << I2S_CONFIG_SWIDTH_SWIDTH_Pos;

   // Left-aligned (not to be mixed up with left-justified)
   NRF_I2S->CONFIG.ALIGN = I2S_CONFIG_ALIGN_ALIGN_Left << I2S_CONFIG_ALIGN_ALIGN_Pos;

   // Format I2S (i.e. not left justified)
   NRF_I2S->CONFIG.FORMAT = I2S_CONFIG_FORMAT_FORMAT_I2S << I2S_CONFIG_FORMAT_FORMAT_Pos;

   // Use mono
   NRF_I2S->CONFIG.CHANNELS = I2S_CONFIG_CHANNELS_CHANNELS_Stereo << I2S_CONFIG_CHANNELS_CHANNELS_Pos;
   //NRF_I2S->CONFIG.CHANNELS = I2S_CONFIG_CHANNELS_CHANNELS_Left << I2S_CONFIG_CHANNELS_CHANNELS_Pos;

   // configure the pins
   NRF_I2S->PSEL.MCK = (mPinMCK << I2S_PSEL_MCK_PIN_Pos);
   NRF_I2S->PSEL.SCK = (mPinBCLK << I2S_PSEL_SCK_PIN_Pos);
   NRF_I2S->PSEL.LRCK = (mPinLRCK << I2S_PSEL_LRCK_PIN_Pos);
   NRF_I2S->PSEL.SDOUT = (mPinDIN << I2S_PSEL_SDOUT_PIN_Pos);

   // Enable the I2S module using the ENABLE register
   NRF_I2S->ENABLE = 1;
#elif defined(ESP32)
   // configure the ESP32's I2S interface
   m_i2s_config.bits_per_sample					= I2S_BITS_PER_SAMPLE_16BIT;
   m_i2s_config.channel_format					= I2S_CHANNEL_FMT_RIGHT_LEFT;
   // memory (SRAM) allocated to I2S buffer: (bits_per_sample/8)*channels*dma_buf_count*dmu_buf_len
   m_i2s_config.dma_buf_count					= 2; // between 2 and 128 (see error code 283)
   m_i2s_config.dma_buf_len						= I2S_BUF_SIZE*2; // number of I2S samples, multiplied by 2 due to 16bit samples stored on 32bit (L/R)
   m_i2s_config.fixed_mclk						= I2S_PIN_NO_CHANGE;
   m_i2s_config.intr_alloc_flags				= ESP_INTR_FLAG_LEVEL1; // interrupt priority
   m_i2s_config.sample_rate						= 16000;
   m_i2s_config.tx_desc_auto_clear				= true;   // new in V1.0.1
   m_i2s_config.use_apll						= APLL_DISABLE;
   m_i2s_config.mode             				= (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
#if ESP_ARDUINO_VERSION_MAJOR >= 2
    m_i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_STAND_I2S); // Arduino vers. > 2.0.0
#else
    // standard I2S format (i.e. not left-justified) means data transmission starts one BCLK cycle after LRCLK transition
    // MAX98357A needs I2S format and MSB first
    m_i2s_config.communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB);
#endif


    m_i2s_returncode = i2s_driver_uninstall((i2s_port_t)I2S_NUM_0);
    log_d("I2S return code driver uninstall: %d", m_i2s_returncode);
   // i2s_driver_install will automatically start the I2S driver
    m_i2s_returncode = i2s_driver_install  ((i2s_port_t)I2S_NUM_0, &m_i2s_config, 0, NULL);
    log_d("I2S return code driver install: %d", m_i2s_returncode);
#endif


   Configure_I2S_Speed(ee2205); //Default I2S speed

   pinMode (mPinSD, OUTPUT);
   digitalWrite (mPinSD, HIGH);
}

int8_t I2SWavPlayer::GetAudioSampleValue()
{
	int8_t lSample;

	lSample=mAudioSampleValue;
	return lSample;
}
