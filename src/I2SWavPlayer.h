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
 * I2SWavPlayer.h
 *
 *  Created on: Jun 30, 2019
 *      Author: JakeSoft
 */

#ifndef I2SWAVPLAYER_H_
#define I2SWAVPLAYER_H_

#include "Arduino.h"
#include "ISDWavFile.h"
#if defined(ESP32)
	#include <driver/i2s.h>
#endif
//I2S buffer size
#define I2S_BUF_SIZE 512

//Maximum concurrent wav files
#define MAX_WAV_FILES 5

//Default Pins
#if defined(NRF52) || defined(NRF52_SERIES)
	#define PIN_I2S_MCK_DEFAULT 13
	#define PIN_I2S_BCLK_DEFAULT (A2)
	#define PIN_I2S_LRCK_DEFAULT (A3)
	#define PIN_I2S_DIN_DEFAULT 18
	#define PIN_I2S_SD_DEFAULT  10
#elif defined(ESP32)
	#define PIN_I2S_MCK_DEFAULT 100 // dummy definition, not needed for ESP32 platform
	#define PIN_I2S_BCLK_DEFAULT 27
	#define PIN_I2S_LRCK_DEFAULT 26
	#define PIN_I2S_DIN_DEFAULT 25
	#define PIN_I2S_SD_DEFAULT  101 // dummy definition, todo: to be defined later
#endif

enum ESampleRate
{
	ee2205,
	ee4410
};

enum EMixingMode
{
   eeMixingModeAlternating, //Odd numbered channels played on right while even played on left
   eeMixingModeRightOnly,   //All sounds mixed to right channel
   eeMixingModeLeftOnly,    //All sounds mixed to left channel
   eeMixingModeStereo       //All sounds mixed to both channels
};

/**
 * This class facilities basic wav file playback via I2S. It does on-the-fly
 * mixing of mutilple channels to create a single I2S stream from potentially
 * multiple files. Performance such as how many files can be played at once will
 * depend on I2S speed, number of simultaneous files, and raw CPU processing power.
 */
class I2SWavPlayer
{
public:
	/**
	 * Constructor.
	 *
	 * Args:
	 *   aPinMCK - Pin for master clock
	 *   aPinBCLK - Pin for bit clock
	 *   aPinLRCD - Pin for Left/Right clock
	 *   aPinDIN - Pin for data out
	 *   aPinSD - Pin for SD
	 */
	I2SWavPlayer(int32_t aPinMCK = PIN_I2S_MCK_DEFAULT,
			     int32_t aPinBCLK = PIN_I2S_BCLK_DEFAULT,
				 int32_t aPinLRCK = PIN_I2S_LRCK_DEFAULT,
				 int32_t aPinDIN = PIN_I2S_DIN_DEFAULT,
				 int32_t aPinSD = PIN_I2S_SD_DEFAULT);

	/**
	 * Destructor.
	 */
	~I2SWavPlayer();

	/**
	 * Initializes pins and hardware.
	 * Call this before starting playback.
	 */
	bool Init();

	/**
	 * Sets wav file to play.
	 * Args:
	 *   apWavFile - Pointer to SDWavFile object to play
	 *   aFileIndex - (optional) In the case of polyphonic playback,
	 *                used to enumerate the file being provided
	 */
	void SetWavFile(ISDWavFile* apWavFile, int aFileIndex = 0);

	/**
	 * Removes wave files from all channels. (sets them to null)
	 * and clears the I2S data buffers.
	 */
	void ClearAllWavFiles();

	/**
	 * Starts playback.
	 */
	void StartPlayback();

	/**
	 * Stops playback.
	 */
	void StopPlayback();

	/**
	 * Continues fetching data from the WAV files and sending I2S data
	 * via the I/O pins. Call this repeatedly in a loop or with a timer
	 * interrupt to keep playback going. Failure to call this frequently
	 * enough will cause gaps in the playback.
	 */
	bool ContinuePlayback();

	/**
	 * Indicates if all files have finished playing.
	 * Returns: TRUE if all files have ended playback, FALSE otherwise
	 */
	bool IsEnded();

	/**
	 * Sets the I2S clock speed.
	 * Args:
	 *  aSampleRate - ee2205 = 22.05 KHz
	 *                ee4410 = 44.1 KHz
	 */
	inline void Configure_I2S_Speed(ESampleRate aSampleRate)
	{
		mSampleRate = aSampleRate;

		// set the sample rate to a value supported by the audio amp
		// LRCLK  ONLY  supports  8kHz,  16kHz,  32kHz,  44.1kHz,  48kHz, 88.2kHz, and 96kHz frequencies.
		// LRCLK clocks at  11.025kHz,  12kHz,  22.05kHz  and  24kHz  are  NOT supported.
		switch (aSampleRate)
		{
		case ee2205: //LRCLK at 44.1kHz (for playback speed of 22.05kHz)
			#if defined(NRF52) || defined(NRF52_SERIES)
				NRF_I2S->CONFIG.MCKFREQ =  I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV11 << I2S_CONFIG_MCKFREQ_MCKFREQ_Pos;
				NRF_I2S->CONFIG.RATIO = I2S_CONFIG_RATIO_RATIO_128X << I2S_CONFIG_RATIO_RATIO_Pos;
			#elif defined(ESP32)
				m_i2s_config.sample_rate						= 22050;
			#endif
				break;
		case ee4410: //LRCLK at 88.1kHz (for playback speed of 44.1kHz)
			#if defined(NRF52) || defined(NRF52_SERIES)
				NRF_I2S->CONFIG.MCKFREQ =  I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV11 << I2S_CONFIG_MCKFREQ_MCKFREQ_Pos;
				NRF_I2S->CONFIG.RATIO = I2S_CONFIG_RATIO_RATIO_64X << I2S_CONFIG_RATIO_RATIO_Pos;
			#elif defined(ESP32)
				m_i2s_config.sample_rate						= 44100;
			#endif
			break;
		default:
			Configure_I2S_Speed(ee2205);
			break;
		}
	}

	/**
	 * Set master volume.
	 * Args:
	 *  aVolume - Any value between 0.0 (mute) and 1.0 (full volume)
	 */
	void SetVolume(float aVolume);

	/**
	 * Sets the mixing mode for polyphonic playback.
	 * Let's user decide if sound should go all to one channel
	 * or be mixed to both.
	 * Args:
	 *  aMode - Mixing mode to use
	 *
	 */
	void SetMixingMode(EMixingMode aMode);

	/**
		 * Fetch pointer to the mixed samples from the last update cycle.
		 * Returns: Pointer to base of the mixing buffer
		 */
		inline int32_t* GetMixedSampleBuffer()
		{
		  return maMixedI2SSamples;
		}

		/**
		 * Fetch size of the I2S buffers
		 * Returns: Size of the internal I2S buffers
		 */
		inline int GetI2SSize()
		{
		  return I2S_BUF_SIZE;
		}

	/**
	 * Fetches the average of a few selected samples of the mixed sample buffer currently played.
	 * Args:
	 *
	 *
	 */
	int8_t GetAudioSampleValue();
#if defined(ESP32)
private:
    enum : int { APLL_AUTO = -1, APLL_ENABLE = 1, APLL_DISABLE = 0 };

    i2s_config_t          m_i2s_config = {}; // stores values for I2S driver
    i2s_pin_config_t      m_pin_config = {};
    esp_err_t			  m_i2s_returncode = 0;
#endif

protected:

	/**
	 * Sets up I2S playback parameters with the hardware.
	 */
	void Configure_I2S();

	/**
	 * Mix samples from all open files into one sample.
	 * Note: Used for Left-only or Right-only mixing strategies
	 * Args:
	 *  arSampleOut - Sample output to put result
	 *  Returns: Number of samples mixed
	 */
	int Mix(int32_t& arSampleOut);

	/**
	 * Mix samples in L/R alternate fashion.
	 * Args:
	 *  arSampleLeft
	 *  Returns: Number of samples mixed
	 */
	int MixAlt(int32_t& arSampleOutLeft, int32_t& arSampleOutRight);

	/**
	 * Creates a mixed 32-bit I2S word from all opened files.
	 * Args:
	 *   arSampleOut - Reference to variable to hold output
	 */
	void GenerateMixedI2SSample(int32_t& arSampleOut);

	/**
	 * Fills the mixing buffer with data from all open files.
	 */
	int PopulateMixingBuffer();

	/**
	 * Clips a sample to a 16-bit value. Used to compensate for overshoot
	 * during the mixing process.
	 * Args:
	 *   arSample - Sample to clip
	 */
	void ClipSample(int32_t& arSample);

	//Pins
	int32_t mPinMCK;
	int32_t mPinBCLK;
	int32_t mPinLRCK;
	int32_t mPinDIN;
	int32_t mPinSD;

	//I2S sample buffers
#if defined(NRF52) || defined(NRF52_SERIES)
	int32_t maBufferA[I2S_BUF_SIZE] = {};
	int32_t maBufferB[I2S_BUF_SIZE] = {};
	int32_t maMixedI2SSamples[I2S_BUF_SIZE] = {};
#elif defined(ESP32)
	int32_t maBufferA[I2S_BUF_SIZE] = {};
	int32_t maBufferB[I2S_BUF_SIZE] = {};
	int32_t maMixedI2SSamples[I2S_BUF_SIZE] = {};
#endif
	//Keep track of if buffer A or B is selected
	bool mBufferASelected;

	//Pointers to WAV file object to play
	ISDWavFile* mapWavFile[MAX_WAV_FILES];

	struct tDownSampleParameters
	{
		//Keep track of if we should down-sample each wave file to
		//meet the configured playback rate
		bool mIsDownsample = false;
	};

	//Keep track of if we should down-sample for each wav file
	//This allows for playback of files at the proper rate even when
	//the bit rate of the file is not the same as the native I2S playback speed
	//of the CPU
	tDownSampleParameters maDownsampleParams[MAX_WAV_FILES];

	//Keep track of number of samples mixed during last mixing calculation
	int mSamplesMixed;

	//Configured sample rate
	ESampleRate mSampleRate;

	//Master volume control
	float mVolume;

	//Mixing strategy for polyphonic playback
	EMixingMode mMixingStrategy;

	//First sample of mixed buffer
	int8_t mAudioSampleValue;

};

#endif /* I2SWAVPLAYER_H_ */
