/*
 * I2SPlay.c
 *
 *  Created on: Apr 7, 2022
 *      Author: Jake
 */

#include <hal/i2s_types.h>
#include <driver/i2s.h>

static const i2s_port_t i2s_num = 0; // i2s port number
static i2s_config_t gI2SMasterCfg;
static i2s_pin_config_t gI2SMasterPinCfg;

void I2SPlayConfigI2S(int aMckPin,
	                  int aBckPin,
	                  int aWsPin,
	                  int aDataOutPin,
					  int aDataLength)
{

//    i2s_mode_t              mode;                       /*!< I2S work mode */
//    uint32_t                sample_rate;                /*!< I2S sample rate */
//    i2s_bits_per_sample_t   bits_per_sample;            /*!< I2S sample bits in one channel */
//    i2s_channel_fmt_t       channel_format;             /*!< I2S channel format.*/
//    i2s_comm_format_t       communication_format;       /*!< I2S communication format */
//    int                     intr_alloc_flags;           /*!< Flags used to allocate the interrupt. One or multiple (ORred) ESP_INTR_FLAG_* values. See esp_intr_alloc.h for more info */
//    int                     dma_buf_count;              /*!< I2S DMA Buffer Count */
//    int                     dma_buf_len;                /*!< I2S DMA Buffer Length */
//    bool                    use_apll;                   /*!< I2S using APLL as main I2S clock, enable it to get accurate clock */
//    bool                    tx_desc_auto_clear;         /*!< I2S auto clear tx descriptor if there is underflow condition (helps in avoiding noise in case of data unavailability) */
//    int                     fixed_mclk;                 /*!< I2S using fixed MCLK output. If use_apll = true and fixed_mclk > 0, then the clock output for i2s is fixed and equal to the fixed_mclk value. If fixed_mclk set, mclk_multiple won't take effect */
//    i2s_mclk_multiple_t     mclk_multiple;              /*!< The multiple of I2S master clock(MCLK) to sample rate */
//    i2s_bits_per_chan_t     bits_per_chan;

    gI2SMasterCfg.mode = I2S_MODE_MASTER | I2S_MODE_TX;
    gI2SMasterCfg.sample_rate = 22050;
    gI2SMasterCfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    gI2SMasterCfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    gI2SMasterCfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    gI2SMasterCfg.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
    gI2SMasterCfg.dma_buf_count = 8;
    if(aDataLength <= 1024)
    {
    	gI2SMasterCfg.dma_buf_len = aDataLength;
    }
    else
    {
    	gI2SMasterCfg.dma_buf_len = 1024; //1024 max
    }
    gI2SMasterCfg.use_apll = false;

	i2s_driver_install(i2s_num, &gI2SMasterCfg, 0, NULL);

	//Set pins
	gI2SMasterPinCfg.mck_io_num = aMckPin;
	gI2SMasterPinCfg.bck_io_num = aBckPin;
	gI2SMasterPinCfg.ws_io_num = aWsPin;
	gI2SMasterPinCfg.data_out_num = aDataOutPin;
	gI2SMasterPinCfg.data_in_num = I2S_PIN_NO_CHANGE;


	i2s_set_pin(i2s_num, &gI2SMasterPinCfg);
}

size_t I2SPlayI2SWrite(int32_t* aBuffer, size_t aBufferSize)
{
//	i2s_port_t i2s_num,
//	const void *src,
//	size_t size,
//	size_t *bytes_written,
//	TickType_t ticks_to_wait

	size_t lBytesWritten = 0;
	i2s_write(i2s_num, (void*)aBuffer, sizeof(int32_t), &lBytesWritten, portMAX_DELAY);

	return lBytesWritten;
}
