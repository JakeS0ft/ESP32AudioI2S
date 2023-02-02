#include "I2S_Audio.h"
#include "Arduino.h"

#define PIN_I2S_MCK 13
#define PIN_I2S_BCLK (A2) // A4
#define PIN_I2S_LRCK (A3) // A5
#define PIN_I2S_DIN 18 // A6
#define PIN_I2S_SD  10 // 27
#define PIN_SPI_CS  11
#define PIN_SCL 16
#define PIN_SDA 15

// Digital I/O used
#define SD_CS          5
#define SPI_MOSI      23
#define SPI_MISO      19
#define SPI_SCK       18
#define I2S_DOUT      25
#define I2S_BCLK      27
#define I2S_LRC       26
#define I2S_SD		  28
//#define I2S_MCK       28  // dummy definition, not needed

//Plays a single wave file until it ends
void PlayWavFile()
{

	long lTimeStamp = 0;

	Serial.println("Entering function PlayWavFile");
	//Wav files to play. Change "22CANT1.WAV" to match the name of
	//whatever file you are playing.
	SDWavFile* lpWavFile1 = new SDWavFile("/hum01.wav");
//	SDWavFile* lpWavFile1 = new SDWavFile("/additional_info_Testfiles_Pink-Panther.wav");

	Serial.println("Create a new I2S Player");
	//Create a new I2S Player
	I2SWavPlayer* lpPlayer = new I2SWavPlayer(PIN_I2S_MCK_DEFAULT,
										      I2S_BCLK,
											  I2S_LRC,
											  I2S_DOUT,
											  I2S_SD);
	Serial.println("Init Wav Player.");

	lpPlayer->Init();                      //Initializes I2S playback hardware

	lpPlayer->Configure_I2S_Speed(ee2205); //Set I2S clock speed (sample rate of the file)
	                                       //ee2205 = 22.05KHz
	                                       //ee4410 = 44.1KHz

	Serial.println("Specify Wav file to play.");

	lpPlayer->SetWavFile(lpWavFile1);      //Set file object to play
	lpPlayer->SetVolume(0.5);              //set master volume, 0.0 (mute) to 1.0 (full volume)
	Serial.println("Start playback.");
	lpPlayer->StartPlayback();             //Begin playing the wave file

	Serial.println("Playback started.");

	//Keep playing as long as playback hasn't ended
	while(false == lpPlayer->ContinuePlayback())
	{
		//Wait for playback to end
		//Serial.println("Playback continued.");
		Serial.println(lTimeStamp = micros() - lTimeStamp);
		lTimeStamp = micros();

	}

	//Cleanup code
	Serial.println("Playback ended.");
	lpPlayer->StopPlayback();
	lpWavFile1->Close();

}

//The setup function is called once at startup of the sketch

void setup()
{
// Add your initialization code here

    pinMode(SD_CS, OUTPUT);      digitalWrite(SD_CS, HIGH);
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);
    SPI.setFrequency(80000000); // has no effect...
	Serial.begin(115200);
	SPI.setClockDivider(2);  // no sure it helps
	Serial.print("SPI clock divider: ");Serial.println(SPI.getClockDivider());
	//Serial.print("SPI clock frequency: ");Serial.println(spi_get_timing(false, 0, 80000000, NULL, 0));
	//Initialize SD card. Make sure to do this before creating any SDWavFile objects or
	//trying to play anything or we won't be able to read the data from the SD card
	if(!SD.begin(SD_CS, SPI, 80000000)) // 8MHz SD read over SPI, freq setting has no effect
	{
		Serial.println("SD init failed.");
		return; //Punt. We can't work without SD card
	}
	else {
		Serial.println("SD init completed.");
        File root = SD.open("/");
        printFiles(root,0);
        uint8_t cardType = SD.cardType();

        if(cardType == CARD_NONE){
          Serial.println("No SD card attached");
          return;
        }

        Serial.print("SD Card Type: ");
        if(cardType == CARD_MMC){
          Serial.println("MMC");
        } else if(cardType == CARD_SD){
          Serial.println("SDSC");
        } else if(cardType == CARD_SDHC){
          Serial.println("SDHC");
        } else {
          Serial.println("UNKNOWN");
        }
        uint64_t cardSize = SD.cardSize() / (1024 * 1024);
         Serial.printf("SD Card Size: %lluMB\n", cardSize);
         Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
         Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
	}

    // put out available SRAM and PSRAM (extension) to Serial Output
      log_d("Total heap: %d", ESP.getHeapSize());
      log_d("Free heap: %d", ESP.getFreeHeap());
      log_d("Total PSRAM: %d", ESP.getPsramSize());
      log_d("Free PSRAM: %d", ESP.getFreePsram());

      log_d("SPI RAM getMinFreePsram (lowest level of free PSRAM since boot) %d", ESP.getMinFreePsram());
      log_d("SPI RAM getMaxAllocPsram (largest block of PSRAM that can be allocated at once) %d",ESP.getMaxAllocPsram());

      log_d("CHIP revision %d", ESP.getChipRevision());
      log_d("CHIP model %s", ESP.getChipModel());
      log_d("CHIP Cores %d", ESP.getChipCores());
      log_d("CYCLE count %d", ESP.getCycleCount());
      log_d("SDK version %s", ESP.getSdkVersion());

      log_d("FLASH chip size %d", ESP.getFlashChipSize());
      log_d("FLASH chip speed %d", ESP.getFlashChipSpeed());
      log_d("FLASH chip mode %d", ESP.getFlashChipMode());

      log_d("SKETCH size %d", ESP.getSketchSize());
      log_d("SKETCH free space %d", ESP.getFreeSketchSpace());

	PlayWavFile();

}

// The loop function is called in an endless loop
void loop()
{
//Add your repeated code here
}

void printFiles(File dir, int numTabs)
{
  while (true)
  {
    File entry =  dir.openNextFile();
    if (! entry)
    {
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++)
    {
      Serial.print('\t');
    }
    Serial.println(entry.name());
    entry.close();
  }
}
