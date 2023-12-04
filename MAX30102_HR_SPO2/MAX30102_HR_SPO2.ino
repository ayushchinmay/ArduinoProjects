// I2C OLED
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

#define OLED_RESET     4
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#include "MAX30105.h"
#include "heartRate.h"

MAX30105 particleSensor;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
/**************************/

static const unsigned char PROGMEM hr1[] = {
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x3E, 0x00, 0x01, 0xFC, 0x7F, 0x00, 0x03, 0xFE, 0xFF, 0x80,
0x07, 0xFF, 0xFF, 0xC0, 0x07, 0xFF, 0xFF, 0xC0, 0x07, 0xF7, 0xDF, 0xC0, 0x07, 0xE3, 0x8F, 0xC0,
0x03, 0x2B, 0xA9, 0x80, 0x01, 0xC9, 0x27, 0x00, 0x00, 0xFD, 0x7E, 0x00, 0x00, 0x7C, 0x7C, 0x00,
0x00, 0x3E, 0xF8, 0x00, 0x00, 0x1F, 0xF0, 0x00, 0x00, 0x0F, 0xE0, 0x00, 0x00, 0x07, 0xC0, 0x00,
0x00, 0x03, 0x80, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, };

static const unsigned char PROGMEM hr2[] = {
0x00, 0x00, 0x00, 0x00, 0x03, 0xF0, 0x1F, 0x80, 0x0F, 0xF8, 0x3F, 0xE0, 0x1F, 0xFC, 0x7F, 0xF0,
0x3F, 0xFE, 0xFF, 0xF8, 0x3F, 0xF7, 0xEF, 0xF8, 0x7F, 0xF7, 0xEF, 0xFC, 0x7F, 0xE3, 0xC7, 0xFC,
0x7F, 0xEB, 0xD7, 0xFC, 0x7F, 0xEB, 0xD7, 0xFC, 0x7F, 0xCB, 0x97, 0xFC, 0x38, 0xDB, 0xB4, 0x38,
0x1E, 0x5B, 0x31, 0xF0, 0x0F, 0x19, 0x7B, 0xE0, 0x07, 0xBD, 0x7F, 0xC0, 0x03, 0xFD, 0x7F, 0x80,
0x01, 0xFC, 0x7F, 0x00, 0x00, 0xFE, 0xFE, 0x00, 0x00, 0x7E, 0xFC, 0x00, 0x00, 0x3F, 0xF8, 0x00,
0x00, 0x1F, 0xF0, 0x00, 0x00, 0x0F, 0xE0, 0x00, 0x00, 0x07, 0xC0, 0x00, 0x00, 0x03, 0x80, 0x00,
0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
};
/**************************/

void setup() {
  Serial.begin(115200);

  // I2C OLED SETUP
  Serial.println("I2C OLED Test!");
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.display();
  delay(2000);

  display.clearDisplay();
  display.display();
  delay(1000);

  // DISPLAY TEXT
  display.setTextSize(1);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.setCursor(0, 0);     // Start at top-left corner
//  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}
/**************************/


void loop(){
  long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true){
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20){
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);

  display.clearDisplay();
  display.setCursor(0,0);
  display.drawBitmap(0,4,hr2,32,26,SSD1306_WHITE);

  display.setCursor(36,8);
  display.print("HR: ");
  display.setTextSize(2);
  display.print(irValue/1000.0);
  display.setTextSize(1);
  display.display();

  if (irValue < 50000){
    Serial.print(" No finger?");
    display.fillRect(0, 0, 32, 26, SSD1306_BLACK);
    display.drawBitmap(0,4,hr1,32,26,SSD1306_WHITE);
    display.display();
  }
  Serial.println();
}
