#include <Arduino.h>
#include <vector>
#include <WiFi.h>
#include <Wire.h>
#include "SPI.h"
#include "Adafruit_GFX.h"  //for matrix led
#include "Adafruit_LEDBackpack.h"

#include "KB_initBoard.h"
#include "KB_music.h"
#include "KB_LDR.h"
#include "KB_LM73.h"
#include "KB_ht16k33.h"
#include "MCP7941x.h"
#include "Kalman.h"
#include "TFT_eSPI.h"  //for matrix led

TFT_eSPI tft = TFT_eSPI();
MCP7941x rtc = MCP7941x();
Kalman mpu;
KB_board board = KB_board();
KB_music music = KB_music();
KB_LDR ldr = KB_LDR();
KB_LM73 lm73 = KB_LM73();
KB_8x16Matrix matrix = KB_8x16Matrix();

typedef int Number;
typedef int Boolean;
using namespace std;

float X;
float Y;

void setup() {
  board.begin();
  music.begin();
  lm73.begin();
  matrix.displayBegin();
  ldr.begin();

  Serial.begin(115200);
  tft.setmode(1);
  tft.fillScreen(0x0);

  mpu.begin();
}
void loop() {
  // tft.setTextFont(GLCD);
  tft.setTextSize(4);
  tft.setCursor(0, 0);
  tft.setTextColor(0x0);
  tft.println(X);
  // tft.setTextFont(GLCD);
  tft.setTextSize(4);
  tft.setCursor(0, 40);
  tft.setTextColor(0x0);
  tft.println(Y);
  X = (float)mpu.readAngle(0);
  Y = (float)mpu.readAngle(1);
  // tft.setTextFont(GLCD);
  tft.setTextSize(4);
  tft.setCursor(0, 0);
  tft.setTextColor(0xf800);
  tft.println(X);
  // tft.setTextFont(GLCD);
  tft.setTextSize(4);
  tft.setCursor(0, 40);
  tft.setTextColor(0xffec);
  tft.println(Y);
  Serial.print(X);
  Serial.print(String(","));
  Serial.println(Y);
  delay(20);
}