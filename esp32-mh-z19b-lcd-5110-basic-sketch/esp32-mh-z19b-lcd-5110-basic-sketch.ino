/*
Copyright © 2018, github.com/satr
License: MIT

ESP32 with CO2 sensor MH-Z19B (v2: 0-5000 ppm) and the display Nokia 5110
Data are read from the sensor's serial port.
Graph displays a change of the CO2 level on time.

  ESP32        | CO2 sensor MH-Z19B
  pin 16 (RX2) - RX
  pin 17 (TX2) - TX
  GND          - GND
  +5V          - Vin

  ESP32  | Display Nokia 5110
  pin 14 - Serial clock out: CLK (SCLK)
  pin 13 - Serial data out: DIN
  pin 27 - Data/Command select: DC (D/C)
  pin 15 - LCD chip select: CE (CS)
  pin 26 - LCD reset: RST
  5V    - VCC
  GND   - GND
*/

#include <HardwareSerial.h>

#include <Arduino.h>
#include <U8g2lib.h>

#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

 //pins description are above
U8G2_PCD8544_84X48_F_4W_SW_SPI display(U8G2_R0, /* clock=*/ 14, /* data=*/ 13, /* cs=*/ 15, /* dc=*/ 27, /* reset=*/ 26);  // Nokia 5110 Display

HardwareSerial co2SensorSerial(1); // pins description are above

// CO2 sensor data structures:
byte cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79}; 
/* --- Request --- 
 *  0xFF - Start byte
 *  0x01 - Sensor #1
 *  0x86 - Commands: Read CO2 concentration
 *  ...
 *  0x79 - "Checksum is expected in the response"
 *  
 * --- Response ---
 *  0xFF - Start byte
 *  0x01 - Sensor #1
 *  0x** - CO2 concentration value (high byte)
 *  0x** - CO2 concentration value (low byte)
 *  ...
 *  0x** - Checksum value 
 *  
 * --- Other commands ---
 *  0x87 - Calibrate Zero Point (ZERO)
 *  0x88 - Calibrate Span Point (SPAN)
 *  0x79 - ON/OFF Auto CalibrationON/OFF
 *  0x99 - Detection range setting
 */

//Response buffer
unsigned char co2SensorResponse[9]; 

//CO2 concentration levels
const unsigned int MIN_VALUE = 300;
const unsigned int VERY_GOOD_VALUE = 450;
const unsigned int GOOD_VALUE = 600;
const unsigned int ACCEPTABLE_VALUE = 1000;
const unsigned int MAX_VALUE = 2500;

//Display settings
unsigned int goodValueGraphValue = 0;
unsigned int acceptableValueGraphValue = 0;
const byte GRAPH_WIDTH = 84;
const byte GRAPH_HEIGHT = 28;
const unsigned int MAX_GRAPH_VALUE = 1500;
const unsigned int GRAPH_STEP = (MAX_GRAPH_VALUE - MIN_VALUE) / GRAPH_HEIGHT;
unsigned int graphData[GRAPH_WIDTH];
#define BACKCOLOR 0 // White
#define PIXELCOLOR 1  // Black
int displayHeight = 48;
byte graphSpeed = 1;
int graphDrawingCounter = 0;

//Messages
String messages[] = {"", "Very good","Good", "Acceptable","Bad","Health risk", 
                     "CO2 sensor error", "No valid CO2 data", "Initializing"};
const byte MSG_EMPTY = 1;
const byte MSG_VERY_GOOD = 1;
const byte MSG_GOOD = 2;
const byte MSG_ACCEPTABLE = 3;
const byte MSG_BAD = 4;
const byte MSG_HEALTH_RISK = 5;
const byte MSG_SENSOR_ERROR = 6;
const byte MSG_NO_VALID_CO2_DATA = 7;
const byte MSG_INITIALIZING = 8;

//Current state
byte currentMessageId = 0;
unsigned int currentCo2Value = 0;


void setup() {
  Serial.begin(115200);
  co2SensorSerial.begin(9600, SERIAL_8N1, 16, 17);
  initDisplay();
  currentCo2Value = 0;
  currentMessageId = MSG_INITIALIZING;
  invalidateDisplay();
  initGraphStructures();
}


void loop() 
{
  readCo2SensorValueToCo2Response();
  
  if(validateCo2Response()) {
    setCo2ValueAndCurrentMessage((256 * (unsigned int) co2SensorResponse[2]) + (unsigned int) co2SensorResponse[3]);
  } else {
    currentMessageId = MSG_SENSOR_ERROR;
  }
  
  invalidateDisplay();
  
  delay(1000);//timeout between reads - 1 seconds
}


void initGraphStructures() {
  for(byte i = 0; i < GRAPH_WIDTH; i++) {
    graphData[i] = getGraphValue(MIN_VALUE);
  }
  goodValueGraphValue = getGraphValue(GOOD_VALUE);
  acceptableValueGraphValue = getGraphValue(ACCEPTABLE_VALUE);
}

int getGraphValue(unsigned int value) {
    if(value < MIN_VALUE) {
      return 0;
    }
    if(value > MAX_GRAPH_VALUE) {
      value = MAX_GRAPH_VALUE;
    }
    return (value - MIN_VALUE) / GRAPH_STEP;
}

bool validateCo2Response() {
  byte crc = 0;
  for (int i = 1; i < 8; i++) {
    crc += co2SensorResponse[i];
  }
  crc = 256 - crc;
  bool valid = co2SensorResponse[0] == 0xFF && co2SensorResponse[1] == 0x86 && co2SensorResponse[8] == crc;
  if(!valid) {
    Serial.println("CRC error: " + String(crc) + "/"+ String(co2SensorResponse[8]));
  }
  return valid; 
}
  
void readCo2SensorValueToCo2Response() {
  co2SensorSerial.write(cmd, 9);
  memset(co2SensorResponse, 0, 9);
  if(co2SensorSerial.available() > 1) {
    co2SensorSerial.readBytes(co2SensorResponse, 9);
  }
  Serial.print("Respond: ");
  for(byte i = 0; i < 9; i++) {
    Serial.print(co2SensorResponse[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");
}

void addValueToGraph(unsigned int value) {
  for(int i = GRAPH_WIDTH - 1; i > 0; i--) {
    graphData[i] = graphData[i - 1];//shift values
  }
  graphData[0] = getGraphValue(value);
}
  
void setCo2ValueAndCurrentMessage(unsigned int co2Value) {
  currentCo2Value = co2Value;
  Serial.println(co2Value);
  addValueToGraph(co2Value);
  
  if (co2Value <= MIN_VALUE || co2Value > 4900) {
    currentCo2Value = 0;
    currentMessageId = MSG_NO_VALID_CO2_DATA;
    return;
  }

  currentCo2Value = co2Value; 
  
  if (co2Value < VERY_GOOD_VALUE) {   
    currentMessageId = MSG_VERY_GOOD;
    return;
  }
  if (co2Value < GOOD_VALUE) {   
    currentMessageId = MSG_GOOD;
    return;
  }
  if (co2Value < ACCEPTABLE_VALUE) {   
    currentMessageId = MSG_ACCEPTABLE;
    return;
  }
  if (co2Value < MAX_VALUE) {   
    currentMessageId = MSG_BAD;
    return;
  }
  currentMessageId = MSG_HEALTH_RISK;
}

void draw() {
  display.setFont(u8g2_font_6x10_tf);
  display.setFontRefHeightExtendedText();
  display.setDrawColor(PIXELCOLOR);
  display.setFontPosTop();
  display.setFontDirection(0);

  String co2StatusText = "CO2: " + String(currentCo2Value) + " ppm";
  char co2ValueBuff[sizeof(co2StatusText) + 1];
  String(co2StatusText).toCharArray(co2ValueBuff, sizeof(co2ValueBuff));
  display.drawStr(0, 0, co2ValueBuff);
  
  char msgBuff[sizeof(messages[currentMessageId]) + 1];
  messages[currentMessageId].toCharArray(msgBuff, sizeof(msgBuff));
  display.drawStr(0, 10, msgBuff);

  drawGraph();
}

void drawGraph() {
  bool dashState = false;
  for(int i = 0; i < GRAPH_WIDTH; i++) {
    byte x = GRAPH_WIDTH - 1 - i;
    display.setDrawColor(PIXELCOLOR);
    display.drawLine(x, displayHeight - 1, x, displayHeight - 1 - graphData[i]);
    display.setDrawColor(dashState ? PIXELCOLOR : BACKCOLOR);
    display.drawPixel(x, displayHeight - 1 - goodValueGraphValue);
    display.drawPixel(x, displayHeight - 1 - acceptableValueGraphValue);
    dashState = !dashState;
  }
}

void invalidateDisplay() {
  display.clearBuffer();
  draw();
  display.sendBuffer();
}

void initDisplay() {
  display.begin();
  display.setPowerSave(0);  // init done
  displayHeight = 48;// display.height();
}
