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

  ESP32  | Temperature and humidity sensor DHT-11
  +3.3V  - 1 (left, down - from the side of the grid)
  pin 21 - 2
  - (NC) - 3
  GND    - 4

  ESP32  | Display Nokia 5110
  pin 14 - Serial clock out: CLK (SCLK)
  pin 13 - Serial data out: DIN
  pin 27 - Data/Command select: DC (D/C)
  pin 15 - LCD chip select: CE (CS)
  pin 26 - LCD reset: RST
  5V    - VCC
  GND   - GND

  Notes for MH-Z19B:
  - When placed in small space, the space should be well ventilated, especially for diffusion window.
  - To ensure the normal work, power supply must be among 4.5V~5.5V DC range by not less than 150mA. 
  Out of this range, it will result in the failure sensor. 
  (The concentration output is low, or the sensor can not work normally)
*/

#include <HardwareSerial.h>
#include <dht.h> //Used this library (download, zip, install to Arduino): https://github.com/RobTillaart/Arduino/tree/master/libraries/DHTstable

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
//put another display from this file: https://github.com/olikraus/u8g2/blob/master/tools/inoupdate/frame_buffer.ino
const byte DISPLAY_WIDTH = 84;
const byte DISPLAY_HEIGHT = 48;

HardwareSerial co2SensorSerial(1); // Serial #1

// CO2 sensor data structures:
byte cmd[9] = {0xFF,0x01,0x86,0x00,0x00,0x00,0x00,0x00,0x79}; 
/* --- Request (9 bytes) --- 
 *  0xFF - Start byte
 *  0x01 - Sensor #1
 *  0x86 - Commands: Read CO2 concentration
 *  ...
 *  0x79 - "Checksum is expected in the response"
 *  
 * --- Response (9 bytes) for the command 0x86 "Read CO2 concentration" ---
 *  0xFF - Start byte
 *  0x86 - Command
 *  0x** - CO2 concentration value (high byte)
 *  0x** - CO2 concentration value (low byte)
 *  ...
 *  0x** - Checksum value 
 *  
 * --- Other commands ---
 *  0x87 - Calibrate Zero Point (ZERO)
 *         - During the zero point calibration procedure by manual, 
 *         the sensor must work in stable gas environment (400ppm) for over 20 minutes.
 *         Connect the HD pin to low level (0V) for over 7 seconds.
 *  0x88 - Calibrate Span Point (SPAN)
 *  0x79 - ON/OFF Auto CalibrationON/OFF
 *  0x99 - Detection range setting
 *  
 */

//Response buffer
unsigned char co2SensorResponse[9]; 
const byte MH_Z19_RESPOND_START_BYTE = 0xFF;
const byte MH_Z19_RESPOND_COMMAND_READ_CO2 = 0x86;

//CO2 concentration levels
const unsigned int MIN_VALUE = 300;
const unsigned int VERY_GOOD_VALUE = 450;
const unsigned int GOOD_VALUE = 600;
const unsigned int ACCEPTABLE_VALUE = 1000;
const unsigned int MAX_VALUE = 2500;

//Display settings
unsigned int goodValueGraphValue = 0;
unsigned int acceptableValueGraphValue = 0;
const byte GRAPH_WIDTH = DISPLAY_WIDTH;
const byte GRAPH_HEIGHT = DISPLAY_HEIGHT - 20;
const unsigned int MAX_GRAPH_VALUE = 1500;
const unsigned int GRAPH_STEP = (MAX_GRAPH_VALUE - MIN_VALUE) / GRAPH_HEIGHT;
unsigned int graphData[GRAPH_WIDTH];
#define BACKCOLOR 0 // White
#define PIXELCOLOR 1  // Black
int displayHeight = DISPLAY_HEIGHT;//default - should be set in the method "initDisplay()"
byte graphSpeed = 1;
int graphDrawingCounter = 0;

//Messages
String messages[] = {"", "Very good","Good", "Acceptable","Bad","Health risk", 
                     "CO2 sensor error", "No valid CO2 data", 
                     "Initializing"};
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
int currentTempValue = 0;
int currHumidityValue = 0;
    
//timer
volatile int timerInterruptCount = 0;
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
#define CPU_CLOCK_MHz 80 //Specify the CPU clock (in MHz)
#define TIMER_DURATION_MICROSEC 2000000 //timeout between reads (in microseconds)

//Temperature and humidity sensor DH-11 (or DH-22)
dht DHT;

#define DHT11_PIN 21


void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  timerInterruptCount++;
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  currentCo2Value = 0;
  currentMessageId = MSG_INITIALIZING;

  Serial.begin(115200); //Communication between ESP32 and PC
  while(!Serial){}
  Serial.println("Serial interface initialized.");

  Serial.print("Init display...");
  initDisplay();
  Serial.println(" complete.");
  invalidateDisplay();
  initGraphStructures();
  
  Serial.print("Init serial interface to MH-Z19...");
  delay(5000);//Give some time to the sensor for starting-up
  co2SensorSerial.begin(9600, SERIAL_8N1, 16, 17); //RX, TX - communication between ESP32 and MH-Z19
  Serial.println(" complete");

  Serial.println("INitialize timers.");
  timer = timerBegin(0, CPU_CLOCK_MHz, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, TIMER_DURATION_MICROSEC, true);
  timerAlarmEnable(timer);
  Serial.println("Initialization complete.");
}


void loop() 
{
  processDataReadAndDisplayIfRequested();
}


void processDataReadAndDisplayIfRequested() {
  if(timerInterruptCount <= 0) {
    return;
  }

  portENTER_CRITICAL(&timerMux);
  timerInterruptCount = 0; //or decrease on 1 - to process readings as a queued requests
  portEXIT_CRITICAL(&timerMux);

  readCo2SensorValueToCo2Response();
  
  if(validateCo2Response()) {
    setCo2ValueAndCurrentMessage((256 * (unsigned int) co2SensorResponse[2]) + (unsigned int) co2SensorResponse[3]);
  } else {
    currentMessageId = MSG_SENSOR_ERROR;
  }

  readTemperatureAndHumidity();
  
  invalidateDisplay();
  
  printRespond();
}

void readTemperatureAndHumidity() {
  Serial.print("Read DHT11..");
  int result = DHT.read11(DHT11_PIN);
  switch (result)
  {
    case DHTLIB_OK:  
    Serial.print("OK."); 
    currentTempValue = DHT.temperature;
    currHumidityValue = DHT.humidity;
    break;
    case DHTLIB_ERROR_CHECKSUM: 
    Serial.print("DHT checksum error"); 
    break;
    case DHTLIB_ERROR_TIMEOUT: 
    Serial.print("DHT timeout errort"); 
    break;
    default: 
    Serial.print("DHT unknown error"); 
    break;
  }
  Serial.println("H:" + String(currHumidityValue) + "; T:" + String(currentTempValue) + "°C");
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
  byte checksum = 0;
  for (byte i = 1; i < 8; i++) {
    checksum += co2SensorResponse[i];
  }
  checksum = 0xFF - checksum;
  checksum++;
  bool valid = co2SensorResponse[0] == MH_Z19_RESPOND_START_BYTE 
               && co2SensorResponse[1] == MH_Z19_RESPOND_COMMAND_READ_CO2
               && co2SensorResponse[8] == checksum;
  if(!valid) {
    Serial.println("CRC error: " + String(checksum) + "/"+ String(co2SensorResponse[8]));
  }
  return valid; 
}
  
void readCo2SensorValueToCo2Response() {
  co2SensorSerial.write(cmd, 9);
  memset(co2SensorResponse, 0, 9);
  if(co2SensorSerial.available()) {
    co2SensorSerial.readBytes(co2SensorResponse, 9);
  }
}

void printRespond() {
  Serial.print("Respond: ");
  for(byte i = 0; i < 9; i++) {
    Serial.print(co2SensorResponse[i], HEX);
    Serial.print(" ");
  }
  Serial.println("");
}

void addValueToGraph(unsigned int value) {
  for(int i = GRAPH_WIDTH - 1; i > 0; i--) {
    graphData[i] = graphData[i - 1];//shift graph values
  }
  graphData[0] = getGraphValue(value);
}
  
void setCo2ValueAndCurrentMessage(unsigned int co2Value) {
  currentCo2Value = co2Value;
  addValueToGraph(co2Value);
  
  if (co2Value <= MIN_VALUE || co2Value > 4900) {
    currentCo2Value = 0;
    currentMessageId = MSG_NO_VALID_CO2_DATA;
  } else {
    currentCo2Value = co2Value; 
    currentMessageId = getMessageByCo2Value(co2Value);
  }

  Serial.println("CO2:" + String(co2Value) + "; \"" + messages[currentMessageId] + "\"");
}

byte getMessageByCo2Value(unsigned int co2Value){
  if (co2Value < VERY_GOOD_VALUE) {   
    return MSG_VERY_GOOD;
  }
  if (co2Value < GOOD_VALUE) {   
    return MSG_GOOD;
  }
  if (co2Value < ACCEPTABLE_VALUE) {   
    return MSG_ACCEPTABLE;
  }
  if (co2Value < MAX_VALUE) {   
    return MSG_BAD;
  }
  return MSG_HEALTH_RISK;
}

void setFont() {  
  display.setFont(u8g2_font_6x10_tf);
  display.setFontRefHeightExtendedText();
  display.setDrawColor(PIXELCOLOR);
  display.setFontPosTop();
  display.setFontDirection(0);
}

void drawCo2Value() {
  String co2StatusText = " CO2 " + String(currentCo2Value) + " ppm";
  char co2ValueBuff[sizeof(co2StatusText) + 1];
  String(co2StatusText).toCharArray(co2ValueBuff, sizeof(co2ValueBuff));
  display.drawStr(0, 0, co2ValueBuff);
}

void drawTempValue() {
  String text = "Temp " + String(currentTempValue) + " C";
  char textBuff[sizeof(text) + 1];
  String(text).toCharArray(textBuff, sizeof(textBuff));
  display.drawStr(0, 10, textBuff);
}

void drawHumidValue() {
  //TODO
  }

void drawMessage() {
  char msgBuff[sizeof(messages[currentMessageId]) + 1];
  messages[currentMessageId].toCharArray(msgBuff, sizeof(msgBuff));
  display.drawStr(0, 10, msgBuff);
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

void draw() {
  setFont();
  drawCo2Value();  
  drawTempValue();  
  drawHumidValue();  
 // drawMessage();
  drawGraph();
}

void invalidateDisplay() {
  display.clearBuffer();
  draw();
  display.sendBuffer();
}

void initDisplay() {
  display.begin();
  display.setPowerSave(0); ///Not sure yet
  displayHeight = DISPLAY_HEIGHT;// quick-fix - it shoul be set from display.height();
}


