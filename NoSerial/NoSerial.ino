// Copied definition from TRSim_Aerostar.h

#include <Arduino.h>

namespace TRSim_Aerostar {

// Internal constants
#define _BUFFER_SIZE 1024
#define _BUFFER_SIZE_MASK 0x3ff
#define _RECEIVER_BUFFER_SIZE 512
#define _PACKET_SIZE 79
#define _DEFAULT_BAUD_RATE 115200
#define _DEFAULT_TIMEOUT 0.1

#define _SYNCBYTE1 0x55
#define _SYNCBYTE2 0xAA

#define _MODE_FINDING_SYNCBYTE1 1
#define _MODE_FINDING_SYNCBYTE2 2
#define _MODE_FINDING_PAYLOAD 3

// pin mappings
//   Note: this pin already defined in header files
// #define PIN_NEOPIXEL     40
#define PIN_PBF 2
#define PIN_GO 3

// Set up events
const unsigned int STATUS_NUM_EVENTS = 5;
const unsigned int STATUS_UNKNOWN = 0;
const unsigned int STATUS_INITIALIZING = 1;
const unsigned int STATUS_LAUNCHING = 2;
const unsigned int STATUS_FLOATING = 3;
const unsigned int STATUS_DESCENDING = 4;

// Set up threshold ascent rates and altitude levels to determine flight status
#define _THRESHOLD_LAUNCH_VUP 1
#define _THRESHOLD_FLOAT_VUP 1
#define _THRESHOLD_DESCEND_VUP -4
#define _THRESHOLD_GROUND_ALT 1000
#define _THRESHOLD_FLOAT_ALT 18500

// Baud rate for USB serial connection
//#define SERIAL_BAUD 115200


// Simulator class
//
class Simulator {
  bool _new_data;
  unsigned int _stream_timeout;

  unsigned char _buffer[_BUFFER_SIZE];
  unsigned char _currPacketBuffer[_BUFFER_SIZE];
  unsigned char _returnBuffer[_PACKET_SIZE];

  unsigned int _flightStatus;

  unsigned int _mode;
  unsigned int _saveStart;
  unsigned int _saveEnd;
  unsigned int _packetStart;
  unsigned int _packetEnd;
  unsigned int _packetBytesFound;

  int _pbfPin;
  int _goPin;
  int _pbfState;

public:
  Simulator();
  ~Simulator();
  void init(int pbfPin = PIN_PBF, int goPin = PIN_GO);
  void update();
  bool isNewData();
  bool isStreaming();
  int getPBF();
  int getGo();
  void setGo(int val);

  unsigned char* getData();
  unsigned int getStatus();
  unsigned int getTimeSecs();
  unsigned int getTimeUsecs();
  float getLatitude();
  float getLongitude();
  float getAltitude();
  float getRoll();
  float getPitch();
  float getYaw();
  float getAccelerationX();
  float getAccelerationY();
  float getAccelerationZ();
  float getPressure();
  float getCourse();
  float getSpeed();
  float getVelocityNorth();
  float getVelocityEast();
  float getVelocityDown();
  float getDeclination();
  int getNumSats();
  int getGPSFix();
  //void serialPrintCurrentPacket();
  //void serialPrintHexString(unsigned char* buff);
};

}  // namespace

#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "Wire.h"
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <SD.h>
#include "Adafruit_BME680.h"
#include "Adafruit_MCP9601.h"

File myFile;

#define PCAADDR 0x70
#define SEALEVELPRESSURE_HPA (1013.25)
#define MCP_Address (0x67)

Adafruit_LIS3DH lis1;
Adafruit_LIS3DH lis2;
Adafruit_LIS3DH lis3;
Adafruit_LIS3DH lis4;
Adafruit_LIS3DH lis5;
Adafruit_LIS3DH lis6;
Adafruit_LIS3DH lis7;
Adafruit_LIS3DH lis8;

Adafruit_BME680 bme;

Adafruit_MCP9601 mcp;

// Set up Neopixel hardware constants and object for the M4's on-board Neopixel
const unsigned int NEOPIXEL_COUNT = 1;
const unsigned int NEOPIXEL_BRIGHTNESS = 0.2;
Adafruit_NeoPixel pixels(NEOPIXEL_COUNT, PIN_NEOPIXEL);

// Set up some basic color constants for use later
const unsigned int COLOR_RED = 0xff0000;
const unsigned int COLOR_GREEN = 0x00ff00;
const unsigned int COLOR_BLUE = 0x0000ff;
const unsigned int COLOR_YELLOW = 0xffff00;
const unsigned int COLOR_MAGENTA = 0xff00ff;
const unsigned int COLOR_CYAN = 0x00ffff;
const unsigned int COLOR_BLACK = 0x000000;
const unsigned int COLOR_GRAY = 0x7f7f7f;
const unsigned int COLOR_WHITE = 0xffffff;

// Set up flight status event Neopixel colors index
unsigned int statusColors[TRSim_Aerostar::STATUS_NUM_EVENTS];

// Set up simulator data library
TRSim_Aerostar::Simulator TRsim;

// Initialize flight status
int currStatus = TRSim_Aerostar::STATUS_UNKNOWN;
int prevStatus = currStatus;

// Variable for tracking number of full telemetry packets received
int numPackets = 0;

void refresh() {
  pcaselect(0);
  if (!lis1.haveNewData()) {
    log("LIS3DH sensor was dropped on PCA multiplexer channel 0; attempting to restart");
    lis1.begin();
    if (!lis1.begin()) {
      log("Failed to restart LIS3DH #1 - Data should not be used");
    }
  }
  pcaselect(1);
  if (!lis2.haveNewData()) {
    log("LIS3DH sensor was dropped on PCA multiplexer channel 1; attempting to restart");
    lis2.begin();
    if (!lis2.begin()) {
      log("Failed to restart LIS3DH #2 - Data should not be used");
    }
  }
  pcaselect(2);
  if (!lis3.haveNewData()) {
    log("LIS3DH sensor was dropped on PCA multiplexer channel 2; attempting to restart");
    lis3.begin();
    if (!lis3.begin()) {
      log("Failed to restart LIS3DH #3 - Data should not be used");
    }
  }
  pcaselect(3);
  if (!lis4.haveNewData()) {
    log("LIS3DH sensor was dropped on PCA multiplexer channel 3; attempting to restart");
    lis4.begin();
    if (!lis4.begin()) {
      log("Failed to restart LIS3DH #4 - Data should not be used");
    }
  }
  pcaselect(4);
  if (!lis5.haveNewData()) {
    log("LIS3DH sensor was dropped on PCA multiplexer channel 4; attempting to restart");
    lis5.begin();
    if (!lis5.begin()) {
      log("Failed to restart LIS3DH #5 - Data should not be used");
    }
  }
  pcaselect(5);
  if (!lis6.haveNewData()) {
    log("LIS3DH sensor was dropped on PCA multiplexer channel 5; attempting to restart");
    lis6.begin();
    if (!lis6.begin()) {
      log("Failed to restart LIS3DH #6 - Data should not be used");
    }
  }
  pcaselect(6);
  if (!lis7.haveNewData()) {
    log("LIS3DH sensor was dropped on PCA multiplexer channel 6; attempting to restart");
    lis7.begin();
    if (!lis7.begin()) {
      log("Failed to restart LIS3DH #7 - Data should not be used");
    }
  }
  pcaselect(7);
  if (!lis8.haveNewData()) {
    log("LIS3DH sensor was dropped on PCA multiplexer channel 7; attempting to restart");
    lis8.begin();
    if (!lis8.begin()) {
      log("Failed to restart LIS3DH #8 - Data should not be used");
    }
  }
  if (!bme.performReading()) {
    log("BME688 sensor was dropped; attempting to restart");
    bme.begin();
    if (!bme.begin()) {
      log("Failed to restart BME688 - Data should not be used");
    }
  }
  if (!mcp.readThermocouple()) {
    log("MCP9601 sensor was dropped; attempting to restart");
    mcp.begin(MCP_Address);
    if (!mcp.begin()) {
      log("Failed to restart MCP9601 - Data should not be used");
    }
  }
  if (!mcp.readAmbient()) {
    log("MCP9601 sensor was dropped; attempting to restart");
    mcp.begin(MCP_Address);
    if (!mcp.begin()) {
      log("Failed to restart MCP9601 - Data should not be used");
    }
  }
  if (!mcp.readADC()) {
    log("MCP9601 sensor was dropped; attempting to restart");
    mcp.begin(MCP_Address);
    if (!mcp.begin()) {
      log("Failed to restart MCP9601 - Data should not be used");
    }
  }
}

void pcaselect(uint8_t i) {
  if (i > 8) return;

  Wire.beginTransmission(PCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

//#define USE_SERIAL

void log(const char c[]) {
/*
#ifdef USE_SERIAL
  Serial.print("log, ");
  Serial.print(c);
  Serial.println(" ,");
  */
//#endif
  myFile.print("log, ");
  myFile.print(c);
  myFile.println(" ,");
}

// setup()
//   Initialization functions
//
void setup() {
  // Set up Serial
  //   Serial = USB
  //   TRSim = Serial1 = UART
  //Serial.print("Initializing SD card...");
  SD.begin(10);

  myFile = SD.open("test.txt", FILE_WRITE);
  myFile.println(":)");
  myFile.print("Time,	TimeUsecs,	TimeSecs,	Latitude,	Longitude,	Altitude,	Roll,	Pitch,	Yaw,	AccelX,	AccelY,	AccelZ,	Pressure,	Course,	Speed,	VelocityN,	VelocityE,	VelocityD,	Declination,	numSats,	gpsFix,	Accel1,	Accel2,	Accel3,	Accel4,	Accel5,	Accel6,	Accel7,	Accel8,	*C,	hPa,	%,	KOhms,	m, *C, *C, uV");
  myFile.println();
  myFile.flush();
  Wire.setClock(20000);
  Wire.begin();
  //Serial.begin(SERIAL_BAUD);
  delay(2000);  // Improves chances of seeing first printed messages over USB serial
  //while (!Serial);
  //Serial.println("ACCELEROMETER DATA :)");
  if (!mcp.begin(MCP_Address)) {
    log("Could not find a valid MCP9601 sensor");
  } else {
    //Serial.println("Found MCP9601!");
    mcp.setADCresolution(MCP9600_ADCRESOLUTION_18);
    //Serial.print("ADC resolution set to ");
    /*switch (mcp.getADCresolution()) {
      case MCP9600_ADCRESOLUTION_18; //Serial.print("18"); break;
      case MCP9600_ADCRESOLUTION_16; //Serial.print("16"); break;
      case MCP9600_ADCRESOLUTION_14; //Serial.print("14"); break;
      case MCP9600_ADCRESOLUTION_12; //Serial.print("12"); break;
    }*/
    //Serial.println(" bits");

    mcp.setThermocoupleType(MCP9600_TYPE_K);
    //Serial.print("Thermocouple type set to ");
    /*switch (mcp.getThermocoupleType()) {
      case MCP9600_TYPE_K; //Serial.print("K"); break;
      case MCP9600_TYPE_J; //Serial.print("J"); break;
      case MCP9600_TYPE_T; //Serial.print("T"); break;
      case MCP9600_TYPE_N; //Serial.print("N"); break;
      case MCP9600_TYPE_S; //Serial.print("S"); break;
      case MCP9600_TYPE_E; //Serial.print("E"); break;
      case MCP9600_TYPE_B; //Serial.print("B"); break;
      case MCP9600_TYPE_R; //Serial.print("R"); break;
    }*/
    //Serial.println(" type");

    mcp.setFilterCoefficient(3);
    //Serial.print("Filter coefficient value set to: ");
    //Serial.println(mcp.getFilterCoefficient());

    mcp.setAlertTemperature(1, 30);
    //Serial.print("Alert #1 temperature set to ");
    //Serial.println(mcp.getAlertTemperature(1));
    mcp.configureAlert(1, true, true);  // alert 1 enabled, rising temp

    mcp.enable(true);
  }

  Wire.begin();
  if (!bme.begin()) {
    log("Could not find a valid BME680 sensor");
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150);  // 320*C for 150 ms

  // pcaselect to set the channel.
  pcaselect(0);
  lis1.begin();
  if (!lis1.begin()) {
    log("Could not find a valid LIS3DH sensor on PCA Multiplexer channel 0");
  }

  pcaselect(1);
  lis2.begin();
  if (!lis2.begin()) {
    log("Could not find a valid LIS3DH sensor on PCA Multiplexer channel 1");
  }

  pcaselect(2);
  lis3.begin();
  if (!lis3.begin()) {
    log("Could not find a valid LIS3DH sensor on PCA Multiplexer channel 2");
  }

  pcaselect(3);
  lis4.begin();
  if (!lis4.begin()) {
    log("Could not find a valid LIS3DH sensor on PCA Multiplexer channel 3");
  }

  pcaselect(4);
  lis5.begin();
  if (!lis5.begin()) {
    log("Could not find a valid LIS3DH sensor on PCA Multiplexer channel 4");
  }

  pcaselect(5);
  lis6.begin();
  if (!lis6.begin()) {
    log("Could not find a valid LIS3DH sensor on PCA Multiplexer channel 5");
  }

  pcaselect(6);
  lis7.begin();
  if (!lis7.begin()) {
    log("Could not find a valid LIS3DH sensor on PCA Multiplexer channel 6");
  }

  pcaselect(7);
  lis8.begin();
  if (!lis8.begin()) {
    log("Could not find a valid LIS3DH sensor on PCA Multiplexer channel 7");
  }
  pinMode(13, OUTPUT);
  digitalWrite(13, 0);

  //Serial.println("Aerostar Run");

  // Set up simulator data library
  //   By default, PBF pin = 2, GO pin = 3. If your wiring is different, see
  //   documentation for how to change pins using this function call.
  TRsim.init();

  // Set up status colors
  statusColors[TRSim_Aerostar::STATUS_UNKNOWN] = COLOR_GRAY;
  statusColors[TRSim_Aerostar::STATUS_INITIALIZING] = COLOR_YELLOW;
  statusColors[TRSim_Aerostar::STATUS_LAUNCHING] = COLOR_GREEN;
  statusColors[TRSim_Aerostar::STATUS_FLOATING] = COLOR_CYAN;
  statusColors[TRSim_Aerostar::STATUS_DESCENDING] = COLOR_BLUE;

  // Display flight status (unknown) on Neopixel
  pixels.begin();
  pixels.setBrightness(0 * NEOPIXEL_BRIGHTNESS);
  pixels.fill(statusColors[currStatus], 0, NEOPIXEL_COUNT);
  pixels.show();
}

// loop()
//   Do forever, start the main loop
//
void loop() {
  // Pointer for keeping track of incoming data
  unsigned char* data;

  // Call Simulator update() function to catch serial input and check PBF.
  //   This must be called at the top of the loop.
  TRsim.update();

  // Check if the PBF header is closed (False). If it is, light Neopixel red.
  //   If it is open (True), we are free to do in-flight work!
  if (TRsim.getPBF() == LOW) {
    // Light the neopixel red to highlight that the PBF is inserted
    pixels.fill(COLOR_RED, 0, NEOPIXEL_COUNT);
    pixels.show();
  } else {
    // PBF header is open, we are flying and can do some work!

    // TRsim.isStreaming() will be True while valid data is incoming
    // If data is paused for more than 1.5 seconds it will become False
    if (TRsim.isStreaming() == true) {
      // TRsim.isNewData() will be True after a new packet arrives
      // When TRsim.getData() is called TRsim.isNewData() will become False.
      if (TRsim.isNewData() == true) {
        // Got a new telemetry packet, let's count it!
        numPackets += 1;

        // Grab new data - NOTE this sets isNewData() to False
        data = TRsim.getData();

        // You can add code here that needs to execute each time new
        //   telemetry data is received.

        // Get current flight status and check to see if it has changed
        currStatus = TRsim.getStatus();
        // See if the status has changed by comparing with previous value
        if (currStatus != prevStatus) {
          // If it has changed, save the current status to prevStatus
          prevStatus = currStatus;
          // Since the event changed, print something to indicate change.
          // You can initiate activity for your payload in this
          //   section, since it will only execute on status change.
          //   However, when running the simulator, please note that
          //   this section may execute again if you pause and unpause
          //   the simulator.
          /*
          if (currStatus == TRSim_Aerostar::STATUS_UNKNOWN) {
            Serial.println("We are unknown");
          } else if (currStatus == TRSim_Aerostar::STATUS_INITIALIZING) {
            Serial.println("We are initializing");
          } else if (currStatus == TRSim_Aerostar::STATUS_LAUNCHING) {
            Serial.println("We are launching");
          } else if (currStatus == TRSim_Aerostar::STATUS_FLOATING) {
            Serial.println("We are floating");
          } else if (currStatus == TRSim_Aerostar::STATUS_DESCENDING) {
            Serial.println("We are descending");
          }
          */
          // Indicate the new event with a color from the status list
          pixels.fill(statusColors[currStatus], 0, NEOPIXEL_COUNT);
          pixels.show();
        }
        //--------------------------------------------------------
        // Print every 10th packet
        //Column 1-21 (A-U) CSV
        if ((numPackets % 10) == 1) {
          refresh();
          //TRsim.serialPrintCurrentPacket();
          SD.open("test.txt", FILE_WRITE);

          //Serial.println(numPackets);

          //Serial.println("Writing to test.txt...");
          myFile.print(TRsim.getTimeSecs());
          myFile.print(",");
          myFile.print(TRsim.getTimeUsecs());
          myFile.print(",");
          myFile.print(TRsim.getTimeSecs());
          myFile.print(",");
          myFile.print(TRsim.getLatitude());
          myFile.print(",");
          myFile.print(TRsim.getLongitude());
          myFile.print(",");
          myFile.print(TRsim.getAltitude());
          myFile.print(",");
          myFile.print(TRsim.getRoll());
          myFile.print(",");
          myFile.print(TRsim.getPitch());
          myFile.print(",");
          myFile.print(TRsim.getYaw());
          myFile.print(",");
          myFile.print(TRsim.getAccelerationX());
          myFile.print(",");
          myFile.print(TRsim.getAccelerationY());
          myFile.print(",");
          myFile.print(TRsim.getAccelerationZ());
          myFile.print(",");
          myFile.print(TRsim.getPressure());
          myFile.print(",");
          myFile.print(TRsim.getCourse());
          myFile.print(",");
          myFile.print(TRsim.getSpeed());
          myFile.print(",");
          myFile.print(TRsim.getVelocityNorth());
          myFile.print(",");
          myFile.print(TRsim.getVelocityEast());
          myFile.print(",");
          myFile.print(TRsim.getVelocityDown());
          myFile.print(",");
          myFile.print(TRsim.getDeclination());
          myFile.print(",");
          myFile.print(TRsim.getNumSats());
          myFile.print(",");
          myFile.print(TRsim.getGPSFix());
          myFile.print(",");
          sensors_event_t event;

          //--------------------------------------------------------
          // Accelerometer 1: Column 22 (V) CSV

          pcaselect(0);

          lis1.read();
          lis2.getEvent(&event);

          float xVal = pow(event.acceleration.x, 2);
          float yVal = pow(event.acceleration.y, 2);
          float zVal = pow(event.acceleration.z, 2);
          float g_Value = sqrt(xVal + yVal + zVal);
          //Serial.print("sensor 1: \t");
          //Serial.print(g_Value, 7);

          lis1.readAndClearInterrupt();

          //Serial.println();
          //----------------------------------------------------------
          // Accelerometer 2: Column 23 (W) CSV

          pcaselect(1);

          lis2.read();
          lis2.getEvent(&event);

          float xVal2 = pow(event.acceleration.x, 2);
          float yVal2 = pow(event.acceleration.y, 2);
          float zVal2 = pow(event.acceleration.z, 2);
          float g_Value2 = sqrt(xVal2 + yVal2 + zVal2);
          //Serial.print("sensor 2: \t");
          //Serial.print(g_Value2, 7);

          lis2.readAndClearInterrupt();

          //Serial.println();
          //------------------------------------------------------------
          // Accelerometer 3: Column 24 (X) CSV

          pcaselect(2);

          lis3.read();
          lis3.getEvent(&event);

          float xVal3 = pow(event.acceleration.x, 2);
          float yVal3 = pow(event.acceleration.y, 2);
          float zVal3 = pow(event.acceleration.z, 2);
          float g_Value3 = sqrt(xVal3 + yVal3 + zVal3);
          //Serial.print("sensor 3: \t");
          //Serial.print(g_Value3, 7);

          lis3.readAndClearInterrupt();

          //Serial.println();
          //------------------------------------------------------------
          // Accelerometer 4: Column 25 (Y) CSV

          pcaselect(3);

          lis4.read();
          lis4.getEvent(&event);

          float xVal4 = pow(event.acceleration.x, 2);
          float yVal4 = pow(event.acceleration.y, 2);
          float zVal4 = pow(event.acceleration.z, 2);
          float g_Value4 = sqrt(xVal4 + yVal4 + zVal4);
          //Serial.print("sensor 4: \t");
          //Serial.print(g_Value4, 7);

          lis4.readAndClearInterrupt();

          //Serial.println();
          //------------------------------------------------------------
          // Accelerometer 5: Column 26 (Z) CSV

          pcaselect(4);

          lis5.read();
          lis5.getEvent(&event);

          float xVal5 = pow(event.acceleration.x, 2);
          float yVal5 = pow(event.acceleration.y, 2);
          float zVal5 = pow(event.acceleration.z, 2);
          float g_Value5 = sqrt(xVal5 + yVal5 + zVal5);
          //Serial.print("sensor 5: \t");
          //Serial.print(g_Value5, 7);

          lis5.readAndClearInterrupt();

          //Serial.println();
          //------------------------------------------------------------
          // Accelerometer 6: Column 27 (AA) CSV

          pcaselect(5);

          lis6.read();
          lis6.getEvent(&event);

          float xVal6 = pow(event.acceleration.x, 2);
          float yVal6 = pow(event.acceleration.y, 2);
          float zVal6 = pow(event.acceleration.z, 2);
          float g_Value6 = sqrt(xVal6 + yVal6 + zVal6);
          //Serial.print("sensor 6: \t");
          //Serial.print(g_Value6, 7);

          lis6.readAndClearInterrupt();

          //Serial.println();
          //------------------------------------------------------------
          // Accelerometer 7: Column 28 (AB) CSV

          pcaselect(6);

          lis7.read();
          lis7.getEvent(&event);

          float xVal7 = pow(event.acceleration.x, 2);
          float yVal7 = pow(event.acceleration.y, 2);
          float zVal7 = pow(event.acceleration.z, 2);
          float g_Value7 = sqrt(xVal7 + yVal7 + zVal7);
          //Serial.print("sensor 7: \t");
          //Serial.print(g_Value7, 7);

          lis7.readAndClearInterrupt();

          //Serial.println();
          //------------------------------------------------------------
          // Accelerometer 8: Column 29 (AC) CSV

          pcaselect(7);

          lis8.read();
          lis8.getEvent(&event);

          float xVal8 = pow(event.acceleration.x, 2);
          float yVal8 = pow(event.acceleration.y, 2);
          float zVal8 = pow(event.acceleration.z, 2);
          float g_Value8 = sqrt(xVal8 + yVal8 + zVal8);
          //Serial.print("sensor 8: \t");
          //Serial.print(g_Value8, 7);

          lis8.readAndClearInterrupt();

          //--------------------------------------------------------
          //BME Temp/Pressure Sensor: Columns 30-34 (AD-AH) CSV
          //Serial.println();
          //if (!bme.performReading()) {
            //Serial.println("Failed to perform reading :(");
          //}
          //Serial.print("Temperature = ");
          //Serial.print(bme.temperature);
          //Serial.println(" *C");

          //Serial.print("Pressure = ");
          //Serial.print(bme.pressure / 100.0);
          //Serial.println(" hPa");

          //Serial.print("Humidity = ");
          //Serial.print(bme.humidity);
          //Serial.println(" %");

          //Serial.print("Gas = ");
          //Serial.print(bme.gas_resistance / 1000.0);
          //Serial.println(" KOhms");

          //Serial.print("Approx. Altitude = ");
          //Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
          //Serial.println(" m");

          //--------------------------------------------------------
          //MCP9601 Thermocouple Amplifier: Columns 35-37 (AI-AK)

          //Serial.println("MCP data:");
          //Serial.print(mcp.readThermocouple());
          //Serial.print(", ");
          //Serial.print(mcp.readAmbient());
          //Serial.print(", ");
          //Serial.print(mcp.readADC() * 2);
          //Serial.print(", ");

          //------------------------------------------------------------
          SD.open("test.txt", FILE_WRITE);

          //Serial.print("Writing to test.txt...");

          // Utilizing 7 decimal places for more precision

          myFile.print(g_Value, 7);
          myFile.print(",");
          myFile.print(g_Value2, 7);
          myFile.print(",");
          myFile.print(g_Value3, 7);
          myFile.print(",");
          myFile.print(g_Value4, 7);
          myFile.print(",");
          myFile.print(g_Value5, 7);
          myFile.print(",");
          myFile.print(g_Value6, 7);
          myFile.print(",");
          myFile.print(g_Value7, 7);
          myFile.print(", ");
          myFile.print(g_Value8, 7);
          myFile.print(", \t");
          myFile.print(bme.temperature);
          myFile.print(", ");
          myFile.print(bme.pressure);
          myFile.print(", ");
          myFile.print(bme.humidity);
          myFile.print(", ");
          myFile.print(bme.gas_resistance);
          myFile.print(", ");
          myFile.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
          myFile.print(", ");
          myFile.print(mcp.readThermocouple());
          myFile.print(", ");
          myFile.print(mcp.readAmbient());
          myFile.print(", ");
          myFile.print(mcp.readADC() * 2);
          myFile.print(", ");

          myFile.println();
          myFile.flush();
          //Serial.println("done.");
        }
      }
    } else {  // not streaming
      // Data stream has stopped for 1.5s, fill pixels with unknown (idle) color
      pixels.fill(statusColors[TRSim_Aerostar::STATUS_UNKNOWN], 0, NEOPIXEL_COUNT);
      pixels.show();

      // Reset the previous status to unknown (idle)
      prevStatus = TRSim_Aerostar::STATUS_UNKNOWN;
    }
  }

  delay(10);
}
