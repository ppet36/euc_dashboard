/**
 * Dashboard pro my DIY EUC. For ESP32 a Nextion NX4024T032 display.
 * My EUC is based on Begode/Gotway Nikola board. Uses 24s/10p battery
 * managed by JK Smart BMS BD6A24S10P. Total capacity of battery is 5kWh.
 *
 * Programmed from many sources I found on the internet.
 *
 * @author ppet36
*/

#include "BLEDevice.h"
#include <EEPROM.h>
#include <Wire.h>
#include <DS3231.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <ArduinoOTA.h>
#include "RunningMedian.h"
#include "css.h"
#include "esp32_up_down_sd.h"

// Frequency of display update.
#define DISPLAY_UPDATE 500

// SD update
#define SD_UPDATE_TIME 1000

// Light update time
#define LIGHT_UPDATE_TIME 1000
#define LIGHT_LEVEL_INCREMENT 50

// Time in milliseconds when a packet must be sent from EUC/BMS otherwise it is taken as
// lose connection and reboot.
#define MAX_PACKET_TIME 10000

// Battery percent grow to assume charged
#define ASSUME_CHARGED_PERCENT 4.0

// Display is connected to UART 2 on ESP32
#define display Serial2

// Search time for BLE devices in seconds
#define SCAN_TIMEOUT 15

// Number of serial cells.
#define NUM_CELLS 24

// Some colors on Nextion display
#define NEXTION_SILVER_COLOR 50712
#define NEXTION_GREEN_COLOR  34784
#define NEXTION_RED_COLOR    64171
#define NEXTION_YELLOW_COLOR 65504
#define NEXTION_CYAN_COLOR   2047

// Page indexes on Nextion display.
#define NEXTION_PAGE_ABOUT 0
#define NEXTION_PAGE_MAIN 1
#define NEXTION_PAGE_LOG 2
#define NEXTION_PAGE_TUNNEL 3
#define NEXTION_PAGE_BATTERY 4
#define NEXTION_PAGE_TRIP 5
#define NEXTION_PAGE_TEMPERATURES 6
#define NEXTION_PAGE_SETTINGS 7
#define NEXTION_PAGE_ERRORS 8
#define NEXTION_PAGE_WIFI 9
#define NEXTION_PAGE_SETTIME 10
#define NEXTION_PAGE_GPS 11
#define NEXTION_PAGE_LIGHT 12

// Image indexes
#define NEXTION_II_LIGHT_OFF 9
#define NEXTION_II_LIGHT_ON 10
#define NEXTION_II_SD_OFF 20
#define NEXTION_II_SD_ON 21
#define NEXTION_II_GPS_ON 22
#define NEXTION_II_GPS_OFF 23

// Gotway pedals mode
#define GW_PEDALS_MODE_HARD 2
#define GW_PEDALS_MODE_MEDIUM 1
#define GW_PEDALS_MODE_SOFT 0
#define GW_SPEED_INCREMENT 3

#define EUC_MAX_POWER 5000
#define EUC_MAX_CHG_POWER 2500

// The minimum distance in meters when a trip is written to the EEPROM when stopping
#define MIN_DISTANCE_TO_SAVE_EEPROM 100L

// Magic to detect empty EEPROM
#define EEPROM_MAGIC 0xDA

// Maximum packets sizes
#define BMS_MAX_PACKET_SIZE 320
#define EUC_MAX_PACKET_SIZE 24

// GPS
#define GPS_RX 32
#define GPS_TX 33
#define GPS_MAX_AGE 15000UL // 15s

// Light
#define PIN_LIGHT_INTENSITY 35

// BLE services; they are the same for BMS and EUC, however, in principle, they are declared separately.
static BLEUUID serviceUUID_EUC("0000ffe0-0000-1000-8000-00805f9b34fb");
static BLEUUID serviceUUID_BMS("0000ffe0-0000-1000-8000-00805f9b34fb");

// Wanted characteristics for notifications.
static BLEUUID charUUID_EUC("0000ffe1-0000-1000-8000-00805f9b34fb");
static BLEUUID charUUID_BMS("0000ffe1-0000-1000-8000-00805f9b34fb");

// BLE addresses of my devices in EUC
static BLEAddress addrUUID_EUC("88:25:84:f0:1f:42");
static BLEAddress addrUUID_BMS("c8:47:8c:ee:61:99");

// BMS requests; initBmsRequest is for information about the device and at the same time send something magic to ensure that the bmsRequest is enough to send
// only once and then the BMS itself sent its status periodically.
static uint8_t initBmsRequest[] = {0x52, 0x12, 0x00, 0xAA, 0x55, 0x90, 0xEB, 0x97, 0x00, 0xE8, 0x13, 0x9E, 0x26, 0xAF, 0xC5, 0x72, 0x44, 0xBC, 0x6D, 0x78, 0x50, 0x66, 0x51};
static uint8_t bmsRequest[] = {0x52, 0x12, 0x00, 0xAA, 0x55, 0x90, 0xEB, 0x96, 0x00, 0x2F, 0x66, 0x8F, 0x8E, 0x4F, 0xA0, 0x34, 0x03, 0x7C, 0x72, 0x20, 0x46, 0x12, 0x4E};

// Initialized addresses of found devices that interest us.
static BLEAddress* pServerEucAddress = NULL;
static BLEAddress* pServerBmsAddress = NULL;

// Sets to true after both BMS and EUC are found.
static boolean doConnect = false;

// Flag of successful connection to both BMS and EUC
static boolean connected = false;

// Characteristics.
static BLERemoteCharacteristic* pRemoteEucCharacteristic;
static BLERemoteCharacteristic* pRemoteBmsCharacteristic;

// RTC
DS3231 rtc;
bool hasRtc = false;

// Structure stored in EEPROM; contains trip data
struct EepromData {
  uint8_t magic;
  uint16_t mTopSpeed;
  uint32_t mDistance;
  uint16_t mMaxPower;
  uint16_t mMaxChargePower;
  uint64_t mRideTime;
  float mLastBatteryPercentRemaining;
  uint32_t mDistanceLastCharge;
  uint64_t mTotalRuntime;
  uint16_t mLightLevelOff;
  uint16_t mLightLevelOn;
} ee;

// Data structure from EUC
struct WheelData {
  uint16_t mSpeed;

  uint32_t mTotalDistance;
  uint32_t mStartDistance;

  double mPhaseCurrent;
  int16_t mTemperature;
  uint16_t mVoltage;

  uint8_t mLightMode;
  uint8_t mPedalsMode;
  int8_t mTiltbackSpeed;

  uint8_t mErrors;
} wd;

// Data structure from BMS
struct BmsData {
  float mTotalVoltage;
  float mCurrent;
  float mCellVoltages [NUM_CELLS];
  float mCellResistances [NUM_CELLS];
  float mMinCellVoltage;
  float mMaxCellVoltage;
  uint8_t mMinVoltageCell;
  uint8_t mMaxVoltageCell;
  uint8_t mMinResistanceCell;
  uint8_t mMaxResistanceCell;
  float mAvgCellVoltage;
  float mDeltaCellVoltage;
  float mTemperature1;
  float mTemperature2;
  float mBmsTemperature;
  float mBalancingCurrent;
  float mPercentRemaining;
  uint16_t mChargeCycles;
  uint16_t mErrors;
} bms;

// Unpacker state
enum UnpackerState {
  unknown,
  collecting,
  done
};

// Current time part in set time
enum CurrentTimePart {
  year,
  month,
  day,
  hour,
  minute,
  second
} currentTimePart;

struct SetTimeStruct {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
} setTimeStruct;

// Structure for silage of data from the BLE characteristic notification, because
// record will never run out entirely in one packet.
struct UnpackerData {
  UnpackerState state;
  uint8_t* buffer;
  uint16_t pos;
  int lastChar;
};


// Unpackers for EUC and BMS
UnpackerData eucUnpacker;
UnpackerData bmsUnpacker;

// Display state.
String curDisplayLine = "";
unsigned long lastDisplayUpdate = 0;
uint8_t curDisplayRow = 0;
uint8_t displayedPage = 0;
uint8_t lastDisplayedPage = 0xFF;

enum BatteryCellMode {
  voltages,
  resistances
} batteryCellMode;

// Last packet times from BMS and EUC
volatile unsigned long lastEucPacketTime = 0UL;
volatile unsigned long lastBmsPacketTime = 0UL;

// Last update of average speed.
unsigned long lastRideTime = 0UL;
// Last update of total runtime.
unsigned long lastTotalRuntimeTime = 0UL;
// Last write to SD card
unsigned long lastSDWrite = 0UL;
// Last light intensity update
unsigned long lastLightUpdate = 0UL;

// GPS
SoftwareSerial gpsSerial (GPS_RX, GPS_TX);
TinyGPS gps;

// Light
RunningMedian<uint16_t,30> lightLevel;
// - automatic light is turned off when any manual light operation is performed
bool autoLight = true;

// BMS errors
#define BMS_ERRORS_SIZE 16
const char* PROGMEM BMS_ERRORS[BMS_ERRORS_SIZE] = {
    "Charge Overtemperature",               // 0000 0000 0000 0001
    "Charge Undertemperature",              // 0000 0000 0000 0010
    "Error 0x00 0x04",                      // 0000 0000 0000 0100
    "Cell Undervoltage",                    // 0000 0000 0000 1000
    "Error 0x00 0x10",                      // 0000 0000 0001 0000
    "Error 0x00 0x20",                      // 0000 0000 0010 0000
    "Error 0x00 0x40",                      // 0000 0000 0100 0000
    "Error 0x00 0x80",                      // 0000 0000 1000 0000
    "Error 0x01 0x00",                      // 0000 0001 0000 0000
    "Error 0x02 0x00",                      // 0000 0010 0000 0000
    "Cell count is not equal to settings",  // 0000 0100 0000 0000
    "Current sensor anomaly",               // 0000 1000 0000 0000
    "Cell Overvoltage",                     // 0001 0000 0000 0000
    "Error 0x20 0x00",                      // 0010 0000 0000 0000
    "Charge overcurrent protection",        // 0100 0000 0000 0000
    "Error 0x80 0x00",                      // 1000 0000 0000 0000
};

// EUC errors
#define EUC_ERRORS_SIZE 8
const char* PROGMEM EUC_ERRORS[EUC_ERRORS_SIZE] = {
    "Over Power",
    "Speed Limit 2",
    "Speed Limit 1",
    "Low Voltage",
    "Over Voltage",
    "Over Temperature",
    "Error Hall Sensors",
    "Transport Mode",
};


// Clears display screen.
void clearScreen(bool refSerialTunnel = true) {
  display.print(F("cls BLACK"));
  displayCommit();

  if (refSerialTunnel && (displayedPage == NEXTION_PAGE_LOG)) {
    // This is the icon to switch to serial tunnel mode to update the display.
    display.print (F("ref p0"));
    displayCommit();
  }

  curDisplayRow = 0;
}

// Commit display command.
void displayCommit() {
  display.write(0xFF);
  display.write(0xFF);
  display.write(0xFF);
}

// Switch display page.
void switchPage (uint8_t page) {
  display.print (F("page "));
  display.print (page);
  displayCommit();
  displayedPage = page;
}

// Logs line on serial and display.
void logLine(String s) {
  Serial.println(s);
  curDisplayLine += s;
  display.print(F("xstr 2,"));
  display.print(curDisplayRow * 16);
  display.print(F(",396,16,1,WHITE,BLACK,0,1,3,\""));
  display.print(curDisplayLine);
  display.print("\"");
  displayCommit();
  curDisplayLine = "";
  curDisplayRow++;
}

// Logs line on serial and display.
void logLine() {
  logLine("");
}

// Logs string on serial and display.
void log(String s) {
  Serial.print(s);
  curDisplayLine += s;
}

// Logs hex byte on serial.
void logHex (uint8_t val) {
  Serial.printf("%02x", val);
}

// Callback that is called when searching for a BLE device.
class EucAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    BLEAddress addr = advertisedDevice.getAddress();

    log(F("DEV: "));
    log(advertisedDevice.getName().c_str());
    log(F(", Addr.: "));
    logLine(addr.toString().c_str());

    for (int i = 0, n = advertisedDevice.getServiceUUIDCount(); i < n; i++) {
      Serial.print(F("  SRV: "));
      Serial.println(advertisedDevice.getServiceUUID(i).toString().c_str());
    }

    if (addr.equals(addrUUID_EUC)) {
      logLine(F("EUC found!"));

      pServerEucAddress = new BLEAddress(addr);
    } else if (addr.equals(addrUUID_BMS)) {
      logLine(F("BMS found!"));

      pServerBmsAddress = new BLEAddress(addr);
    }

    if (pServerEucAddress && pServerBmsAddress) {
      advertisedDevice.getScan()->stop();

      doConnect = true;
    }
  }
};

// Initial settings.
void setup() {
  Serial.begin(115200);
  display.begin(115200);
  gpsSerial.begin(9600);

  delay(2000);

  switchPage (NEXTION_PAGE_LOG);
  clearScreen();

  eucUnpacker.state = UnpackerState::unknown;
  eucUnpacker.buffer = (uint8_t*)malloc(EUC_MAX_PACKET_SIZE * sizeof(uint8_t));
  memset(eucUnpacker.buffer, 0, EUC_MAX_PACKET_SIZE);
  eucUnpacker.pos = 0;

  bmsUnpacker.state = UnpackerState::unknown;
  bmsUnpacker.buffer = (uint8_t*)malloc(BMS_MAX_PACKET_SIZE * sizeof(uint8_t));
  memset(bmsUnpacker.buffer, 0, BMS_MAX_PACKET_SIZE);
  bmsUnpacker.pos = 0;

  logLine(F("Starting EUC dashboard..."));
  BLEDevice::init("");

  logLine(F("Initializing RTC..."));
  if (Wire.begin()) {
    logLine (F("Scanning I2C bus..."));
    int devCount = 0;
    for (byte i = 8; i < 120; i++) {
      Wire.beginTransmission (i);
      int error = Wire.endTransmission();
      if (error == 0) {
        log (F("Found address: "));
        log (String(i));
        log (F(" (0x"));
        log (String (i, HEX));
        logLine (F(")"));
 
        devCount++;
      } else if (error == 4) {
        log (F("ERROR at address: "));
        log (String(i));
        log (F(" (0x"));
        log (String (i, HEX));
        logLine (F(")"));
      }
    }
    log (F("Found "));      
    log (String(devCount));
    logLine (F(" device(s)."));

    if (devCount > 0) {
      rtc.setClockMode (false);
 
      DateTime now = RTClib::now();
      log (F("Current time is "));
      log (String(now.year()));
      log ("-");
      log (String(now.month()));
      log ("-");
      log (String(now.day()));
      log (" ");
      log (String(now.hour()));
      log (":");
      log (String(now.minute()));
      log (":");
      logLine (String(now.second()));

      hasRtc = (now.year() <= 2100);
    } else {
      logLine (F("ERROR; no RTC module found!"));
    }
  } else {
    logLine (F("ERROR; Can't initialize I2C bus!"));
  }

  // Setup SD card
  SD_setup();

  delay (2000);
  clearScreen();
  logLine (F("Scanning BLE devices..."));

  EEPROM.begin (sizeof(EepromData));

  memset(&wd, 0, sizeof(WheelData));
  memset(&bms, 0, sizeof(BmsData));

  EEPROM.get (0, ee);
  if (ee.magic != EEPROM_MAGIC) {
    Serial.printf ("Magic in EEPROM mismatch 0x%02x<>0x%02x. Initializing trip to zero...\r\n", ee.magic, EEPROM_MAGIC);
    ee.magic = EEPROM_MAGIC;
    ee.mDistance = 0;
    ee.mTopSpeed = 0;
    ee.mRideTime = 0;
    ee.mMaxPower = 0;
    ee.mMaxChargePower = 0;
    ee.mLastBatteryPercentRemaining = -1;
    ee.mDistanceLastCharge = 0;
    ee.mTotalRuntime = 0;
    ee.mLightLevelOff = 4095;
    ee.mLightLevelOn = 0;
  }

  // Scan devices
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks (new EucAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan (true);
  pBLEScan->start (SCAN_TIMEOUT);

  logLine(F("Scanning ended..."));
}

/*
   Methods for setting simple elements in Nextion.
*/

void displaySet (String field, uint32_t value) {
  display.print (field);
  display.print (F(".val="));
  display.print (value);
  displayCommit();
}

void displaySet (String field, String value) {
  displaySet (field, value, "");
}

void displaySet (String field, String value, String unit) {
  displaySet ("", field, value, unit);
}

void displaySet (String prefix, String field, String value, String unit) {
  display.print (field);
  display.print (F(".txt=\""));
  display.print (prefix);
  display.print (value);
  display.print (unit);
  display.print ('"');
  displayCommit();
}

void displaySet (String field, double value, int decimalPlaces) {
  displaySet ("", field, value, decimalPlaces, "");
}

void displaySet (String field, double value, int decimalPlaces, String unit) {
  displaySet ("", field, value, decimalPlaces, unit);
}

void displaySet (String prefix, String field, double value, int decimalPlaces, String unit) {
  display.print (field);
  display.print (F(".txt=\""));
  display.print (prefix);
  display.print (value, decimalPlaces);
  display.print (unit);
  display.print ('"');
  displayCommit();
}

void displaySetPco (String field, uint16_t color) {
  display.print (field);
  display.print (F(".pco="));
  display.print (color);
  displayCommit();
}

void displayBatteryCell (uint8_t index) {
  String batField = String(F("bat")) + String(index + 1);

  float val;
  uint8_t red, green;
  if (batteryCellMode == BatteryCellMode::voltages) {
    val = bms.mCellVoltages [index];
    red = bms.mMinVoltageCell;
    green = bms.mMaxVoltageCell;
  } else {
    val = bms.mCellResistances [index];
    green = bms.mMinResistanceCell;
    red = bms.mMaxResistanceCell;
  }

  displaySet (batField, val, 3);

  uint16_t color;
  if (index == red) {
    color = NEXTION_RED_COLOR;
  } else if (index == green) {
    color = NEXTION_GREEN_COLOR;
  } else {
    color = NEXTION_SILVER_COLOR;
  }
  displaySetPco (batField, color);
}

String timeToString (unsigned long t) {
 char str[25];
 
 long h = t / 3600;
 long d = h / 24;
 t = t % 3600;
 int m = t / 60;
 int s = t % 60;
 sprintf(str, "(%02ld) %02ld:%02d:%02d", d, h, m, s);
 return String (str);
}

// Save the trip if at least MIN_DISTANCE_TO_SAVE_EEPROM meter was covered.
void saveTrip() {
  EepromData ed;
  EEPROM.get (0, ed);
  if (abs ((long)ee.mDistance - (long)ed.mDistance) > MIN_DISTANCE_TO_SAVE_EEPROM) {
    EEPROM.put (0, ee);
    EEPROM.commit();
    Serial.print (F("Trip saved to EEPROM..."));
  }
}

// Resets trip.
void resetTrip() {
  wd.mStartDistance = wd.mTotalDistance;
  ee.mDistance = 0;
  ee.mTopSpeed = 0;
  ee.mRideTime = 0;
  ee.mMaxPower = 0;
  ee.mMaxChargePower = 0;
  lastRideTime = 0UL;

  saveTrip();
  Serial.print (F("Trip reset..."));
  switchPage (NEXTION_PAGE_MAIN);
}

// Update set time part highlight
void updateSetTimePart() {
  displaySetPco (F("stYear"), currentTimePart == year ? NEXTION_CYAN_COLOR : NEXTION_YELLOW_COLOR);
  displaySetPco (F("stMonth"), currentTimePart == month ? NEXTION_CYAN_COLOR : NEXTION_YELLOW_COLOR);
  displaySetPco (F("stDay"), currentTimePart == day ? NEXTION_CYAN_COLOR : NEXTION_YELLOW_COLOR);
  displaySetPco (F("stHour"), currentTimePart == hour ? NEXTION_CYAN_COLOR : NEXTION_YELLOW_COLOR);
  displaySetPco (F("stMinute"), currentTimePart == minute ? NEXTION_CYAN_COLOR : NEXTION_YELLOW_COLOR);
  displaySetPco (F("stSecond"), currentTimePart == second ? NEXTION_CYAN_COLOR : NEXTION_YELLOW_COLOR);

  String curVal;
  switch (currentTimePart) {
     case year :
       curVal = F("year");
       break;
     case month :
       curVal = F("month");
       break;
     case day :
       curVal = F("day");
       break;
     case hour :
       curVal = F("hour");
       break;
     case minute :
       curVal = F("minute");
       break;
     case second :
       curVal = F("second");
       break;
  }
  displaySet (F("cur"), curVal);
}

// Add amount to current set time part
void addCurrentSetTime (int amount) {
  switch (currentTimePart) {
     case year :
       setTimeStruct.year += amount;
       setTimeStruct.year = constrain (setTimeStruct.year, 2024, 2100);
       break;
     case month :
       setTimeStruct.month += amount;
       setTimeStruct.month = constrain (setTimeStruct.month, 1, 12);
       break;
     case day :
       setTimeStruct.day += amount;
       setTimeStruct.day = constrain (setTimeStruct.day, 1, 31);
       break;
     case hour :
       setTimeStruct.hour += amount;
       setTimeStruct.hour = constrain (setTimeStruct.hour, 0, 23);
       break;
     case minute :
       setTimeStruct.minute += amount;
       setTimeStruct.minute = constrain (setTimeStruct.minute, 0, 59);
       break;
     case second :
       setTimeStruct.second += amount;
       setTimeStruct.second = constrain (setTimeStruct.second, 0, 59);
       break;
  }
}

// Updates set time form
void updateSetTimeForm() {
  displaySet (F("stYear"), setTimeStruct.year);
  displaySet (F("stMonth"), setTimeStruct.month);
  displaySet (F("stDay"), setTimeStruct.day);
  displaySet (F("stHour"), setTimeStruct.hour);
  displaySet (F("stMinute"), setTimeStruct.minute);
  displaySet (F("stSecond"), setTimeStruct.second);
}

// Updates RTC time
void rtcSetTime() {
  rtc.setYear (setTimeStruct.year - 2000);
  rtc.setMonth (setTimeStruct.month);
  rtc.setDate (setTimeStruct.day);
  rtc.setHour (setTimeStruct.hour);
  rtc.setMinute (setTimeStruct.minute);
  rtc.setSecond (setTimeStruct.second);
}

// Updates auto light
void updateAutoLight() {
  EEPROM.put (0, ee);
  EEPROM.commit();
}


// Handles display events.
void handleDisplay() {
  while (display.available() > 0) {
    uint8_t val = display.read();
    switch (val) {
      case 'L':
        autoLight = false;
        toggleLight();
        break;
      case 'l':
        switchPage (NEXTION_PAGE_LIGHT);
        break;
      case 'D':
        serialTunnel();
        break;
      case 'w':
        wifiAp();
        break;
      case 'H':
        switchPage (NEXTION_PAGE_MAIN);
        break;
      case 'B':
        switchPage (NEXTION_PAGE_BATTERY);
        break;
      case 'R':
        resetTrip();
        break;
      case 'r':
        switchPage (NEXTION_PAGE_TRIP);
        break;
      case 'T':
        switchPage (NEXTION_PAGE_TEMPERATURES);
        break;

      case 't': {
        if (displayedPage == NEXTION_PAGE_SETTIME) {
          rtcSetTime();
          switchPage (NEXTION_PAGE_MAIN);
        } else {
          if (!hasRtc) {
            break;
          }

          currentTimePart = year;
          DateTime now = RTClib::now();

          setTimeStruct.year = now.year();
          setTimeStruct.month = now.month();
          setTimeStruct.day = now.day();
          setTimeStruct.hour = now.hour();
          setTimeStruct.minute = now.minute();
          setTimeStruct.second = now.second();

          switchPage (NEXTION_PAGE_SETTIME);
          updateSetTimeForm();
          updateSetTimePart();
        }
        break;
      }

      case '0' :
        currentTimePart = year;
        updateSetTimePart();
        break;
      case '1' :
        currentTimePart = month;
        updateSetTimePart();
        break;
      case '2' :
        currentTimePart = day;
        updateSetTimePart();
        break;
      case '3' :
        currentTimePart = hour;
        updateSetTimePart();
        break;
      case '4' :
        currentTimePart = minute;
        updateSetTimePart();
        break;
      case '5' :
        currentTimePart = second;
        updateSetTimePart();
        break;

      case 'h':
        wd.mPedalsMode = GW_PEDALS_MODE_HARD;
        updatePedalsMode();
        break;
      case 'm':
        wd.mPedalsMode = GW_PEDALS_MODE_MEDIUM;
        updatePedalsMode();
        break;
      case 's':
        wd.mPedalsMode = GW_PEDALS_MODE_SOFT;
        updatePedalsMode();
        break;

      case '+':
        if (displayedPage == NEXTION_PAGE_SETTINGS) {
          wd.mTiltbackSpeed = min (wd.mTiltbackSpeed + GW_SPEED_INCREMENT, 60);
          updateTiltbackSpeed();
        } else if (displayedPage == NEXTION_PAGE_SETTIME) {
          addCurrentSetTime (1);
          updateSetTimeForm();
        } else if (displayedPage == NEXTION_PAGE_LIGHT) {
          ee.mLightLevelOn += LIGHT_LEVEL_INCREMENT;
          if (ee.mLightLevelOn > 4095) {
            ee.mLightLevelOn = 4095;
          }
          updateAutoLight();
        }
        break;
      case '-':
        if (displayedPage == NEXTION_PAGE_SETTINGS) {
          wd.mTiltbackSpeed = max (wd.mTiltbackSpeed - GW_SPEED_INCREMENT, 0);
          updateTiltbackSpeed();
        } else if (displayedPage == NEXTION_PAGE_SETTIME) {
          addCurrentSetTime (-1);
          updateSetTimeForm();
        } else if (displayedPage == NEXTION_PAGE_LIGHT) {
          if (ee.mLightLevelOn < LIGHT_LEVEL_INCREMENT) {
            ee.mLightLevelOn = 0;
          } else {
            ee.mLightLevelOn -= LIGHT_LEVEL_INCREMENT;
          }
          updateAutoLight();
        }
        break;
      case '*':
        ee.mLightLevelOff += LIGHT_LEVEL_INCREMENT;
        if (ee.mLightLevelOff > 4095) {
          ee.mLightLevelOff = 4095;
        }
        updateAutoLight();
        break;
      case '/':
        if (ee.mLightLevelOff < LIGHT_LEVEL_INCREMENT) {
          ee.mLightLevelOff = 0;
        } else {
          ee.mLightLevelOff -= LIGHT_LEVEL_INCREMENT;
        }
        updateAutoLight();
        break;
      case ',':
        if (!lightLevel.getMedian (ee.mLightLevelOn)) {
          updateAutoLight();
        }
        break;
      case '.':
        if (!lightLevel.getMedian (ee.mLightLevelOff)) {
          updateAutoLight();
        }
        break;
      case 'S':
        switchPage (NEXTION_PAGE_SETTINGS);
        break;
      case 'E' :
        switchPage (NEXTION_PAGE_ERRORS);
        break;
      case 'b':
        if (batteryCellMode == resistances) {
          batteryCellMode = voltages;
        } else {
          batteryCellMode = resistances;
        }
        break;
      case 'g' :
        switchPage(NEXTION_PAGE_GPS);
        break;
      default:
        Serial.printf("<< DISPLAY: %02x\r\n", val);
        break;
    }
  }
}

void loop() {
  if (doConnect) {
    if (connectToServers()) {
      logLine(F("Connected to EUC & BMS..."));
      connected = true;

      // Zde je zobrazen about.
      delay(1000);
      switchPage (NEXTION_PAGE_MAIN);
    } else {
      reboot(F("Failed connect to EUC or BMS..."));
    }

    doConnect = false;
  }

  handleDisplay();

  DateTime currentTime;
  if (hasRtc) {
    currentTime = RTClib::now();
  }

  if (connected) {
    unsigned long now = millis();

    if ((now - lastDisplayUpdate) > DISPLAY_UPDATE) {
      lastDisplayUpdate = now;
      switch (displayedPage) {
        case NEXTION_PAGE_MAIN : {
          double voltage = bms.mTotalVoltage;
          uint16_t speed = wd.mSpeed;
          uint32_t totalDistance = (uint32_t)round(wd.mTotalDistance / 1000.0);
          double phaseCurrent = wd.mPhaseCurrent / 100.0;
          int16_t temperature = wd.mTemperature / 100;
          uint16_t batteryLevel = bms.mPercentRemaining;
          uint32_t distanceSinceCharge = ee.mDistanceLastCharge;
          uint32_t distance = ee.mDistance;

          bool charging = (bms.mCurrent > 0);

          bool hasErrors = bms.mErrors || wd.mErrors;

          int power, load;
          if (charging) {
            power = constrain ((int) round (voltage * bms.mCurrent), 0, EUC_MAX_CHG_POWER);
            load = map (power, 0, EUC_MAX_CHG_POWER, 0, 100);
            if (power > ee.mMaxChargePower) {
              ee.mMaxChargePower = power;
            }
          } else {
            power = constrain ((int) abs (round(voltage * bms.mCurrent)), 0, EUC_MAX_POWER);
            load = -1 * map (power, 0, EUC_MAX_POWER, 0, 100);
            if (power > ee.mMaxPower) {
              ee.mMaxPower = power;
            }
          }

          Serial.printf ("EUC: BV=%.2fV, BL=%d%%, PC=%.2fA, TEMP=%d°C, LOAD=%d%%, SPD=%dkm/h, DIST=%dm, TOT_DIST=%dkm\r\n", voltage, batteryLevel, phaseCurrent, temperature, load, speed, distance, totalDistance);

          if (!hasErrors) {
            displaySet (F("speed"), speed);

            if (hasRtc) {
              display.print(F("timeLabel.txt=\""));
              display.printf ("%02d:%02d", currentTime.hour(), currentTime.minute());
              display.print('"');
              displayCommit();

              display.print(F("dateLabel.txt=\""));
              display.printf ("%04d-%02d-%02d", currentTime.year(), currentTime.month(), currentTime.day());
              display.print('"');
              displayCommit();
            }
          }

          displaySet (F("sch. "), F("kmSinceCharge"), distanceSinceCharge / 1000.0, 1, F("km"));

          displaySet (F("batPercent"), batteryLevel);

          displaySet (F("batVolt"), voltage, 1, F("V"));

          displaySet (F("temp"), temperature);

          displaySet (F("pwm"), load);

          display.print(F("km.txt=\""));
          if (charging) {
            display.print (F("C:"));
            display.print (power);
            display.print ('W');
          } else {
            if (distance < 1000) {
              display.print(distance);
              display.print('m');
            } else {
              display.print(distance / 1000.0, 1);
              display.print(F("km"));
            }
          }
          display.print('"');
          displayCommit();

          displaySetPco (F("km"), charging ? NEXTION_GREEN_COLOR : NEXTION_YELLOW_COLOR);

          displaySet (F("total "), F("totalKm"), String(totalDistance), F("km"));

          display.print (F("light.pic="));
          display.print (wd.mLightMode == 0 ? NEXTION_II_LIGHT_OFF : NEXTION_II_LIGHT_ON);
          displayCommit();

          display.print (F("sdImg.pic="));
          display.print (SD_initialized() ? NEXTION_II_SD_ON : NEXTION_II_SD_OFF);
          displayCommit();

          long lat, lon;
          unsigned long age;
          gps.get_position(&lat, &lon, &age);

          display.print(F("gpsImg.pic="));
          display.print (((age > GPS_MAX_AGE) || (lat == TinyGPS::GPS_INVALID_ANGLE) || (lon == TinyGPS::GPS_INVALID_ANGLE)) ? NEXTION_II_GPS_OFF : NEXTION_II_GPS_ON);
          displayCommit();

          display.print (F("vis err,"));
          display.print (hasErrors ? 1 : 0);
          displayCommit();
          if (hasErrors) {
            String s = "";
            if (bms.mErrors) {
              s += F("BMS ERRORS\r");
            }
            if (wd.mErrors) {
              s += F("EUC ERRORS\r");
            }
            displaySet (F("err"), s);
          }

          break;
        }
        case NEXTION_PAGE_BATTERY : {
          Serial.printf ("BMS: CAP=%.0f%%, U=%.1fV, I=%.1fA, BAL_I=%.3f, T1=%.0f, T2=%.0f, T_BMS=%.0f, CYCLES=%d\r\n", bms.mPercentRemaining, bms.mTotalVoltage, bms.mCurrent, bms.mBalancingCurrent, bms.mTemperature1, bms.mTemperature2, bms.mBmsTemperature, bms.mChargeCycles);

          Serial.printf ("BMS: CELLS: MIN=%.3f, MAX=%.3f, AVG=%.3f, DELTA=%.3f, VOLTAGES: ", bms.mMinCellVoltage, bms.mMaxCellVoltage, bms.mAvgCellVoltage, bms.mDeltaCellVoltage);
          for (uint8_t i = 0; i < NUM_CELLS; i++) {
            Serial.print (bms.mCellVoltages[i], 3);
            if (i < NUM_CELLS - 1) {
              Serial.print (F(", "));
            }
          }
          Serial.println();

          Serial.print (F("BMS: CELLS: RESISTANCES: "));
          for (uint8_t i = 0; i < NUM_CELLS; i++) {
            Serial.print (bms.mCellResistances[i], 3);
            if (i < NUM_CELLS - 1) {
              Serial.print (F(", "));
            }
          }
          Serial.println();

          display.print (F("batInfo1.txt=\""));
          display.printf ("U: %.1fV | I: %.1fA | BI: %.3fA | CY: %d\"", bms.mTotalVoltage, bms.mCurrent, bms.mBalancingCurrent, bms.mChargeCycles);
          displayCommit();

          display.print (F("batInfo2.txt=\""));
          display.printf ("BT1: %.0fC | BT2: %.0fC | BMST: %.0fC\"", bms.mTemperature1, bms.mTemperature2, bms.mBmsTemperature);
          displayCommit();

          displaySet (F("batMin"), bms.mMinCellVoltage, 3);

          displaySet (F("batMax"), bms.mMaxCellVoltage, 3);

          displaySet (F("batAvg"), bms.mAvgCellVoltage, 3);

          displaySet (F("batDiff"), bms.mDeltaCellVoltage, 3);

          displaySet (F("cellsCapt"), (batteryCellMode == BatteryCellMode::voltages) ? F("Voltages:") : F("Resistances:"));

          for (uint8_t i = 0; i < NUM_CELLS; i++) {
            displayBatteryCell (i);
          }

          break;
        }
        case NEXTION_PAGE_TEMPERATURES : {
          displaySet (F("eucTemp"), round (wd.mTemperature / 100.0), 0);

          displaySet (F("batTemp1"), bms.mTemperature1, 0);

          displaySet (F("batTemp2"), bms.mTemperature2, 0);

          displaySet (F("bmsTemp"), bms.mBmsTemperature, 0);

          displaySet (F("dashTemp"), rtc.getTemperature(), 0);

          break;
        }
        case NEXTION_PAGE_SETTINGS : {
          displaySet (F("rSoft"), wd.mPedalsMode == GW_PEDALS_MODE_SOFT);
          displaySet (F("rMedium"), wd.mPedalsMode == GW_PEDALS_MODE_MEDIUM);
          displaySet (F("rHard"), wd.mPedalsMode == GW_PEDALS_MODE_HARD);

          displaySet (F("tiltbackSpeed"), wd.mTiltbackSpeed);

          if (lastDisplayedPage != displayedPage) {
            uint64_t free, used;
            int count;

            SD_info (&used, &free, &count);

            display.print(F("sdInfo.txt=\"SD: "));

            display.print (count);
            display.print (F(" files, "));
            display.print (SD_file_size (free));
            display.print (F(" free"));
          }

          display.print('"');
          displayCommit();
          break;
        }
        case NEXTION_PAGE_TRIP : {
          displaySet (F("maxSpeed"), ee.mTopSpeed, 0, F("km/h"));

          double avgSpeed = 0;
          if (ee.mRideTime > 0) {
            avgSpeed = ((double)ee.mDistance / 1000.0) / (ee.mRideTime / 1000.0 / 3600.0);
          }

          displaySet (F("avgSpeed"), avgSpeed, 0, F("km/h"));

          displaySet (F("km"), ee.mDistance / 1000.0, 1, F("km"));

          displaySet (F("kmSinceCharge"), ee.mDistanceLastCharge / 1000.0, 1, F("km"));

          displaySet (F("totalKm"), wd.mTotalDistance / 1000.0, 0, F("km"));

          displaySet (F("maxPower"), ee.mMaxPower, 0, F("W"));

          displaySet (F("maxChargePower"), ee.mMaxChargePower, 0, F("W"));

          displaySet (F("rideTime"), timeToString (ee.mRideTime / 1000UL));

          displaySet (F("totalRuntime"), timeToString (ee.mTotalRuntime / 1000UL));

          break;
        }
        case NEXTION_PAGE_ERRORS : {
          if (lastDisplayedPage != displayedPage) {
            uint16_t y = 35;
            uint8_t i;
            if (bms.mErrors) {
              for (i = 0; i < BMS_ERRORS_SIZE; i++) {
                if (bms.mErrors & (1 << i)) {
                  display.print(F("xstr 2,"));
                  display.print(y);
                  display.print(F(",396,25,0,RED,BLACK,0,1,3,\"BMS: "));
                  display.print(BMS_ERRORS[i]);
                  display.print("\"");
                  displayCommit();
                  y += 25;
                }
              }
            }

            if (wd.mErrors) {
              for (i = 0; i < EUC_ERRORS_SIZE; i++) {
                if (wd.mErrors & (1 << i)) {
                  display.print(F("xstr 2,"));
                  display.print(y);
                  display.print(F(",396,25,0,RED,BLACK,0,1,3,\"EUC: "));
                  display.print(EUC_ERRORS[i]);
                  display.print("\"");
                  displayCommit();
                  y += 25;
                }
              }
            }
          }

          break;
        }
        case NEXTION_PAGE_GPS : {
          float flat, flon, altitude;
          unsigned long age, hdop; // ms
          unsigned short satellites;


          gps.f_get_position(&flat, &flon, &age);
          hdop = gps.hdop();
          satellites = gps.satellites();
          altitude = gps.f_altitude();

          display.print(F("gpsLoc.txt=\""));
          if ((age < GPS_MAX_AGE) && (flat != TinyGPS::GPS_INVALID_F_ANGLE) && (flon != TinyGPS::GPS_INVALID_F_ANGLE)) {
            display.printf ("%.6fN %.6fE", flat, flon);
          } else {
            display.print ('-');
          }
          display.print ('"');
          displayCommit();

          display.print (F("gpsAlt.txt=\""));
          if (altitude == TinyGPS::GPS_INVALID_F_ALTITUDE) {
            display.print ('-');
          } else {
            display.printf ("%.0fm", altitude);
          }
          display.print ('"');
          displayCommit();

          displaySet (F("gpsSat"), (satellites == TinyGPS::GPS_INVALID_SATELLITES) ? "-" : String(satellites));

          display.print (F("gpsAge.txt=\""));
          if (age == TinyGPS::GPS_INVALID_AGE) {
            display.print (F("not fixed"));
          } else {
            display.printf ("%.0fsec.", age / 1000.0);
          }
          display.print ('"');
          displayCommit();

          display.print (F("gpsHdop.txt=\""));
          if (hdop == TinyGPS::GPS_INVALID_HDOP) {
            display.print ('-');
          } else {
            display.printf ("%.1f", hdop / 100.0);
          }
          display.print ('"');
          displayCommit();

          unsigned long chars;
          unsigned short good_sentences;
          unsigned short failed_cs;
           
          gps.stats(&chars, &good_sentences, &failed_cs);

          display.print (F("gpsInfo.txt=\""));
          display.printf ("Chars: %lu, Sentences: %u, Failed: %u", chars, good_sentences, failed_cs);
          display.print ('"');
          displayCommit();

          Serial.printf ("characters: %lu, sentences: %u, failed_cs: %u\r\n", chars, good_sentences, failed_cs);

          break;
        }
        case NEXTION_PAGE_LIGHT : {
          uint16_t median, lowest;
          if (!lightLevel.getMedian(median) && !lightLevel.getLowest(lowest)) {
            display.print ("lCurrent.txt=\"Med: ");
            display.print (median);
            display.print (", Low: ");
            display.print (lowest);
            display.print ('"');
            displayCommit();
          }

          displaySet (F("lOffLevel"), String(ee.mLightLevelOff));
          displaySet (F("lOnLevel"), String(ee.mLightLevelOn));
          break;
        }
        default:
          break;
      }

      lastDisplayedPage = displayedPage;
    }

    // If it stopped (and traveled more than ${MIN_DISTANCE_TO_SAVE_EEPROM} meters), then the trip is saved.
    if (wd.mSpeed == 0) {
      saveTrip();
    }

    // Riding time update.
    if (wd.mSpeed > 0) {
      if (lastRideTime > 0UL) {
        ee.mRideTime += (now - lastRideTime);
      }
      lastRideTime = now;
    } else {
      lastRideTime = 0UL;
    }

    if (lastTotalRuntimeTime > 0UL) {
      ee.mTotalRuntime += (now - lastTotalRuntimeTime);
    }
    lastTotalRuntimeTime = now;

    // Writing to the log
    if ((displayedPage == NEXTION_PAGE_MAIN) && SD_initialized() && hasRtc && ((now - lastSDWrite) > SD_UPDATE_TIME)) {
      writeDataToLog (currentTime);
      lastSDWrite = now;
    }

    if ((now - lastLightUpdate) > LIGHT_UPDATE_TIME) {
      lightLevel.add(analogRead(PIN_LIGHT_INTENSITY));

      uint16_t lightLevelMedian, lightLevelMin;
      if (autoLight && !lightLevel.getMedian(lightLevelMedian) && !lightLevel.getLowest(lightLevelMin)) {
        if (lightLevelMin < ee.mLightLevelOn) {
          turnLightOn();
        } else if (lightLevelMedian > ee.mLightLevelOff) {
          turnLightOff();
        } 
      }

      lastLightUpdate = now;
    }
  }

  // Check if packets are going
  unsigned long now = millis();

  if ((lastEucPacketTime > 0UL) && (now - lastEucPacketTime > MAX_PACKET_TIME)) {
    reboot (F("Max EUC packet time reached. Rebooting..."));
  }
  if ((lastBmsPacketTime > 0UL) && (now - lastBmsPacketTime > MAX_PACKET_TIME)) {
    reboot (F("Max BMS packet time reached. Rebooting..."));
  }

  while (gpsSerial.available()) {
    gps.encode (gpsSerial.read());
  }

  yield();
}

/*
   Write CSV file with metrics.
*/
void writeDataToLog (DateTime time) {
  char logLine [1024];
  float flat, flon, falt;
  unsigned long age, hdop;
  unsigned short satellites;

  gps.f_get_position(&flat, &flon, &age);
  falt = gps.f_altitude();
  satellites = gps.satellites();
  hdop = gps.hdop();

  if ((age > GPS_MAX_AGE) || (hdop == TinyGPS::GPS_INVALID_HDOP) || (flat == TinyGPS::GPS_INVALID_F_ANGLE) || (flon == TinyGPS::GPS_INVALID_F_ANGLE)) {
    flat = -1;
    flon = -1;
    falt = -1;
    satellites = 0;
    hdop = 0;
  }

  // HH:MM:SS;speed;euc_temp;euc_dist_since_charge;euc_phase_current;bms_voltage;bms_current;battery_level;bms_temp;bat1_temp;bat2_temp;bms_balance_cur;gps_lat;gps_lon;gps_alt;gps_satellites;hdop;
  sprintf (logLine, "%02d:%02d:%02d;%d;%d;%d;%.1f;%.1f;%.1f;%.0f;%.0f;%.0f;%.0f;%.3f;%.6f;%.6f;%.0f;%d;%lu;",
    time.hour(),
    time.minute(),
    time.second(),
    wd.mSpeed,
    wd.mTemperature / 100,
    ee.mDistanceLastCharge,
    wd.mPhaseCurrent / 100.0,
    bms.mTotalVoltage,
    bms.mCurrent,
    bms.mPercentRemaining,
    bms.mBmsTemperature,
    bms.mTemperature1,
    bms.mTemperature2,
    bms.mBalancingCurrent,
    flat, flon, falt,
    satellites, hdop
  );

  SD_write_log (time, logLine);
}

/*
   Notification callbacks for EUC and BMS.
*/
static void notifyEucCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  decodeEuc (pData, length);
}

static void notifyBmsCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  decodeBms (pData, length);
}

// Connects to given server.
BLEClient* connectToServer(BLEAddress& pAddress) {
  log(F("Connecting to "));
  logLine(pAddress.toString().c_str());

  BLEClient* pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  // Connect to the remote BLE Server.
  pClient->connect(pAddress);
  int i = 0;
  while (!pClient->isConnected()) {
    delay(250);
    log(".");
    i++;
    if (i % 80 == 0) {
      logLine();
    }

    if (i > 200) {
      logLine();
      log(F("Timeout..."));
      ESP.restart();
    }
  }
  log(F(" - Connected to server "));
  logLine(pAddress.toString().c_str());

  return pClient;
}

// Logs scanned BLE services.
void logServices(BLEClient* pClient) {
  auto services = pClient->getServices();
  std::map<std::string, BLERemoteService*>::iterator itS = services->begin();
  while (itS != services->end()) {
    log("  ");
    logLine(String(itS->first.c_str()));
    itS++;
  }
}

// Logs the scanned BLE characteristics.
void logCharacteristic(BLERemoteService* pService) {
  logLine(F("Characteristics:"));
  auto characteristics = pService->getCharacteristics();
  std::map<std::string, BLERemoteCharacteristic*>::iterator itCh = characteristics->begin();
  while (itCh != characteristics->end()) {
    log("  ");
    logLine(itCh->first.c_str());
    itCh++;
  }
}

// Write request to the given characteristic.
void writeCharacteristic (BLERemoteCharacteristic* ch, uint8_t* data, size_t len) {
  Serial.print (F(">>> "));
  Serial.print (ch->toString().c_str());
  Serial.print (F(": "));
  for (int i = 0; i < len; i++) {
    logHex (data[i]);
  }
  Serial.println();

  ch->writeValue (data, len, false);
}

// Set up connections to BMS and EUC.
bool connectToServers() {
  BLEClient* pEucClient = connectToServer(*pServerEucAddress);
  BLEClient* pBmsClient = connectToServer(*pServerBmsAddress);

  clearScreen();

  logLine(F("EUC Services:"));
  logServices(pEucClient);

  BLERemoteService* pRemoteEucService = pEucClient->getService(serviceUUID_EUC);
  if (pRemoteEucService == nullptr) {
    log(F("Failed to find service UUID of EUC: "));
    logLine(serviceUUID_EUC.toString().c_str());
    return false;
  }
  logLine(F(" - Found EUC service"));
  logCharacteristic(pRemoteEucService);

  pRemoteEucCharacteristic = pRemoteEucService->getCharacteristic(charUUID_EUC);
  if (pRemoteEucCharacteristic == nullptr) {
    log(F("Failed to find characteristic UUID for EUC: "));
    logLine(charUUID_EUC.toString().c_str());
    return false;
  }
  logLine(F(" - Found characteristic"));

  pRemoteEucCharacteristic->registerForNotify(notifyEucCallback);

  delay (1000);
  clearScreen();

  logLine(F("BMS Services:"));
  logServices(pBmsClient);

  BLERemoteService* pRemoteBmsService = pBmsClient->getService(serviceUUID_BMS);
  if (pRemoteBmsService == nullptr) {
    log(F("Failed to find service UUID of BMS: "));
    logLine(serviceUUID_BMS.toString().c_str());
    return false;
  }
  logLine(F(" - Found BMS service"));
  logCharacteristic(pRemoteBmsService);

  pRemoteBmsCharacteristic = pRemoteBmsService->getCharacteristic(charUUID_BMS);
  if (pRemoteBmsCharacteristic == nullptr) {
    log(F("Failed to find characteristic UUID for BMS: "));
    logLine(charUUID_BMS.toString().c_str());
    return false;
  }
  logLine(F(" - Found characteristic"));

  pRemoteBmsCharacteristic->registerForNotify(notifyBmsCallback);

  writeCharacteristic (pRemoteBmsCharacteristic, initBmsRequest, sizeof(initBmsRequest));
  delay (1000);
  writeCharacteristic (pRemoteBmsCharacteristic, bmsRequest, sizeof(bmsRequest));

  return true;
}

// Restart ESP.
void reboot(String cause) {
  saveTrip();
  switchPage (NEXTION_PAGE_LOG);
  clearScreen();
  logLine(cause);
  delay(2000);
  ESP.restart();
}

// Turns the light on/off on the EUC.
void toggleLight() {
  Serial.println(F("Toggle light"));
  if (wd.mLightMode == 0) {
    turnLightOn();
  } else {
    turnLightOff();
  }
}

void turnLightOn() {
  if (wd.mLightMode == 0) {
    pRemoteEucCharacteristic->writeValue((uint8_t)'Q');
  }
}

void turnLightOff() {
  if (wd.mLightMode > 0) {
    pRemoteEucCharacteristic->writeValue((uint8_t)'E');
  }
} 

// Updates pedal mode
void updatePedalsMode() {
  uint8_t command;

  switch (wd.mPedalsMode) {
      case GW_PEDALS_MODE_HARD : command = 'h'; break;
      case GW_PEDALS_MODE_MEDIUM : command = 'f'; break;
      case GW_PEDALS_MODE_SOFT : command = 's'; break;
      default :
        return;
  }

  pRemoteEucCharacteristic->writeValue (command);
}

// Updates tiltback speed
void updateTiltbackSpeed() {
  if (wd.mTiltbackSpeed > 0) {
    uint8_t h = (uint8_t) ((wd.mTiltbackSpeed / 10) + 0x30);
    uint8_t l = (uint8_t) ((wd.mTiltbackSpeed % 10) + 0x30);

    pRemoteEucCharacteristic->writeValue ('W');
    pRemoteEucCharacteristic->writeValue ('Y');
    pRemoteEucCharacteristic->writeValue (h);
    pRemoteEucCharacteristic->writeValue (l);
  } else {
    pRemoteEucCharacteristic->writeValue ('"');
  }
}

// Start the serial tunnel to update the display.
void serialTunnel() {
  if (pRemoteBmsCharacteristic) {
    pRemoteBmsCharacteristic->registerForNotify(NULL);
  }

  if (pRemoteEucCharacteristic) {
    pRemoteEucCharacteristic->registerForNotify(NULL);
  }

  switchPage (NEXTION_PAGE_TUNNEL);
  delay(1000);

  while (true) {
    if (Serial.available()) {
      display.write(Serial.read());
    }

    if (display.available()) {
      Serial.write(display.read());
    }
  }
}

// Create WiFi AP for SD card access.
void wifiAp() {
  if (pRemoteBmsCharacteristic) {
    pRemoteBmsCharacteristic->registerForNotify(NULL);
  }

  if (pRemoteEucCharacteristic) {
    pRemoteEucCharacteristic->registerForNotify(NULL);
  }

  SD_flush();

  switchPage(NEXTION_PAGE_LOG);
  clearScreen (false);
  logLine(F("Starting WiFi access point..."));
  SD_setup_wifi();
  delay(1000);

  ArduinoOTA.setHostname(SERVERNAME);
  ArduinoOTA.setPassword(WIFI_PASSWORD);
  ArduinoOTA.begin();

  switchPage(NEXTION_PAGE_WIFI);

  while (true) {
    ArduinoOTA.handle();
    SD_loop_wifi();
  }
}

/*
  Conversion methods for BigEndian that uses Begode.
*/

uint32_t longFromBytesBE(uint8_t* bytes, int starting, size_t length) {
  if (length >= starting + 4) {
    uint8_t arr[4];
    arr[0] = bytes[starting + 3];
    arr[1] = bytes[starting + 2];
    arr[2] = bytes[starting + 1];
    arr[3] = bytes[starting];

    uint32_t resl;
    memcpy(&resl, arr, 4);
    return resl;
  }
  return 0;
}

uint16_t shortFromBytesBE(uint8_t* bytes, int starting, size_t length) {
  if (length >= starting + 2) {
    return (uint16_t)(bytes[starting] << 8) | (bytes[starting + 1] & 0xFF);
  }
  return 0;
}

int16_t signedShortFromBytesBE(uint8_t* bytes, int starting, size_t length) {
  if (length >= starting + 2) {
    uint8_t byte1 = bytes[starting] & 0xFF;
    uint8_t byte2 = bytes[starting + 1] & 0xFF;
    return (int16_t)(byte1 << 8) | byte2;
  }
  return 0;
}

// Updates the maximum speed.
void updateTopSpeed() {
  if (wd.mSpeed > 60) {
    return;
  }

  if (ee.mTopSpeed < wd.mSpeed) {
    ee.mTopSpeed = wd.mSpeed;
  }
}

// Unpacker for EUC packets.
bool eucUnpackerAddChar(uint8_t c) {
  if (eucUnpacker.state == UnpackerState::collecting) {
    eucUnpacker.buffer[eucUnpacker.pos++] = c;
    eucUnpacker.lastChar = c;
    size_t size = eucUnpacker.pos;
    if ((size == 20 && c != 0x18) || (size > 20 && size <= EUC_MAX_PACKET_SIZE && c != 0x5A)) {
      Serial.println(F("Invalid frame footer (expected 18 5A 5A 5A 5A)"));
      eucUnpacker.state = UnpackerState::unknown;
      return false;
    }
    if (size == EUC_MAX_PACKET_SIZE) {
      eucUnpacker.state = UnpackerState::done;
      return true;
    }
  } else {
    if ((c == 0xAA) && (eucUnpacker.lastChar == 0x55)) {
      eucUnpacker.pos = 0;
      eucUnpacker.buffer[eucUnpacker.pos++] = 0x55;
      eucUnpacker.buffer[eucUnpacker.pos++] = 0xAA;
      eucUnpacker.state = UnpackerState::collecting;
    }
    eucUnpacker.lastChar = c;
  }
  return false;
}

// Method for decoding EUC communication.
void decodeEuc(uint8_t* pData, size_t length) {
  for (int i = 0; i < length; i++) {
    uint8_t c = pData[i];

    if (eucUnpackerAddChar(c)) {
      processEucPacket(eucUnpacker.buffer, eucUnpacker.pos);
    }
  }
}

// Processing the complete Begode packet.
void processEucPacket(uint8_t* buff, size_t length) {
  lastEucPacketTime = millis();

  if (buff[18] == 0x00) {
    uint16_t voltage = shortFromBytesBE(buff, 2, length);
    uint16_t speed = (uint16_t)abs((int16_t)round(signedShortFromBytesBE(buff, 4, length) * 3.6 / 100.0));
    int16_t phaseCurrent = signedShortFromBytesBE(buff, 10, length) * -1;
    int16_t temperature = (int16_t)round((((float)signedShortFromBytesBE(buff, 12, length) / 340.0) + 36.53) * 100);  // mpu6050

    voltage = (uint16_t) round((double)voltage * 1.5);

    wd.mSpeed = speed;
    updateTopSpeed();

    wd.mTemperature = temperature;
    wd.mPhaseCurrent = phaseCurrent;
    wd.mVoltage = voltage;

  } else if (buff[18] == 0x04) {
    uint32_t totalDistance = longFromBytesBE(buff, 2, length);
    wd.mTotalDistance = totalDistance;

    uint16_t settings = shortFromBytesBE(buff, 6, length);
    wd.mPedalsMode = (settings >> 13) & 0x03;

    wd.mTiltbackSpeed = shortFromBytesBE(buff, 10, length);
    if (wd.mTiltbackSpeed >= 100) {
      wd.mTiltbackSpeed = 0;
    }

    wd.mErrors = buff[12] & 0xFF;

    wd.mLightMode = buff[15] & 0x03;

    if (wd.mStartDistance > 0) {
      uint32_t dist = totalDistance - wd.mStartDistance;
      ee.mDistance += dist;
      ee.mDistanceLastCharge += dist; 
    }

    wd.mStartDistance = totalDistance;
  }
}

// Write buffer in HEX to Serial.
void dumpData(uint8_t* buffer, size_t size) {
  for (int i = 0; i < size; i++) {
    logHex (buffer[i]);
  }
}

// crc8, which uses BMS to check the packet.
uint8_t crc8(uint8_t* buffer, size_t size) {
  uint8_t resl = 0;
  for (size_t i = 0; i < size; i++) {
    resl += buffer[i];
  }
  return resl;
}

// Unpacker for BMS packets.
bool bmsUnpackerAddChar(uint8_t c) {
  if (bmsUnpacker.state == UnpackerState::collecting) {
    bmsUnpacker.buffer[bmsUnpacker.pos++] = c;
    bmsUnpacker.lastChar = c;
    size_t size = bmsUnpacker.pos;

    if ((size == 4) && ((bmsUnpacker.buffer[2] != 0xeb) || (bmsUnpacker.buffer[3] != 0x90))) {
      Serial.print(F("Bad packet: "));
      dumpData(bmsUnpacker.buffer, 4);
      Serial.println();
      bmsUnpacker.state = UnpackerState::unknown;
      return false;
    }

    if (size == 300) {
      uint8_t dataCrc = bmsUnpacker.buffer[size - 1];
      uint8_t calcCrc = crc8(bmsUnpacker.buffer, size - 1);

      if (dataCrc == calcCrc) {
        bmsUnpacker.state = UnpackerState::done;
        return true;
      } else {
        Serial.print(F("Bad checksum (len: 300) (data: "));
        logHex (dataCrc);
        Serial.print(F(", calc: "));
        logHex (calcCrc);
        Serial.println(')');
        return false;
      }
    } else if (size == BMS_MAX_PACKET_SIZE) {
      uint8_t dataCrc = bmsUnpacker.buffer[size - 1];
      uint8_t calcCrc = crc8(bmsUnpacker.buffer, size - 1);

      if (dataCrc == calcCrc) {
        bmsUnpacker.state = UnpackerState::done;
        return true;
      } else {
        Serial.print(F("Bad checksum (data: "));
        logHex (dataCrc);
        Serial.print(F(", calc: "));
        logHex (calcCrc);
        Serial.println(')');
        bmsUnpacker.state = UnpackerState::unknown;
        return false;
      }
    }
  } else {
    if ((c == 0xAA) && (bmsUnpacker.lastChar == 0x55)) {
      bmsUnpacker.pos = 0;
      bmsUnpacker.buffer[bmsUnpacker.pos++] = 0x55;
      bmsUnpacker.buffer[bmsUnpacker.pos++] = 0xAA;
      bmsUnpacker.state = UnpackerState::collecting;
    }
    bmsUnpacker.lastChar = c;
  }
  return false;
}

// Load and decode the BMS packet.
void decodeBms(uint8_t* pData, size_t length) {
  for (int i = 0; i < length; i++) {
    uint8_t c = pData[i];

    if (bmsUnpackerAddChar(c)) {
      processBmsPacket(bmsUnpacker.buffer, bmsUnpacker.pos);
    }
  }
}

// Process the BMS packet.
void processBmsPacket(uint8_t* pData, size_t length) {
  lastBmsPacketTime = millis();

  uint8_t packetType = pData[4];
  switch (packetType) {
    case 0x01:
      Serial.println (F("Ignoring packet 0x01."));
      break;
    case 0x03:
      Serial.println (F("Ignoring packet 0x03."));
      break;
    case 0x02:
      processBms02Data (pData, length);
      break;

    default:
      logHex (packetType);
      Serial.print(F(" ??? ("));
      Serial.print(length);
      Serial.print(F("): "));
      for (int i = 0; i < length; i++) {
        logHex (pData[i]);
      }
      Serial.println();
    break;
  }
}

// Process the 0x02 BMS packet.
// https://github.com/syssi/esphome-jk-bms/blob/main/components/jk_bms_ble/jk_bms_ble.cpp
void processBms02Data (uint8_t* data, size_t length) {
  auto jk_get_16bit = [&](size_t i) -> uint16_t { return (uint16_t(data[i + 1]) << 8) | (uint16_t(data[i + 0]) << 0); };
  auto jk_get_32bit = [&](size_t i) -> uint32_t {
    return (uint32_t(jk_get_16bit(i + 2)) << 16) | (uint32_t(jk_get_16bit(i + 0)) << 0);
  };

  uint8_t offset = 16;

  bms.mMinCellVoltage = 100.0f;
  bms.mMaxCellVoltage = -100.0f;
  float minCellResistance = 100.0f;
  float maxCellResistance = -100.0f;
  for (uint8_t i = 0; i < NUM_CELLS; i++) {
    float cell_voltage = (float) jk_get_16bit(i * 2 + 6) * 0.001f;
    float cell_resistance = (float) jk_get_16bit(i * 2 + 64 + offset) * 0.001f;

    if ((cell_voltage > 0) && (cell_voltage < bms.mMinCellVoltage)) {
      bms.mMinCellVoltage = cell_voltage;
      bms.mMinVoltageCell = i;
    }

    if (cell_voltage > bms.mMaxCellVoltage) {
      bms.mMaxCellVoltage = cell_voltage;
      bms.mMaxVoltageCell = i;
    }

    if (cell_resistance < minCellResistance) {
      minCellResistance = cell_resistance;
      bms.mMinResistanceCell = i;
    }

    if (cell_resistance > maxCellResistance) {
      maxCellResistance = cell_resistance;
      bms.mMaxResistanceCell = i;
    }

    bms.mCellVoltages[i] = cell_voltage;
    bms.mCellResistances[i] = cell_resistance;
  }

  bms.mAvgCellVoltage = (float) jk_get_16bit(58 + offset) * 0.001f;
  bms.mDeltaCellVoltage = (float) jk_get_16bit(60 + offset) * 0.001f;

  offset = offset * 2;

  bms.mTotalVoltage = (float) jk_get_32bit(118 + offset) * 0.001f;
  bms.mCurrent = (float) ((int32_t) jk_get_32bit(126 + offset)) * 0.001f;

  bms.mTemperature1 = (float) ((int16_t) jk_get_16bit(130 + offset)) * 0.1f;
  bms.mTemperature2 = (float) ((int16_t) jk_get_16bit(132 + offset)) * 0.1f;
  bms.mBmsTemperature = (float) ((int16_t) jk_get_16bit(112 + offset)) * 0.1f;

  bms.mBalancingCurrent = (float) ((int16_t) jk_get_16bit(138 + offset)) * 0.001f;

  bms.mErrors = ((uint16_t(jk_get_16bit (134 + offset)) << 8) | (uint16_t (jk_get_16bit (135 + offset) << 0)));

  bms.mPercentRemaining = (float) data[141 + offset];
  bms.mChargeCycles = (uint16_t) jk_get_32bit(150 + offset);

  if (bms.mPercentRemaining - ASSUME_CHARGED_PERCENT > ee.mLastBatteryPercentRemaining) {
    ee.mDistanceLastCharge = 0;
  }

  ee.mLastBatteryPercentRemaining = bms.mPercentRemaining;
}

