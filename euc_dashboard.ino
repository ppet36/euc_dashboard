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

// Frequency of display update.
#define DISPLAY_UPDATE 500

// Time in milliseconds when a packet must be sent from EUC/BMS otherwise it is taken as
// lose connection and reboot.
#define MAX_PACKET_TIME 10000

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

// Page indexes on Nextion display.
#define NEXTION_PAGE_ABOUT 0
#define NEXTION_PAGE_MAIN 1
#define NEXTION_PAGE_LOG 2
#define NEXTION_PAGE_TUNNEL 3
#define NEXTION_PAGE_BATTERY 4
#define NEXTION_PAGE_CONFIRM_RESET_TRIP 5
#define NEXTION_PAGE_TEMPERATURES 6
#define NEXTION_PAGE_SETTINGS 7

// Image indexes
#define NEXTION_II_LIGHT_OFF 9
#define NEXTION_II_LIGHT_ON 10

// Gotway pedals mode
#define GW_PEDALS_MODE_HARD 2
#define GW_PEDALS_MODE_MEDIUM 1
#define GW_PEDALS_MODE_SOFT 0
#define GW_SPEED_INCREMENT 3
#define GW_MAX_POWER 5000

// The minimum distance in meters when a trip is written to the EEPROM when stopping
#define MIN_DISTANCE_TO_SAVE_EEPROM 100L

// Magic to detect empty EEPROM
#define EEPROM_MAGIC 0xD2

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

// Structure stored in EEPROM; contains trip data
struct EepromData {
  uint8_t magic;
  uint16_t mTopSpeed;
  uint32_t mDistance;
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
} bms;

// Unpacker state
enum UnpackerState {
  unknown,
  collecting,
  done
};

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
uint8_t curDisplayRow = 0;
uint8_t displayedPage = 0;

enum BatteryCellMode {
  voltages,
  resistances
} batteryCellMode;

// Last packet times from BMS and EUC
volatile unsigned long lastEucPacketTime = 0UL;
volatile unsigned long lastBmsPacketTime = 0UL;

// Clears display screen.
void clearScreen() {
  display.print(F("cls BLACK"));
  displayCommit();

  if (displayedPage == NEXTION_PAGE_LOG) {
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

  EEPROM.begin (sizeof(EepromData));

  switchPage (NEXTION_PAGE_LOG);
  clearScreen();

  eucUnpacker.state = UnpackerState::unknown;
  eucUnpacker.buffer = (uint8_t*)malloc(24 * sizeof(uint8_t));
  memset(eucUnpacker.buffer, 0, 24);
  eucUnpacker.pos = 0;

  bmsUnpacker.state = UnpackerState::unknown;
  bmsUnpacker.buffer = (uint8_t*)malloc(320 * sizeof(uint8_t));
  memset(bmsUnpacker.buffer, 0, 320);
  bmsUnpacker.pos = 0;

  // This delay is here because it will run at the same time as the EUC so it has time to start.
  delay(2000);

  logLine(F("Starting EUC dashboard..."));
  BLEDevice::init("");

  memset(&wd, 0, sizeof(WheelData));
  memset(&bms, 0, sizeof(BmsData));

  EEPROM.get (0, ee);
  if (ee.magic != EEPROM_MAGIC) {
    Serial.printf ("Magic in EEPROM mismatch 0x%02x<>0x%02x. Initializing trip to zero...\n", ee.magic, EEPROM_MAGIC);
    ee.magic = EEPROM_MAGIC;
    ee.mDistance = 0;
    ee.mTopSpeed = 0;
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
  display.print (field);
  display.print (F(".txt=\""));
  display.print (value);
  display.print ('"');
  displayCommit();
}

void displaySet (String field, double value, int decimalPlaces) {
  display.print (field);
  display.print (F(".txt=\""));
  display.print (value, decimalPlaces);
  display.print ('"');
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

  display.print (batField);
  display.print (F(".pco="));
  if (index == red) {
    display.print (NEXTION_RED_COLOR);
  } else if (index == green) {
    display.print (NEXTION_GREEN_COLOR);
  } else {
    display.print (NEXTION_SILVER_COLOR);
  }
  displayCommit();
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

  saveTrip();
  Serial.print (F("Trip reset..."));
  switchPage (NEXTION_PAGE_MAIN);
}

// Handles display events.
void handleDisplay() {
  while (display.available() > 0) {
    uint8_t val = display.read();
    switch (val) {
      case 'L':
        toggleLight();
        break;
      case 'D':
        serialTunnel();
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
        switchPage (NEXTION_PAGE_CONFIRM_RESET_TRIP);
        break;
      case 'T':
        switchPage (NEXTION_PAGE_TEMPERATURES);
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
        wd.mTiltbackSpeed = min (wd.mTiltbackSpeed + GW_SPEED_INCREMENT, 60);
        updateTiltbackSpeed();
        break;
      case '-':
        wd.mTiltbackSpeed = max (wd.mTiltbackSpeed - GW_SPEED_INCREMENT, 0);
        updateTiltbackSpeed();
        break;
      case 'S':
        switchPage (NEXTION_PAGE_SETTINGS);
        break;
      case 'b':
        if (batteryCellMode == BatteryCellMode::resistances) {
          batteryCellMode = BatteryCellMode::voltages;
        } else {
          batteryCellMode = BatteryCellMode::resistances;
        }
        break;
      default:
        Serial.printf("<< DISPLAY: %02x\n", val);
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

  if (connected) {
    switch (displayedPage) {
      case NEXTION_PAGE_MAIN : {
        double voltage = bms.mTotalVoltage;
        uint16_t speed = wd.mSpeed;
        int32_t totalDistance = (int32_t)round(wd.mTotalDistance / 1000.0);
        double phaseCurrent = wd.mPhaseCurrent / 100.0;
        int16_t temperature = wd.mTemperature / 100;
        uint16_t batteryLevel = bms.mPercentRemaining;
        uint16_t topSpeed = ee.mTopSpeed;
        uint32_t distance = ee.mDistance;

        uint32_t power = (uint32_t) round (bms.mTotalVoltage * bms.mCurrent);
        uint32_t load = map (power, 0, GW_MAX_POWER, 0, 100);

        Serial.printf ("EUC: BV=%.2fV, BL=%d%%, PC=%.2fA, TEMP=%d°C, LOAD=%d%%, SPD=%dkm/h, TOP_SPD=%dkm/h, DIST=%dm, TOT_DIST=%dkm\n", voltage, batteryLevel, phaseCurrent, temperature, load, speed, topSpeed, distance, totalDistance);

        displaySet (F("speed"), speed);

        display.print (F("topSpeed.txt=\"max. "));
        display.print (topSpeed);
        display.print(F("km/h\""));
        displayCommit();

        displaySet (F("batPercent"), batteryLevel);

        display.print(F("batVolt.txt=\""));
        display.print(voltage, 1);
        display.print("V\"");
        displayCommit();

        displaySet (F("temp"), temperature);

        displaySet (F("pwm"), load);

        display.print(F("km.txt=\""));
        if (distance < 1000) {
          display.print(distance);
          display.print("m");
        } else {
          display.print(distance / 1000.0, 1);
          display.print("km");
        }
        display.print('"');
        displayCommit();

        display.print(F("totalKm.txt=\"total "));
        display.print(totalDistance);
        display.print("km\"");
        displayCommit();

        display.print (F("light.pic="));
        display.print (wd.mLightMode == 0 ? NEXTION_II_LIGHT_OFF : NEXTION_II_LIGHT_ON);
        displayCommit();

        break;
      }
      case NEXTION_PAGE_BATTERY : {
        Serial.printf ("BMS: CAP=%.0f%%, U=%.1fV, I=%.1fA, BAL_I=%.3f, T1=%.0f, T2=%.0f, T_BMS=%.0f, CYCLES=%d\n", bms.mPercentRemaining, bms.mTotalVoltage, bms.mCurrent, bms.mBalancingCurrent, bms.mTemperature1, bms.mTemperature2, bms.mBmsTemperature, bms.mChargeCycles);

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

        break;
      }
      case NEXTION_PAGE_SETTINGS : {
        displaySet (F("rSoft"), wd.mPedalsMode == GW_PEDALS_MODE_SOFT);
        displaySet (F("rMedium"), wd.mPedalsMode == GW_PEDALS_MODE_MEDIUM);
        displaySet (F("rHard"), wd.mPedalsMode == GW_PEDALS_MODE_HARD);

        displaySet (F("tiltbackSpeed"), wd.mTiltbackSpeed);
        
        break;
      }
      default:
        break;
    }

    // Pokud se zastavilo (a ujelo vic jak ${MIN_DISTANCE_TO_SAVE_EEPROM} metru), tak se ulozi trip.
    if (wd.mSpeed == 0) {
      saveTrip();
    }
  }

  // Kontrola zdali chodi pakety.
  unsigned long now = millis();

  if ((lastEucPacketTime > 0UL) && (now - lastEucPacketTime > MAX_PACKET_TIME)) {
    reboot (F("Max EUC packet time reached. Rebooting..."));
  }
  if ((lastBmsPacketTime > 0UL) && (now - lastBmsPacketTime > MAX_PACKET_TIME)) {
    reboot (F("Max BMS packet time reached. Rebooting..."));
  }

  delay(DISPLAY_UPDATE);
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
    pRemoteEucCharacteristic->writeValue((uint8_t)'Q');
  } else {
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

/*
  Conversion methods for BigEndian that use Begode.
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
    if ((size == 20 && c != 0x18) || (size > 20 && size <= 24 && c != 0x5A)) {
      Serial.println(F("Invalid frame footer (expected 18 5A 5A 5A 5A)"));
      eucUnpacker.state = UnpackerState::unknown;
      return false;
    }
    if (size == 24) {
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

    wd.mLightMode = buff[15] & 0x03;

    if (wd.mStartDistance > 0) {
      ee.mDistance += totalDistance - wd.mStartDistance;
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
    } else if (size == 320) {
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

// Function to convert uint32 to float for BMS.
float ieee_float_(uint32_t f) {
  float ret;
  memcpy(&ret, &f, sizeof(float));
  return ret;
}

// Process the 0x02 BMS packet.
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

  bms.mPercentRemaining = (float) data[141 + offset];
  bms.mChargeCycles = (uint16_t) jk_get_32bit(150 + offset);
}

/*
    Taken from: https://github.com/Wheellog/Wheellog.Android

    Gotway/Begode reverse-engineered protocol

    Gotway uses byte stream from a serial port via Serial-to-BLE adapter.
    There are two types of frames, A and B. Normally they alternate.
    Most numeric values are encoded as Big Endian (BE) 16 or 32 bit integers.
    The protocol has no checksums.

    Since the BLE adapter has no serial flow control and has limited input buffer,
    data come in variable-size chunks with arbitrary delays between chunks. Some
    bytes may even be lost in case of BLE transmit buffer overflow.

         0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23
        -----------------------------------------------------------------------
     A: 55 AA 19 F0 00 00 00 00 00 00 01 2C FD CA 00 01 FF F8 00 18 5A 5A 5A 5A
     B: 55 AA 00 0A 4A 12 48 00 1C 20 00 2A 00 03 00 07 00 08 04 18 5A 5A 5A 5A
     A: 55 AA 19 F0 00 00 00 00 00 00 00 F0 FD D2 00 01 FF F8 00 18 5A 5A 5A 5A
     B: 55 AA 00 0A 4A 12 48 00 1C 20 00 2A 00 03 00 07 00 08 04 18 5A 5A 5A 5A
        ....

    Frame A:
        Bytes 0-1:   frame header, 55 AA
        Bytes 2-3:   BE voltage, fixed point, 1/100th (assumes 67.2 battery, rescale for other voltages)
        Bytes 4-5:   BE speed, fixed point, 3.6 * value / 100 km/h
        Bytes 6-9:   BE distance, 32bit fixed point, meters
        Bytes 10-11: BE current, signed fixed point, 1/100th amperes
        Bytes 12-13: BE temperature, (value / 340 + 36.53) / 100, Celsius degrees (MPU6050 native data)
        Bytes 14-17: unknown
        Byte  18:    frame type, 00 for frame A
        Byte  19:    18 frame footer
        Bytes 20-23: frame footer, 5A 5A 5A 5A

    Frame B:
        Bytes 0-1:   frame header, 55 AA
        Bytes 2-5:   BE total distance, 32bit fixed point, meters
        Byte  6:     pedals mode (high nibble), speed alarms (low nibble)
        Bytes 7-12:  unknown
        Byte  13:    LED mode
        Bytes 14-17: unknown
        Byte  18:    frame type, 04 for frame B
        Byte  19:    18 frame footer
        Bytes 20-23: frame footer, 5A 5A 5A 5A

    Unknown bytes may carry out other data, but currently not used by the parser.
*/
