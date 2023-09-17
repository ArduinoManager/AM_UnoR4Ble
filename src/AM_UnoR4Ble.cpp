/*
 *
 * AMController libraries, example sketches (“The Software”) and the related documentation (“The Documentation”) are supplied to you 
 * by the Author in consideration of your agreement to the following terms, and your use or installation of The Software and the use of The Documentation 
 * constitutes acceptance of these terms.  
 * If you do not agree with these terms, please do not use or install The Software.
 * The Author grants you a personal, non-exclusive license, under author's copyrights in this original software, to use The Software. 
 * Except as expressly stated in this notice, no other rights or licenses, express or implied, are granted by the Author, including but not limited to any 
 * patent rights that may be infringed by your derivative works or by other works in which The Software may be incorporated.
 * The Software and the Documentation are provided by the Author on an "AS IS" basis.  THE AUTHOR MAKES NO WARRANTIES, EXPRESS OR IMPLIED, INCLUDING WITHOUT 
 * LIMITATION THE IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE, REGARDING THE SOFTWARE OR ITS USE AND OPERATION 
 * ALONE OR IN COMBINATION WITH YOUR PRODUCTS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, INDIRECT, INCIDENTAL OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) ARISING IN ANY WAY OUT OF THE USE, 
 * REPRODUCTION AND MODIFICATION OF THE SOFTWARE AND OR OF THE DOCUMENTATION, HOWEVER CAUSED AND WHETHER UNDER THEORY OF CONTRACT, TORT (INCLUDING NEGLIGENCE), 
 * STRICT LIABILITY OR OTHERWISE, EVEN IF THE AUTHOR HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

   Author: Fabrizio Boco - fabboco@gmail.com

   All rights reserved

*/
#include "AM_UnoR4Ble.h"


static void connectHandler(BLEDevice central);
static void disconnectHandler(BLEDevice central);
static void characteristicWritten(BLEDevice central, BLECharacteristic characteristic);

static AMController *myGlobal;

AMController::AMController(
  void (*doWork)(void),
  void (*doSync)(),
  void (*processIncomingMessages)(char *variable, char *value),
  void (*processOutgoingMessages)(void),
  void (*deviceConnected)(void),
  void (*deviceDisconnected)(void)) {
  _doWork = doWork;
  _doSync = doSync;
  _processIncomingMessages = processIncomingMessages;
  _processOutgoingMessages = processOutgoingMessages;
  _deviceConnected = deviceConnected;
  _deviceDisconnected = deviceDisconnected;
  _sync = false;

  _connected = false;
  _connectionChanged = false;
  myGlobal = this;

#if defined(ALARMS_SUPPORT) || defined(SDLOGGEDATAGRAPH_SUPPORT)
  _rtc = new RTClock();
#endif
#ifdef ALARMS_SUPPORT
  _processAlarms = NULL;
#endif
}

#if defined(ALARMS_SUPPORT)

static bool checkAlarmsNow = false;  // true if it's time to check alarms

AMController::AMController(
  void (*doWork)(void),
  void (*doSync)(),
  void (*processIncomingMessages)(char *variable, char *value),
  void (*processOutgoingMessages)(void),
  void (*processAlarms)(char *alarm),
  void (*deviceConnected)(void),
  void (*deviceDisconnected)(void))
  : AMController(doWork, doSync, processIncomingMessages, processOutgoingMessages, deviceConnected, deviceDisconnected) {

  _processAlarms = processAlarms;
  myGlobal = this;
}
#endif

void AMController::begin() {

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting Bluetooth® Low Energy module failed!");

    while (1)
      ;
  }

  // set the local name peripheral advertises
  BLE.setLocalName("AManager");
  BLE.setDeviceName("AManager");
  BLE.setAdvertisedService(mainService);

  // add the characteristic to the service
  mainService.addCharacteristic(rxCharacteristic);
  mainService.addCharacteristic(txCharacteristic);

  BLE.addService(mainService);

  batteryService.addCharacteristic(batteryLevelCharacteristic);
  BLE.addService(batteryService);

  // assign event handlers for connected, disconnected to peripheral
  BLE.setEventHandler(BLEConnected, connectHandler);
  BLE.setEventHandler(BLEDisconnected, disconnectHandler);
  rxCharacteristic.setEventHandler(BLEWritten, characteristicWritten);

  // start advertising
  BLE.advertise();

#ifdef ALARMS_SUPPORT

  if (_processAlarms != NULL) {
    initializeAlarms();

    _rtc->begin();
    // RTC must be set at time > 0, otherwise periodic callback doesn't start
    // Great job Arduino!!
    //
    // 1/1/2000 : 12:00:00 AM GMT
    //
    RTCTime startTime(946684800);
    _rtc->setTime(startTime);
    _rtc->setPeriodicCallback(enableCheckAlarms, Period::ONCE_EVERY_2_SEC);
  }
#endif

#ifdef SDLOGGEDATAGRAPH_SUPPORT
  _rtc->begin();
  // RTC must be set at time > 0, otherwise periodic callback doesn't start
  // Great job Arduino!!
  //
  // 1/1/2000 : 12:00:00 AM GMT
  //
  RTCTime startTime(946684800);
  _rtc->setTime(startTime);
#endif

  Serial.println(("Bluetooth® device active, waiting for connections..."));
}

void AMController::loop() {
  this->loop(0);
}

void AMController::loop(unsigned long _delay) {

  BLE.poll();

  if (_connectionChanged) {
    _connectionChanged = false;
    if (_connected) {
      BLE.poll();
      if (_deviceConnected != NULL)
        _deviceConnected();
    } else {
      BLE.poll();
      if (_deviceDisconnected != NULL)
        _deviceDisconnected();
    }
  }

  BLE.poll();
  if (_dataAvailable) {
    _dataAvailable = false;
    processIncomingData();
  }

  if (_sync) {
    _sync = false;
    BLE.poll();
    _doSync();
    BLE.poll();
  }

  BLE.poll();
  _doWork();

  if (_connected) {
    BLE.poll();
    _processOutgoingMessages();
  }

  BLE.poll();
#ifdef ALARMS_SUPPORT
  // CheckAlarms
  if (checkAlarmsNow) {
    checkAndFireAlarms();
  }
#endif

  delay(_delay);
}


void AMController::processIncomingData() {

  char _variable[VARIABLELEN + 1];
  char _value[VALUELEN + 1];
  bool _var = true;
  uint8_t _idx = 0;
  int lastPound = -1;

  _variable[0] = '\0';
  _value[0] = '\0';

  uint8_t l = strlen(_remainBuffer);

  //Serial.print("Full buffer before >"); Serial.print(_remainBuffer); Serial.println("<");

  for (uint8_t i = 0; i < l; i++) {

    BLE.poll();

    if (_var) {
      if (_remainBuffer[i] != '=') {
        _variable[_idx++] = _remainBuffer[i];
      } else {
        _variable[_idx] = '\0';
        _var = false;
        _idx = 0;
      }
    } else {
      if (_remainBuffer[i] == '#') {
        lastPound = i;
      }
      if (_remainBuffer[i] != '#') {
        _value[_idx++] = _remainBuffer[i];
      } else {
        _value[_idx] = '\0';
        _var = true;
        _idx = 0;

        if (strlen(_value) > 0 && strcmp(_variable, "Sync") == 0) {
          _sync = true;
        } else
#if defined(ALARMS_SUPPORT) || defined(SDLOGGEDATAGRAPH_SUPPORT)
          if (strcmp(_variable, "$Time$") == 0) {
          unsigned long unixTime = atol(_value);
          RTCTime timeToSet = RTCTime(unixTime);
          PRINTMSG("Setting current time at:", timeToSet.toString());
          _rtc->setTime(timeToSet);
        } else
#endif
#ifdef ALARMS_SUPPORT
          if (strlen(_value) > 0 && (strcmp(_variable, "$AlarmId$") == 0 || strcmp(_variable, "$AlarmT$") == 0 || strcmp(_variable, "$AlarmR$") == 0)) {
          manageAlarms(_variable, _value);
        } else
#endif
#ifdef SD_SUPPORT
          if (strlen(_variable) > 0 && (strcmp(_variable, "SD") == 0 || strcmp(_variable, "$SDDL$") == 0)) {
          manageSD(_variable, _value);
        } else
#endif
          if (strlen(_variable) > 0 && strlen(_value) > 0) {
#ifdef SDLOGGEDATAGRAPH_SUPPORT
          if (strlen(_variable) > 0 && strcmp(_variable, "$SDLogData$") == 0) {
            Serial.print("Logged data request for: ");
            Serial.println(_value);
            sdSendLogData(_value);
          } else
#endif
          {
            // Process incoming messages
#ifdef DEBUG
            Serial.print("process ");
            Serial.print(_variable);
            Serial.print(" -> ");
            Serial.println(_value);
#endif
            _processIncomingMessages(_variable, _value);
          }
        }
      }
    }
  }

  if (lastPound == l - 1) {
    _remainBuffer[0] = '\0';
  } else if (lastPound > 0) {
    char tmp[128];
    strcpy(tmp, &_remainBuffer[lastPound + 1]);
    strcpy(_remainBuffer, tmp);
  }

#ifdef DEBUG
  Serial.print("Full buffer after  >");
  Serial.print(_remainBuffer);
  Serial.println("<");
#endif
}

void AMController::writeMessage(const char *variable, int value) {
  char buffer[128];

  if (!_connected) {
    return;
  }
  memset(&buffer, 0, 128);
  snprintf(buffer, 128, "%s=%d#", variable, value);
  writeBuffer((uint8_t *)&buffer, strlen(buffer));
  BLE.poll();
  delay(WRITE_DELAY);
}

void AMController::writeMessage(const char *variable, float value) {
  char buffer[128];

  if (!_connected) {
    return;
  }
  memset(&buffer, 0, 128);
  snprintf(buffer, 128, "%s=%.3f#", variable, value);
  writeBuffer((uint8_t *)&buffer, strlen(buffer));
  BLE.poll();
  delay(WRITE_DELAY);
}

void AMController::writeTripleMessage(const char *variable, float vX, float vY, float vZ) {
  char buffer[VARIABLELEN + VALUELEN + 3];

  if (!_connected) {
    return;
  }
  snprintf(buffer, VARIABLELEN + VALUELEN + 3, "%s=%.2f:%.2f:%.2f#", variable, vX, vY, vZ);
  writeBuffer((uint8_t *)&buffer, strlen(buffer) * sizeof(char));
  BLE.poll();
  delay(WRITE_DELAY);
}

void AMController::writeTxtMessage(const char *variable, const char *value) {
  char buffer[128];

  if (!_connected) {
    return;
  }
  memset(&buffer, 0, 128);
  snprintf(buffer, 128, "%s=%s#", variable, value);
  writeBuffer((uint8_t *)&buffer, strlen(buffer));
  BLE.poll();
  delay(WRITE_DELAY);
}


/**
	Can send a buffer longer than 20 bytes
**/
void AMController::writeBuffer(uint8_t *buffer, int l) {
  uint8_t buffer1[22];

  if (!_connected) {
    return;
  }

  uint8_t idx = 0;

  while (idx < l) {

    uint8_t this_block_size = min(20, l - idx);
    memset(&buffer1, '\0', 22);
    memcpy(&buffer1, buffer + idx, this_block_size);

    //PRINT("Sending >"); PRINT((char *)buffer1); PRINT("<"); PRINTLN();

    txCharacteristic.writeValue((uint8_t *)&buffer1, 20);
    delay(WRITE_DELAY);
    BLE.poll();

    idx += this_block_size;
  }
}

void AMController::updateBatteryLevel(uint8_t level) {
  if (!_connected) {
    return;
  }

  batteryLevelCharacteristic.writeValue(level);
  delay(WRITE_DELAY);
}

void AMController::log(const char *msg) {
  this->writeTxtMessage("$D$", msg);
}

void AMController::log(int msg) {

  char buffer[11];
  itoa(msg, buffer, 10);

  this->writeTxtMessage("$D$", buffer);
}

void AMController::logLn(const char *msg) {
  this->writeTxtMessage("$DLN$", msg);
}

void AMController::logLn(int msg) {
  char buffer[11];
  itoa(msg, buffer, 10);

  this->writeTxtMessage("$DLN$", buffer);
}

void AMController::logLn(long msg) {
  char buffer[11];
  ltoa(msg, buffer, 10);

  this->writeTxtMessage("$DLN$", buffer);
}

void AMController::logLn(unsigned long msg) {
  char buffer[11];
  ltoa(msg, buffer, 10);

  this->writeTxtMessage("$DLN$", buffer);
}

void AMController::temporaryDigitalWrite(uint8_t pin, uint8_t value, unsigned long ms) {
  boolean previousValue = digitalRead(pin);

  digitalWrite(pin, value);
  delay(ms);
  digitalWrite(pin, previousValue);
}

#if defined(ALARMS_SUPPORT) || defined(SDLOGGEDATAGRAPH_SUPPORT)

unsigned long AMController::now() {
  RTCTime currentTime;
  _rtc->getTime(currentTime);
  return currentTime.getUnixTime();
}

#endif

#ifdef ALARMS_SUPPORT

#ifdef DEBUG
void AMController::printTime(unsigned long time) {
  RTCTime currentTime = RTCTime(time);
  PRINTLN(String(currentTime));
}
#endif

void AMController::initializeAlarms() {

  PRINTLN("Initialize Alarms");

  for (int i = 0; i < MAX_ALARMS; i++) {
    alarm a;

    EEPROM.get(i * sizeof(a), a);
    if (a.id[0] != 'A') {
      a.id[0] = 'A';
      a.id[1] = '\0';
      a.time = 0;
      a.repeat = 0;
      EEPROM.put(i * sizeof(a), a);
    }
  }
#ifdef DEBUG
  dumpAlarms();
#endif
}

void AMController::manageAlarms(char *variable, char *value) {
  PRINT("Manage Alarm variable: ");
  PRINT(variable);
  PRINT(" value: ");
  PRINTLN(value);

  if (strcmp(variable, "$AlarmId$") == 0) {
    strcpy(_alarmId, value);
  } else if (strcmp(variable, "$AlarmT$") == 0) {
    _alarmTime = atol(value);
  } else if (strcmp(variable, "$AlarmR$") == 0) {
    if (_alarmTime == 0) {
      PRINTMSG("Deleting Alarm ", _alarmId);
      BLE.poll();
      removeAlarm(_alarmId);
    } else {
      PRINTMSG("Adding/Updating Alarm ", _alarmId);
      BLE.poll();
      createUpdateAlarm(_alarmId, _alarmTime, atoi(value));
    }
#ifdef DEBUG
    dumpAlarms();
#endif
  }
}

void AMController::createUpdateAlarm(char *id, unsigned long time, bool repeat) {
  char lid[12];
  lid[0] = 'A';
  strcpy(&lid[1], id);

  // Update
  for (int i = 0; i < MAX_ALARMS; i++) {
    alarm a;
    EEPROM.get(i * sizeof(a), a);
    if (strcmp(a.id, lid) == 0) {
      a.time = time;
      a.repeat = repeat;
      EEPROM.put(i * sizeof(a), a);
      return;
    }
  }

  // Create
  for (int i = 0; i < MAX_ALARMS; i++) {
    alarm a;
    EEPROM.get(i * sizeof(a), a);
    if (a.id[1] == '\0') {
      strcpy(a.id, lid);
      a.time = time;
      a.repeat = repeat;
      EEPROM.put(i * sizeof(a), a);
      return;
    }
  }

#ifdef DEBUG
  dumpAlarms();
#endif
}


void AMController::removeAlarm(char *id) {
  char lid[12];
  lid[0] = 'A';
  strcpy(&lid[1], id);

  for (int i = 0; i < MAX_ALARMS; i++) {
    alarm a;
    EEPROM.get(i * sizeof(a), a);
    if (strcmp(a.id, lid) == 0) {
      a.id[1] = '\0';
      a.time = 0;
      a.repeat = 0;
      EEPROM.put(i * sizeof(a), a);
    }
  }
}


#ifdef DEBUG
void AMController::dumpAlarms() {

  BLE.poll();

  Serial.println("\t----Current Alarms -----");

  for (int i = 0; i < MAX_ALARMS; i++) {
    alarm al;
    EEPROM.get(i * sizeof(al), al);

    Serial.print("\t");
    Serial.print(al.id);
    Serial.print(" ");
    RTCTime currentTime = RTCTime(al.time);
    PRINT(String(currentTime));
    Serial.print(" ");
    Serial.println(al.repeat);
  }
}
#endif

void AMController::enableCheckAlarms() {
  checkAlarmsNow = true;
}

void AMController::checkAndFireAlarms() {

  checkAlarmsNow = false;

  unsigned long currentUnixTime = this->now();

#ifdef DEBUG_ALARMS
  RTCTime currentTime(currentUnixTime);
  PRINTMSG2("checkAndFireAlarms @", currentTime.toString(), currentUnixTime);
  dumpAlarms();
#endif

  for (int i = 0; i < MAX_ALARMS; i++) {
    alarm a;

    EEPROM.get(i * sizeof(a), a);
    if (a.id[1] != '\0' && a.time < currentUnixTime) {
      PRINTLN(a.id);
      // First character of id is A and has to be removed
      _processAlarms(&a.id[1]);
      if (a.repeat) {
        a.time += 86400;  // Scheduled again tomorrow
#ifdef DEBUG_ALARMS
        currentTime.setUnixTime(a.time);
        PRINTMSG("larm rescheduled @", currentTime.toString());
#endif
      } else {
        //     Alarm removed
        a.id[1] = '\0';
        a.time = 0;
        a.repeat = 0;
      }

      EEPROM.put(i * sizeof(a), a);
#ifdef DEBUG_ALARMS
      this->dumpAlarms();
#endif
    }
  }
}

#endif

#ifdef SDLOGGEDATAGRAPH_SUPPORT

void AMController::sdLogLabels(const char *variable, const char *label1) {
  this->sdLogLabels(variable, label1, NULL, NULL, NULL, NULL);
}

void AMController::sdLogLabels(const char *variable, const char *label1, const char *label2) {
  this->sdLogLabels(variable, label1, label2, NULL, NULL, NULL);
}

void AMController::sdLogLabels(const char *variable, const char *label1, const char *label2, const char *label3) {
  this->sdLogLabels(variable, label1, label2, label3, NULL, NULL);
}

void AMController::sdLogLabels(const char *variable, const char *label1, const char *label2, const char *label3, const char *label4) {

  this->sdLogLabels(variable, label1, label2, label3, label4, NULL);
}

void AMController::sdLogLabels(const char *variable, const char *label1, const char *label2, const char *label3, const char *label4, const char *label5) {

  File dataFile = SD.open(variable, FILE_WRITE);

  if (dataFile) {
    dataFile.print("-");
    dataFile.print(";");
    dataFile.print(label1);
    dataFile.print(";");

    if (label2 != NULL)
      dataFile.print(label2);
    else
      dataFile.print("-");
    dataFile.print(";");

    if (label3 != NULL)
      dataFile.print(label3);
    else
      dataFile.print("-");
    dataFile.print(";");

    if (label4 != NULL)
      dataFile.print(label4);
    else
      dataFile.print("-");
    dataFile.print(";");

    if (label5 != NULL)
      dataFile.println(label5);
    else
      dataFile.println("-");

    dataFile.flush();
    dataFile.close();
  } else {
    PRINTMSG("Error opening", variable);
  }
}


void AMController::sdLog(const char *variable, unsigned long time, float v1) {
  File dataFile = SD.open(variable, FILE_WRITE);

  if (dataFile && (time > 946684800)) {
    dataFile.print(time);
    dataFile.print(";");
    dataFile.print(v1);

    dataFile.print(";-;-;-;-");
    dataFile.println();

    dataFile.flush();
    dataFile.close();
  } else {
    PRINTMSG("Error opening", variable);
  }
}

void AMController::sdLog(const char *variable, unsigned long time, float v1, float v2) {

  File dataFile = SD.open(variable, FILE_WRITE);

  if (dataFile && (time > 946684800)) {
    dataFile.print(time);
    dataFile.print(";");
    dataFile.print(v1);
    dataFile.print(";");

    dataFile.print(v2);

    dataFile.print(";-;-;-");
    dataFile.println();

    dataFile.flush();
    dataFile.close();
  } else {
    PRINTMSG("Error opening", variable);
  }
}

void AMController::sdLog(const char *variable, unsigned long time, float v1, float v2, float v3) {
  File dataFile = SD.open(variable, FILE_WRITE);

  if (dataFile && (time > 946684800)) {
    dataFile.print(time);
    dataFile.print(";");
    dataFile.print(v1);
    dataFile.print(";");

    dataFile.print(v2);
    dataFile.print(";");

    dataFile.print(v3);

    dataFile.print(";-;-");
    dataFile.println();

    dataFile.flush();
    dataFile.close();
  } else {
    PRINTMSG("Error opening", variable);
  }
}

void AMController::sdLog(const char *variable, unsigned long time, float v1, float v2, float v3, float v4) {
  File dataFile = SD.open(variable, FILE_WRITE);

  if (dataFile && (time > 946684800)) {
    dataFile.print(time);
    dataFile.print(";");
    dataFile.print(v1);
    dataFile.print(";");

    dataFile.print(v2);
    dataFile.print(";");

    dataFile.print(v3);
    dataFile.print(";");

    dataFile.print(v4);

    dataFile.println(";-");
    dataFile.println();

    dataFile.flush();
    dataFile.close();
  } else {
    PRINTMSG("Error opening", variable);
  }
}

void AMController::sdLog(const char *variable, unsigned long time, float v1, float v2, float v3, float v4, float v5) {

  File dataFile = SD.open(variable, FILE_WRITE);

  if (dataFile && (time > 946684800)) {
    dataFile.print(time);
    dataFile.print(";");
    dataFile.print(v1);
    dataFile.print(";");

    dataFile.print(v2);
    dataFile.print(";");

    dataFile.print(v3);
    dataFile.print(";");

    dataFile.print(v4);
    dataFile.print(";");

    dataFile.println(v5);

    dataFile.println();

    dataFile.flush();
    dataFile.close();
  } else {
    PRINTMSG("Error opening", variable);
  }
}

void AMController::sdSendLogData(const char *variable) {
  char fileNameBuffer[VARIABLELEN + 1];

  strcpy(fileNameBuffer, "/");
  strcat(fileNameBuffer, variable);

  File dataFile = SD.open(fileNameBuffer);

  if (dataFile) {

    char c;
    char buffer[128];
    int i = 0;

    dataFile.seek(0);

    while (dataFile.available()) {

      c = dataFile.read();

      if (c == '\n') {

        buffer[i++] = '\0';
        PRINTLN(buffer);
        this->writeTxtMessage(variable, buffer);

        i = 0;
      } else
        buffer[i++] = c;
    }

    PRINTLN("All data sent");

    dataFile.close();
  } else {
    PRINTMSG("Error opening", variable);
  }

  this->writeTxtMessage(variable, "");
}

// Size in Kbytes
uint16_t AMController::sdFileSize(const char *variable) {

  File dataFile = SD.open(variable, FILE_READ);

  if (dataFile) {
    return max(1, dataFile.size() / 1024);
  }

  return 0;
}

void AMController::sdPurgeLogData(const char *variable) {
  char fileNameBuffer[VARIABLELEN + 1];

  strcpy(fileNameBuffer, "/");
  strcat(fileNameBuffer, variable);
  SD.remove(fileNameBuffer);
}

#endif

////////////////////////////////////////////////////

void AMController::connected(void) {
  _connected = true;
  if (_deviceConnected != NULL)
    _deviceConnected();
}

void AMController::disconnected(void) {
  _connected = false;
  _remainBuffer[0] = '\0';
  if (_deviceDisconnected != NULL)
    _deviceDisconnected();
}


void AMController::dataAvailable(String data) {
  strcat(_remainBuffer, data.c_str());
  _dataAvailable = true;
}

#ifdef SD_SUPPORT

void AMController::manageSD(char *variable, char *value) {
  PRINTLN("Manage SD");

  if (strcmp(variable, "SD") == 0) {
    PRINTLN("\t[File List Start]");

    File dir = SD.open("/");
    if (!dir) {
      PRINTLN("Failed to open /");
      return;
    }
    dir.rewindDirectory();
    File entry = dir.openNextFile();
    if (!entry) {
      PRINTLN("\tFailed to get first file");
      return;
    }

    while (entry) {
      if (!entry.isDirectory()) {
        this->writeTxtMessage("SD", entry.name());
        PRINTLN("\t" + String(entry.name()));
      }
      entry.close();
      entry = dir.openNextFile();
    }

    dir.close();

    this->writeTxtMessage("SD", "$EFL$");
    PRINTLN("\t[File List End]");
  }
  if (strcmp(variable, "$SDDL$") == 0) {
    PRINTMSG("Sending File: ", value);

    File dataFile = SD.open(value, FILE_READ);
    if (dataFile) {
      unsigned long n = 0;
      uint8_t buffer[64];
      strcpy((char *)&buffer[0], "SD=$C$#");
      this->writeBuffer(buffer, 7 * sizeof(uint8_t));

      delay(500);  // OK

      while (dataFile.available()) {
        n = dataFile.read(buffer, sizeof(buffer));
        this->writeBuffer(buffer, n * sizeof(uint8_t));
      }
      strcpy((char *)&buffer[0], "SD=$E$#");
      this->writeBuffer(buffer, 7 * sizeof(uint8_t));
      delay(150);
      dataFile.close();

      PRINTLN("File sent");
    }
  }
}
#endif

////////////////////

void connectHandler(BLEDevice central) {
  // central connected event handler
  myGlobal->connected();
  PRINT("\tConnected event, central: ");
  PRINTLN(central.address());
}

void disconnectHandler(BLEDevice central) {
  // central disconnected event handler
  myGlobal->disconnected();
  PRINT("\Disconnected event, central: ");
  PRINTLN(central.address());
}

void characteristicWritten(BLEDevice central, BLECharacteristic characteristic) {

  //PRINTLN("Characteristic event, written: ");

  char buffer[40];
  int n = characteristic.readValue(buffer, sizeof(buffer));
  memset(&buffer[n], 0, sizeof(buffer) - n);
  String d = String(buffer);

  PRINT("R >");
  PRINT(d);
  PRINT("< ");
  PRINTLN(n);

  myGlobal->dataAvailable(d);
}
