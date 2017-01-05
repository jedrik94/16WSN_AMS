#include <Arduino.h>

#include <SPI.h>
#include <RF24.h>
#include <printf.h>
#include <nRF24L01.h>
#include <AESLib.h>
#include "definitionsFile.h"

uint8_t sentFramesCounter = 0x00;

RF24 radio(9,10);

const uint64_t adr[2] = { 0x1B1B1B1B21LL, 0x1B1B1B1B12LL };

const uint8_t keyAES[] = { 0x00, 0x01, 0x02, 0x03,
                           0x04, 0x05, 0x06, 0x07,
                           0x08, 0x09, 0x0A, 0x0B,
                           0x0C, 0x0D, 0x0E, 0x0F};

                           // char data[] = "Jedrzej Wojtkowiak";
                           // aes128_enc_single(key, data);
                           // aes128_dec_single(key, data);

uint8_t message[] = {"SlaveGalaWojtkow"};

const uint8_t emptyArray[] = { 0x00, 0x01, 0x02, 0x03,
                               0x04, 0x05, 0x06, 0x07,
                               0x08, 0x09, 0x0A, 0x0B,
                               0x0C, 0x0D, 0x0E, 0x0F,
                               0x10, 0x11, 0x12};

uint8_t transmissionType = NON;

const long timeToStart = 1000;
const long timeToNextSTM = 49000;

long slaveStartSendingTime = 0;
long slaveStartNextSTM = 0;

bool isMaster = false;

static const byte crc_gen = 0x39;

static int errorCounter = 0;

long startTime = 0;

bool noSecure = false, modeCRC = false, modeCRC_AES = false;

static int slavesNumber = 0;
uint8_t* availableSlavesAddresses = new uint8_t[15];

uint8_t* dataToSend = new uint8_t[FRAMELENGTH];
uint8_t* dataReceived = new uint8_t[FRAMELENGTH];
uint8_t* onlyMessage = new uint8_t[17];

void radioSetUp() {
   radio.begin();

   radio.setPALevel(RF24_PA_MIN);
   radio.setDataRate(RF24_1MBPS);

   radio.setPayloadSize(FRAMELENGTH);

   radio.setChannel(16);

   if(isMaster) {
     Serial.println("---Master---");
     radio.openWritingPipe(adr[0]);
     radio.openReadingPipe(1, adr[1]);
   } else if (!isMaster) {
     Serial.println("---Slave---");
     radio.openWritingPipe(adr[1]);
     radio.openReadingPipe(1, adr[0]);
   }

   radio.setAutoAck(false);
   radio.setRetries(0,0);

   radio.disableCRC();

   radio.printDetails();
}

void printHex(uint8_t* data, int length) {
    for (int i = 0; i < length; i++) {
      Serial.print("0x");
      if(data[i] < 0x10)
        Serial.print("0");
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
}

uint8_t CRC8(uint8_t dataBytes[], unsigned short usDataLen) {

  uint8_t crc = 0x00;

  while (usDataLen--) {
    uint8_t inByte = *dataBytes++;
    crc ^= inByte;
    for (int i = 8; i > 0; i--) {
      if (crc & 0x80)
        crc = (crc << 1) ^ crc_gen;
      else
        crc <<= 1;
    }
  }
  return crc;
}

inline void assigningIO() {
  pinMode(LED_PIN, OUTPUT);

  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(BUTTON3, INPUT);
}

inline void defineMode() {
  if (digitalRead(BUTTON1)) {
    isMaster = true;
  } else if(digitalRead(BUTTON2)) {
    isMaster = false;
  }
}

void selectionSignal() {
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
}

uint8_t* convIntToUint8(long intParase) {
  uint8_t* bytesArray = new uint8_t[4];

  for (int i = 0; i < 4; i++) {
      bytesArray[3 - i] = (intParase >> (i * 8)) & 0xFF;
  }
  return bytesArray;
}

long convUint8ToInt(uint8_t* uintParase) {
  return ((uint32_t)uintParase[0] << 24) | ((uint32_t)uintParase[1] << 16) | ((uint32_t)uintParase[2] << 8) | (uint32_t)uintParase[3];
}

void createFrame(uint8_t RN, uint8_t FNC, uint8_t* data) {
  dataToSend[0] = NN;
  dataToSend[1] = RN;
  dataToSend[2] = FNC;
  if(FNC == MES || FNC == CNF)
  {
    dataToSend[3] = sentFramesCounter++;
    for(int i = 0; i <= 16; i++) {
      dataToSend[i + 4] = *(data + i);
    }
  } else {
    for(int i = 3; i <= 20; i++){
      dataToSend[i] = 0x55;
    }
  }
}

// Sending errors is turned off!
void errorHandler(uint8_t address) {
  radio.stopListening();

  createFrame(address, ERR, 0x00);

  // radio.write(dataToSend, FRAMELENGTH);

  errorCounter++;

  Serial.print("Count of errors: ");
  Serial.println(errorCounter);
}

bool getACKMaster() {
  if(*(dataReceived + 2) == ACK) {
    Serial.print("ACK received from: ");
    Serial.print(*(dataReceived + 1), HEX);
    Serial.println();
    return true;
  } else {
    errorHandler(RN_0);
    return false;
  }
}

bool getACKSlave() {
  if(*(dataReceived + 2) == ACK) {
    Serial.print("ACK received from Master");
    Serial.println();
    return true;
  } else {
    errorHandler(*(dataReceived + 1));
    return false;
  }
}

bool getACK(uint8_t address) {
  bool timeout = false;
  startTime = millis();

  radio.startListening();

  while(!radio.available()) {
    if((millis() - startTime) > TIMESLOT) {
      Serial.println();
      Serial.println("Here");
      Serial.println();
      timeout = true;
      break;
    }
  }

  if(timeout) {
    if (isMaster) {
      Serial.println("Timeout of connection with Node");
      errorHandler(RN_0);
    } else if (!isMaster) {
      Serial.println("Timeout of connection with Master");
      errorHandler(address);
    }
    return false;
  }

  radio.read(dataReceived, FRAMELENGTH);
  radio.stopListening();

  if (isMaster) {
    return getACKMaster();
  } else if (!isMaster) {
    return getACKSlave();
  }

  return false;
}

void presenceTest(uint8_t address, uint8_t transmissionMode) {

  radio.stopListening();

  createFrame(address, transmissionMode, 0x00);
  delay(500);
  radio.write(dataToSend, FRAMELENGTH);

  // Debugging
  Serial.print("---Debugging--- ");
  printHex(dataToSend, FRAMELENGTH);
  Serial.print("---EndOfDebugging---");
  Serial.println();

  radio.startListening();

    if(getACK(address)) {
      availableSlavesAddresses[slavesNumber] = *(dataReceived + 1);
      Serial.print("ACK recieved from Node. Address: ");
      Serial.println(*(dataReceived + 1), HEX);
      Serial.print("Current number of slaves: ");
      Serial.println(slavesNumber + 1);
      slavesNumber++;
    } else {
      Serial.print("ACK not recieved from Node. There is no such a Node like: ");
      Serial.println(address, HEX);
      Serial.println("\n");
    }

    radio.stopListening();

}

void setSlaveTimeSlot(uint8_t address, uint8_t transmissionMode) {
  uint8_t* slotTimeData = new uint8_t[4];
  uint8_t* nextSTM = new uint8_t[4];
  uint8_t* dataPrepared = new uint8_t[17];

  long slotTime = 0;
  long slotTimeBetweenSTM = 0;

  slotTimeBetweenSTM = timeToStart + timeToNextSTM;
  nextSTM = convIntToUint8(slotTimeBetweenSTM);

  radio.stopListening();

  for(int i = 0; i < slavesNumber; i++) {
    slotTime = timeToStart + i * TIMESLOT;

    slotTimeData = convIntToUint8(slotTime);

    for(int j = 0; j <= 8; j++) {
      if(j < 4) {
        dataPrepared[j] = slotTimeData[j];
        dataPrepared[j + 4] = nextSTM[j];
      }
      dataPrepared[j + 8] = 0x00;
    }
    if(transmissionMode == NON) {
      dataPrepared[16] = 0x00;
    } else if(transmissionMode == CRC) {
      dataPrepared[16] = CRC8(dataPrepared, 16);
    } else if(transmissionMode == CRC_AES) {
      /*
      * AES is cripting message with CRC8
      */
      dataPrepared[16] = CRC8(dataPrepared, 16);
      aes128_enc_single(keyAES, dataPrepared);
    }

    Serial.println("$$$$$$$$$$$$");
    printHex(nextSTM, 4);
    Serial.println("\n$$$$$$$$$$$$");
    printHex(slotTimeData, 4);
    Serial.println("\n$$$$$$$$$$$$");

    delay(1000);

    createFrame(address, CNF, dataPrepared);

    radio.write(dataToSend, FRAMELENGTH);
  }
}

void sendACK(uint8_t address) {
  radio.stopListening();
  createFrame(address, ACK, 0x00);
  radio.write(dataToSend, FRAMELENGTH);
  Serial.println("ACK sent");
}

void getTime() {
  uint8_t* tempArray = new uint8_t[4];

  for(int i = 0; i < 4; i++) {
    tempArray[i] = *(onlyMessage + i);
  }
  slaveStartSendingTime = convUint8ToInt(tempArray);

  for(int i = 0; i < 4; i++) {
    tempArray[i] = *(onlyMessage + i + 4);
  }
  slaveStartNextSTM = convUint8ToInt(tempArray);

  Serial.println("@@@@@@@@@@@@@@@");
  Serial.println(slaveStartSendingTime);
  Serial.println("@@@@@@@@@@@@@@@");
  Serial.println(slaveStartNextSTM);
  Serial.println("@@@@@@@@@@@@@@@");
}

uint8_t* getOnlyMessage() {
  uint8_t* tempMessage = new uint8_t [17];

  for(int i = 0; i <= 16; i++) {
    tempMessage[i] = *(dataReceived + i + 4);
  }

  return tempMessage;
}

bool confirmCRC() {
  uint8_t* tempArray = new uint8_t[16];

  for(int i = 0; i < 16; i++) {
    tempArray[i] = *(dataReceived + i + 4);
  }

  if(*(dataReceived + 20) == CRC8(tempArray, 16)) {
    return true;
  } else {
    return false;
  }
}

uint8_t* fillMessageBuffer(uint8_t* array) {
  for(int i = 0; i < 16; i++) {
    array[i] = message[i];
  }

  return array;
}

void foo(uint8_t transmissionMode) {
  long tempTime = millis();

  uint8_t* tempArray = new uint8_t[17];

  tempArray = fillMessageBuffer(tempArray);

  if(transmissionMode == NON) {
    tempArray[16] = 0x00;
  } else if(transmissionMode == CRC) {
    tempArray[16] = CRC8(tempArray, 16);
  } else if(transmissionMode == CRC_AES) {
    /*
    * AES is cripting message with CRC8
    */
    tempArray[16] = CRC8(tempArray, 16);
    aes128_enc_single(keyAES, tempArray);
  }

  createFrame(RN_S, MES, tempArray);

  delay(timeToStart - (millis() - tempTime));

  while(true) {
    delay(slaveStartSendingTime - timeToStart);

    Serial.println();
    printHex(message, 16);
    Serial.println();
    printHex(dataToSend, 21);
    Serial.println();

    radio.write(dataToSend, FRAMELENGTH);

    delay(timeToNextSTM + slaveStartNextSTM);
  }
}

void receiveData(uint8_t address) {
  radio.startListening();

  startTime = millis();

  while (!radio.available() ) {
    if ((millis() - startTime) > TIMESLOT ) {
      Serial.println("There is no radio to listen to!");
      break;
    }
  }

  radio.read(dataReceived, FRAMELENGTH);

  // Debugging
  Serial.print("---Debugging--- ");
  printHex(dataReceived, FRAMELENGTH);
  Serial.print("---EndOfDebugging---");
  Serial.println();

  if(*(dataReceived + 1) == RN_S || *(dataReceived + 1) == RN_0) {
    switch (*(dataReceived + 2)) {
      case NON:
        transmissionType = NON;
        delay(50);
        sendACK(RN_S);
        break;
      case CRC:
        transmissionType = CRC;
        delay(50);
        sendACK(RN_S);
        break;
      case CRC_AES:
        transmissionType = CRC_AES;
        delay(50);
        sendACK(RN_S);
        break;
      case STM:
        radio.stopListening();

        foo(transmissionType);
        break;
      case CNF:
        onlyMessage = getOnlyMessage();
        if(transmissionType == CRC_AES) {
          aes128_dec_single(keyAES, onlyMessage);
        } else if (transmissionType == CRC_AES || transmissionType == CRC) {
          if(confirmCRC()) {
            Serial.println("Data was sent properly. CRC8 confirmed!");
          } else {
            Serial.println("Data was NOT sent properly. CRC8 NOT confirmed!");
            errorHandler(RN_S);
          }
        }
        getTime();
        break;
      default:
        break;
    }
  } else {
    // Debugging
    Serial.println("Nothing to read!");
  }
  radio.stopListening();
}

void getMessageFromSlave(uint8_t transmissionMode) {
  startTime = millis();

  radio.startListening();

  while (!radio.available()) {
    if ((millis() - startTime) > TIMESLOT) {
      Serial.println("There is no radio to listen to!");
      return;
    }
  }

  radio.read(dataReceived, FRAMELENGTH);

  if(*(dataReceived + 2) != MES) {
      Serial.println("Unexpected type of data!");
      errorHandler(*(dataReceived + 1));
      return;
  }

  Serial.print("Message received from Node: ");
  Serial.println(*(dataReceived + 1), HEX);

  Serial.print("Transmission parameter: ");
  switch (transmissionMode) {
    case NON:
      Serial.println("no security");
      break;
    case CRC:
      Serial.println("CRC8");
      break;
    case CRC_AES:
      Serial.println("CRC8 with AES128");
      break;
  }

  onlyMessage = getOnlyMessage();

  if(transmissionType == CRC_AES) {
    aes128_dec_single(keyAES, onlyMessage);
  } else if (transmissionType == CRC_AES || transmissionType == CRC) {
    if(confirmCRC()) {
      Serial.println("Data was sent properly. CRC8 confirmed!");
    } else {
      Serial.println("Data was NOT sent properly. CRC8 NOT confirmed!");
      errorHandler(RN_S);
    }
  }

  for(int i = 0; i < 16; i++) {
    Serial.print((char)* (onlyMessage + i));
  }

  Serial.print("\n");

  radio.stopListening();

  Serial.print("End of transmission from Node: ");
  Serial.println(*(dataReceived + 1), HEX);
}

void slaveMode() {
  receiveData(RN_S);
}

void masterMode(uint8_t transmissionMode) {
  slavesNumber = 0;

  radio.stopListening();

  Serial.println();
  Serial.println("--------------------------");
  Serial.println("Looking for Nodes");
  Serial.println("--------------------------");
  Serial.println();


  for(uint8_t i = 0x01; i <= 15; i++) {
    presenceTest(i, transmissionMode);
  }

  if(slavesNumber == 0) {
    Serial.println("There is no Nodes to communicate with.");
    return;
  } else {
    for(int i = 0; i < slavesNumber; i++){
      setSlaveTimeSlot(availableSlavesAddresses[i], transmissionMode);
    }
  }

  createFrame(0x00, STM, 0x00);
  radio.write(dataToSend, FRAMELENGTH);

  delay(timeToStart);



  while(true) {
    startTime = millis();
    for(int i = 0; i < slavesNumber; i++) {
      getMessageFromSlave(transmissionMode);

      while(true) {
        if ((millis() - startTime) > (unsigned long)(TIMESLOT * (i + 1)) ) {
        break;
        }
      }
    }
    delay(timeToNextSTM);
  }
}

void masterStart() {
  int errorClickCounter = 0;
  while(true) {
    do {
      noSecure = digitalRead(BUTTON1);
      modeCRC = digitalRead(BUTTON2);
      modeCRC_AES = digitalRead(BUTTON3);
    } while(!noSecure && !modeCRC && !modeCRC_AES);

    if((noSecure && modeCRC) || (noSecure && modeCRC_AES) || (modeCRC && modeCRC_AES)) {
      errorClickCounter++;

      if(errorClickCounter >= 5) {
        Serial.println("--------------------------");
        Serial.println("Error ocurred. Too many buttons clicked at the same monent!");
        Serial.println("--------------------------");
      } else {
        Serial.println("--------------------------");
        Serial.println("click just one button!");
        Serial.println("--------------------------");
      }
    } else {
      selectionSignal();
      if(noSecure) {
        Serial.println("\n--------------------------");
        Serial.println("No secure Mode");
        Serial.println("-------------------------\n");
        transmissionType = NON;
        masterMode(NON);
      }
      else if(modeCRC) {
        Serial.println("\n--------------------------");
        Serial.println("CRC8 Mode");
        Serial.println("-------------------------\n");
        transmissionType = CRC;
        masterMode(CRC);
      }
      else if(modeCRC_AES) {
        Serial.println("\n--------------------------");
        Serial.println("CRC8 + AES128 Mode");
        Serial.println("-------------------------\n");
        transmissionType = CRC_AES;
        masterMode(CRC_AES);
      }
      else {
        Serial.println("--------------------------");
        Serial.println("++++++++++++++++++++++++++");
        Serial.print("Sth went wrong! Please reset Arduino.");
        Serial.println("++++++++++++++++++++++++++");
        Serial.println("--------------------------");
        return;
      }
    }
  }
}

void setup() {
  assigningIO();

  Serial.begin(57600);
  printf_begin();

  defineMode();
  radioSetUp();
}

void loop() {
  if(isMaster) {
    masterStart();
  } else if (!isMaster) {
    slaveMode();
  }

  delay(1000);

  if (isMaster) {
    for(int i = 0; i <= 15; i++) {
      availableSlavesAddresses[i] = 0x00;
    }
    slavesNumber = 0;
  }
}
