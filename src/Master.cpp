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

const uint8_t message[] = {"SlaveWojtkowiakGala"};

const uint8_t emptyArray[] = { 0x00, 0x01, 0x02, 0x03,
                               0x04, 0x05, 0x06, 0x07,
                               0x08, 0x09, 0x0A, 0x0B,
                               0x0C, 0x0D, 0x0E, 0x0F,
                               0x10, 0x11, 0x12};

uint8_t transmissionType = NON;

const int timeToStart = 1000;
const int timeToNextSTM = 4000;

int slaveStartSendingTime = 0;
int slaveStartNextSTM = 0;

bool isMaster = false;

static const byte crc_gen = 0x39;

static int errorCounter = 0;

long startTime = 0;

bool noSecure = false, modeCRC = false, modeCRC_AES = false;

// TODO: reset and clear those two values (under) after whole process
static int slavesNumber = 0;
uint8_t *availableSlavesAddresses = new uint8_t[15];

uint8_t *dataToSend = new uint8_t[FRAMELENGTH];
uint8_t *dataReceived = new uint8_t[FRAMELENGTH];

void radioSetUp() {
   radio.begin();

   radio.disableCRC();
   // radio.setCRCLength(RF24_CRC_8);

   radio.setChannel(16);

   radio.setAutoAck(false);
   radio.setDataRate(RF24_1MBPS);
   radio.setPALevel(RF24_PA_MIN);

   radio.setRetries(0,0);

   radio.setPayloadSize(FRAMELENGTH);

   if(isMaster) {
     Serial.println("---Master---");
     radio.openWritingPipe(adr[0]);
     radio.openReadingPipe(1, adr[1]);
   } else if (!isMaster) {
     Serial.println("---Slave---");
     radio.openWritingPipe(adr[1]);
     radio.openReadingPipe(1, adr[0]);
   }

   radio.startListening();

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
  // byte bytes[name.length() + 1];
  // name.getBytes(bytes, name.length() + 1);
  // byte a = CRC8(bytes, name.length());

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
  if (!digitalRead(BUTTON1)) {
    isMaster = true;
  } else if(!digitalRead(BUTTON2)) {
    isMaster = false;
  }
}

void selectionSignal() {
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
}

uint8_t* convIntToUint8(int intParase) {
  uint8_t* bytesArray = new uint8_t[4];

  for (int i = 0; i < 4; i++) {
      bytesArray[3 - i] = (intParase >> (i * 8)) & 0xFF;
  }
  return bytesArray;
}

int convUint8ToInt(uint8_t* uintParase) {
  return (uintParase[0] << 24) | (uintParase[1] << 16) | (uintParase[2] << 8) | uintParase[3];
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

void errorHandler(uint8_t address) {
  radio.stopListening();

  createFrame(address, ERR, 0x00);

  radio.write(dataToSend, FRAMELENGTH);

  errorCounter++;

  Serial.print("Count of errors: ");
  Serial.println(errorCounter);
}

bool getACKMaster() {
  if(*(dataReceived + 2) == ACK) {
    Serial.print("ACK received from");
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
  radio.write(dataToSend, FRAMELENGTH);

  // Debugging
  printHex(dataToSend, FRAMELENGTH);
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
    }

    radio.stopListening();

}

// TODO: Do
void fillMessage (uint8_t* data) {
  for(int i = 0; i <= 15; i++) {
    dataToSend[i] = *(data + 1);
  }
}

void setSlaveTimeSlot(uint8_t address, uint8_t transmissionMode) {
  uint8_t* slotTimeData = new uint8_t[4];
  uint8_t* nextSTM = new uint8_t[4];
  uint8_t* dataPrepared = new uint8_t[18];

  int slotTime = 0;

  radio.stopListening();

  for(int i = 0; i <= slavesNumber; i++) {
    slotTime = timeToStart + i * 50;

    slotTimeData = convIntToUint8(slotTime);
    nextSTM = convIntToUint8(timeToNextSTM);

    for(int j = 0; j <= 8; j++) {
      if(j < 4) {
        dataPrepared[j] = slotTimeData[j];
        dataPrepared[j + 4] = nextSTM[j];
      }
      dataPrepared[j + 8] = 0x00;
    }
    if(transmissionMode == NON) {
      dataPrepared[17] = 0x00;
    } else if(transmissionMode == CRC) {
      dataPrepared[17] = CRC8(dataPrepared, 16);
    } else if(transmissionMode == CRC_AES) {
      // TODO: AES z CRC8 jest zaszyfrowany
      dataPrepared[17] = CRC8(dataPrepared, 16);
      aes128_enc_single(keyAES, dataPrepared);
    }

    createFrame(address, CNF, dataPrepared);

    radio.write(dataToSend, FRAMELENGTH);
  }
}

void sendACK(uint8_t address) {
  radio.stopListening();
  delay(100);
  createFrame(address, ACK, 0x00);
  radio.write(dataToSend, FRAMELENGTH);
  Serial.println("ACK sent");
}

// TODO: Do
void sendMessage() {
  while(true) {

  }
}

// TODO: Do
void getTime() {
  uint8_t* tempArray = new uint8_t[4];

  for(int i = 0; i < 4; i++) {
    tempArray[i] = *(dataReceived + i + 4);
  }
  slaveStartSendingTime = convUint8ToInt(tempArray);

  for(int i = 0; i < 4; i++) {
    tempArray[i] = *(dataReceived + i + 8);
  }
  slaveStartNextSTM = convUint8ToInt(tempArray);
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

// TODO: Do
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
  printHex(dataReceived, FRAMELENGTH);
  Serial.println();

  if(*(dataReceived + 1) == address || *(dataReceived + 1) == RN_0) {
    switch (*(dataReceived + 2)) {
      case NON:
        transmissionType = NON;
        sendACK(RN_S);
        break;
      case CRC:
        transmissionType = CRC;
        sendACK(RN_S);
        break;
      case CRC_AES:
        transmissionType = CRC_AES;
        sendACK(RN_S);
        break;
      case STM:

        break;
      case CNF:
        if(transmissionType == CRC_AES) {
          aes128_dec_single(keyAES, dataReceived);
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
        sendACK(RN_S);
        break;
    }
  } else {
    Serial.println("Nothing to read!");
    {
      sendACK(RN_S);
      return;
    }
  }
  radio.stopListening();
}

void slaveMode() {
  radio.startListening();

  while(radio.available()) {
    radio.read(dataReceived, FRAMELENGTH);
  }
  radio.stopListening();

  if(*(dataReceived + 1) == RN_S && *(dataReceived + 2) == CNF) {
    switch (*(dataReceived + 2)) {
      case NON:
        break;
      case CRC:
        break;
      case CRC_AES:
        break;
    }
  } else if(*(dataReceived + 2) == STM) {

  }

}

void masterMode(uint8_t transmissionMode) {
  slavesNumber = 0;

  radio.stopListening();

  Serial.println("Looking for Nodes");

  for(uint8_t i = 0x01; i <= 15; i++) {
    presenceTest(i, transmissionMode);
  }

  if(slavesNumber == 0) {
    Serial.print("There is no Nodes to communicate with.");
    return;
  } else {
    for(int i = 0; i <= slavesNumber; i++){
      setSlaveTimeSlot(*(availableSlavesAddresses + i), transmissionMode);
    }
  }

  delay(1000);

  startTime = millis();

  createFrame(0x00, STM, 0x00);
  radio.write(dataToSend, FRAMELENGTH);

  for(int i = 0; i < slavesNumber; i++) {

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
        Serial.println("Error ocurred. Too many buttons clicked at the same monent!");
      } else {
        Serial.println("click just one button!");
      }
    } else {
      selectionSignal();
      if(noSecure)
        masterMode(NON);
      else if(modeCRC)
        masterMode(CRC);
      else if(modeCRC_AES)
        masterMode(CRC_AES);
      else {
        Serial.print("Sth went wrong! Please reset Arduino.");
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
  // isMaster = false;
  isMaster = true;
  radioSetUp();
}

void loop() {
  // receiveData(RN_S);
  // presenceTest(RN_S, NON);
  Serial.println("--------------------------");
  radioSetUp();
  Serial.println("--------------------------");

  delay(10000);
}
