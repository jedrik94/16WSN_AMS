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

const int timeToStart = 1000;

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

void radioConfig() {
   radio.begin();

   radio.disableCRC();
   // radio.setCRCLength(RF24_CRC_8);

   radio.setChannel(16);

   radio.setAutoAck(false);
   radio.setDataRate(RF24_1MBPS);

   radio.setRetries(0,0);

   radio.setPayloadSize(21);

   if(isMaster) {
     radio.openWritingPipe(adr[0]);
     radio.openReadingPipe(1,adr[1]);
   } else if (!isMaster) {
     radio.openReadingPipe(1, adr[0]);
     radio.openWritingPipe(adr[1]);
   }

   // radio.printDetails();
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

void assigningIO() {
  pinMode(LED_PIN, OUTPUT);

  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(BUTTON3, INPUT);
}

void defineMode() {
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

void createFrame(uint8_t RN, uint8_t FNC) {
  dataToSend[0] = NN;
  dataToSend[1] = RN;
  dataToSend[2] = FNC;
  if(FNC == MES || FNC == CNF)
  {
    // TODO: Fill "data + CRC8Mode"
  } else {
    for(int i = 3; i >= 20; i++)
      dataToSend[i] = 0x55;
  }
}

void presenceTest(uint8_t address) {
  // bool timeout = false;
  startTime = millis();

  radio.stopListening();

  createFrame(address, TSA);
  radio.write(dataToSend, FRAMELENGTH);

  radio.startListening();

  // while(!radio.available()) {
  //   if((millis() - startTime) > TIMESLOT) {
  //     timeout = true;
  //     break;
  //   }
  // }

  // if(timeout) {
  //   Serial.print("Timeout for node: ");
  //   Serial.println(address, HEX);
  // } else {
    // TODO: logs + ?? is it good method to get address

    if(getACK(address)) {
      availableSlavesAddresses[slavesNumber] = *(dataReceived + 1);
      Serial.print("ACK recieved from Node. Address: ");
      Serial.println(*(dataReceived + 1), HEX);
      Serial.print("Current number of slaves: ");
      Serial.println(slavesNumber + 1);
      slavesNumber++;
    } else {
      Serial.print("ACK not recieved from Node. There is no such a Node like: ");
      Serial.println(*(dataReceived + 1), HEX);
    }

    // radio.read(dataReceived, FRAMELENGTH);

    radio.stopListening();

}

// TODO: Do
void fillMessage (uint8_t* data) {
  for(int i = 0; i <= 15; i++) {
    dataToSend[i] = *(data + 1);
  }
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

// TODO: SlotTime
void setSlaveTimeSlot(uint8_t address) {
  int slotTime = 0;

  radio.stopListening();

  for(int i = 0; i <= numberOfSlaves; i++) {
    slotTime = timeToStart + i * 50;

    convIntToUint8(slotTime);
  }

  createFrame(address, CNF);

  radio.write(dataToSend, FRAMELENGTH);
}

void errorHandler(uint8_t address) {
  radio.stopListening();

  createFrame(address, ERR);

  radio.write(dataToSend, FRAMELENGTH);

  errorCounter++;

  Serial.print("Count of errors: ");
  Serial.println(errorCounter);
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
      Serial.print("Timeout of connection with Node");
      errorHandler(RN_0);
    } else if (!isMaster) {
      Serial.print("Timeout of connection with Master");
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

bool getACKMaster() {
  if(*dataReceived + 2 == ACK) {
    Serial.print("ACK received from");
    Serial.print(*dataReceived + 1, HEX);
    Serial.println();
    return true;
  } else {
    errorHandler(RN_0);
    return false;
  }
}

bool getACKSlave() {
  if(*dataReceived + 2 == ACK) {
    Serial.print("ACK received from Master");
    Serial.println();
    return true;
  } else {
    errorHandler(*dataReceived + 1);
    return false;
  }
}

void sendACK(uint8_t address) {
  radio.stopListening();
  createFrame(address, ACK);
  radio.write(dataToSend, FRAMELENGTH);
  Serial.println("ACK sent");
}

// TODO: Do
void sendMessage() {
  while(true) {

  }
}

void receiveMessage(uint8_t address) {
  radio.startListening();

  radio.read(dataReceived, FRAMELENGTH);

  if(*(dataReceived + 1) == address) {

  }
}

void foo2(uint8_t transmissionMode) {
  slavesNumber = 0;

  Serial.println("Looking for Nodes");

  for(uint8_t i = 0x01; i <= 0x15; i++)
    presenceTest(i);

  if(slavesNumber == 0)
    return;

}

void foo1() {
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
        foo2(NON);
      else if(modeCRC)
        foo2(CRC);
      else if(modeCRC_AES)
        foo2(CRC_AES);
      else {
        Serial.print("Sth went wrong! Please reset Arduino.");
        return;
      }
    }
  }
}

void setup() {
  assigningIO();

  Serial.begin(9600);

  defineMode();
  radioConfig();

}

void loop() {

}
