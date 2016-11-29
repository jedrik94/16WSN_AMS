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

bool isMaster = false;

static int errorCounter = 0;

long startTime = 0;

// TODO: reset and clear those two values (under) after whole process
int slavesNumber = 0;
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

void assigningIO() {
  pinMode(LED_PIN, OUTPUT);

  pinMode(BUTTON1, INPUT_PULLUP);
  pinMode(BUTTON2, INPUT_PULLUP);
  pinMode(BUTTON3, INPUT_PULLUP);
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
  bool timeout = false;

  radio.stopListening();

  createFrame(address, TSA);
  radio.write(dataToSend, FRAMELENGTH);

  radio.startListening();

  while(!radio.available()) {
    if((millis() - startTime) > TIMESLOT) {
      timeout = true;
      break;
    }
  }

  if(timeout) {
    Serial.print("Timeout for node: ");
    Serial.println(address, HEX);
  } else {
    // TODO: logs + ?? is it good method to get address
    radio.read(dataReceived, FRAMELENGTH);
    availableSlavesAddresses[slavesNumber] = dataReceived[1];
    slavesNumber++;
    radio.stopListening();
  }
}

// TODO: SlotTime
void setSlaveTimeSlot(uint8_t address) {

  radio.stopListening();

  createFrame(address, CNF);
}

void errorHandler(uint8_t address) {
  radio.stopListening();

  createFrame(address, ERR);

  radio.write(dataToSend, FRAMELENGTH);

  errorCounter++;

  Serial.print("Count of errors: ");
  Serial.println(errorCounter);
}

void getACK(uint8_t address) {
  bool timeout = false;

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
    return;
  }

  radio.read(dataReceived, FRAMELENGTH);
  radio.stopListening();

  if (isMaster) {
    getACKMaster();
  } else if (!isMaster) {
    getACKSlave();
  }

}

void getACKMaster() {
  if(*dataReceived + 2 == ACK) {
    Serial.print("ACK received from");
    Serial.print(*dataReceived + 1, HEX);
    Serial.println();
  } else {
    errorHandler(RN_0);
  }
}

void getACKSlave() {
  if(*dataReceived + 2 == ACK) {
    Serial.print("ACK received from Master");
    Serial.println();
  } else {
    errorHandler(*dataReceived + 1);
  }
}

void sendACK(uint8_t address) {
  radio.stopListening();
  createFrame(address, ACK);
  radio.write(dataToSend, FRAMELENGTH);
  Serial.println("ACK sent");
}

void setup() {
  assigningIO();

  Serial.begin(9600);

  defineMode();
  radioConfig();

}

void loop() {

}
