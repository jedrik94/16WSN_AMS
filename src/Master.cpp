#include <Arduino.h>

#include <SPI.h>
#include <RF24.h>
#include <printf.h>
#include <nRF24L01.h>
#include <AESLib.h>
#include "Definitions.h"

uint8_t sentFramesCounter = 0x00; // counts sent frames from node

RF24 radio(9,10); // init of RF24

const uint64_t adr[2] = { 0x1B1B1B1B21LL, 0x1B1B1B1B12LL }; // piplines

const uint8_t keyAES[] = { 0x00, 0x01, 0x02, 0x03,
                           0x04, 0x05, 0x06, 0x07,
                           0x08, 0x09, 0x0A, 0x0B,
                           0x0C, 0x0D, 0x0E, 0x0F}; // AES encryption key

uint8_t message[] = {"SlaveGalaWojtkow"}; // message to send to master

uint8_t* tempArray = new uint8_t[16];

uint8_t transmissionType = NON; // initial transmission mode

const long timeToStart = 1000; // time to start STM
const long timeToNextSTM = 4000; // time to next STM

long slaveStartSendingTime = 0; // temps to define time for slave
long slaveStartNextSTM = 0;

bool isMaster = false; // mode master/slave

static const byte crc_gen = 0x39; // crc generator

static int errorCounter = 0; // error counter

long startTime = 0; // timer

bool noSecure = false, modeCRC = false, modeCRC_AES = false; // temps to define transmission mode

static int slavesNumber = 0; // slaves counter
uint8_t* availableSlavesAddresses = new uint8_t[15]; // array with available slaves addresses

uint8_t* dataToSend = new uint8_t[FRAMELENGTH]; // arrays of communication data
uint8_t* dataReceived = new uint8_t[FRAMELENGTH];
uint8_t* onlyMessage = new uint8_t[17];

/**
    setting up radio parameters
*/
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

/**
    printing HEX values to serial port
*/
void printHex(uint8_t* data, int length) {
    for (int i = 0; i < length; i++) {
      Serial.print("0x");
      if(data[i] < 0x10)
        Serial.print("0");
      Serial.print(data[i], HEX);
      Serial.print(" ");
    }
}

/**
    CRC8 values creator
*/
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

/**
    assigning inputs and outputs
*/
inline void assigningIO() {
  pinMode(LED_PIN, OUTPUT);

  pinMode(BUTTON1, INPUT);
  pinMode(BUTTON2, INPUT);
  pinMode(BUTTON3, INPUT);
}

/**
    defining node mode master or slave
*/
inline void defineMode() {
  if (digitalRead(BUTTON1)) {
    isMaster = true;
  } else if(digitalRead(BUTTON2)) {
    isMaster = false;
  }
}

/**
    signalization of chosen transmission mode
*/
void selectionSignal() {
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  digitalWrite(LED_PIN, LOW);
}

/**
    converter from 32bits long to array of 8bits chars
*/
uint8_t* convIntToUint8(long intParase) {
  uint8_t* bytesArray = new uint8_t[4];

  for (int i = 0; i < 4; i++) {
      bytesArray[3 - i] = (intParase >> (i * 8)) & 0xFF;
  }
  return bytesArray;
}

/**
    converter from array of 8bits chars to 32bits long
*/
long convUint8ToInt(uint8_t* uintParase) {
  return ((uint32_t)uintParase[0] << 24) | ((uint32_t)uintParase[1] << 16) | ((uint32_t)uintParase[2] << 8) | (uint32_t)uintParase[3];
}

/**
    frame creator, filling spaces in data buffer
*/
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

/**
    handling errors and sending frame with error
*/
void errorHandler(uint8_t address) {
  radio.stopListening();

  createFrame(address, ERR, 0x00);

  // radio.write(dataToSend, FRAMELENGTH);

  errorCounter++;

  Serial.print("Count of errors: ");
  Serial.println(errorCounter);
}

/**
    acknowledge validator (master) - sub-function
*/
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

/**
    acknowledge validator (slave) - sub-function
*/
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

/**
    acknowledge validator
*/
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

/**
    testing presence of slave nodes
*/
void presenceTest(uint8_t address, uint8_t transmissionMode) {

  radio.stopListening();

  createFrame(address, transmissionMode, 0x00);
  delay(800);
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

/**
    setting time for present slaves and passing frames with data to send
*/
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

    // Debugging
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

/**
    sending acknowledge to master address
*/
void sendACK(uint8_t address) {
  radio.stopListening();
  createFrame(address, ACK, 0x00);
  radio.write(dataToSend, FRAMELENGTH);
  Serial.println("ACK sent");
}

/**
    fetching time form gathered data
*/
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


  // Debugging
  Serial.println("@@@@@@@@@@@@@@@");
  Serial.println(slaveStartSendingTime);
  Serial.println("@@@@@@@@@@@@@@@");
  Serial.println(slaveStartNextSTM);
  Serial.println("@@@@@@@@@@@@@@@");
}

/**
    fetching just message form gathered data
*/
uint8_t* getOnlyMessage() {
  uint8_t* tempMessage = new uint8_t [17];

  for(int i = 0; i <= 16; i++) {
    tempMessage[i] = *(dataReceived + i + 4);
  }

  return tempMessage;
}

/**
    CRC8 validator
*/
bool confirmCRC() {

  for(int i = 0; i < 16; i++) {
    tempArray[i] = *(dataReceived + i + 4);
  }

  if(*(dataReceived + 20) == CRC8(tempArray, 16)) {
    return true;
  } else {
    return false;
  }
}

/**
    filling buffer for next preparations
*/
uint8_t* fillMessageBuffer(uint8_t* array) {
  for(int i = 0; i < 16; i++) {
    array[i] = message[i];
  }

  return array;
}

/**
    preparing message and sending it to master (loop)
*/
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

  Serial.println("Time to first");
  Serial.println(timeToStart - (millis() - tempTime));

  delay(timeToStart - (millis() - tempTime));

  while(true) {
    tempTime = millis();
    Serial.println("Time to my sending period");
    Serial.println(slaveStartSendingTime - timeToStart);
    delay(slaveStartSendingTime - timeToStart);

    Serial.println();
    printHex(message, 16);
    Serial.println();
    printHex(dataToSend, 21);
    Serial.println();

    radio.write(dataToSend, FRAMELENGTH);

    Serial.println("Time to next whole period");
    Serial.println(slaveStartNextSTM);
    while(true){
      if((signed long)millis() - tempTime > slaveStartNextSTM - timeToStart)
        break;
    }
  }
}

/**
    data validator, defining next actions after validation of message FNC
*/
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

/**
    message from slave getter
*/
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

  // onlyMessage = getOnlyMessage();

  for(int i = 0; i <= 16; i++) {
    onlyMessage[i] = *(dataReceived + i + 4);
  }

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
    Serial.print((char)onlyMessage[i]);
  }

  Serial.print("\n");

  radio.stopListening();

  Serial.print("End of transmission from Node: ");
  Serial.println(*(dataReceived + 1), HEX);
}

/**
    intermediary function
*/
void slaveMode() {
  receiveData(RN_S);
}

/**
    main master function
*/
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
  delay(700);

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
    while(true){
      if(millis() - startTime > (unsigned long)timeToNextSTM)
        break;
    }
  }
}

/**
    master starter function, validation of chosen transmission mode
*/
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

/**
    setup faunction
*/
void setup() {
  assigningIO();

  Serial.begin(57600);
  printf_begin();

  defineMode();
  radioSetUp();
}

/**
    main loop
*/
void loop() {


  if(isMaster) {
    delay(3000);
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
