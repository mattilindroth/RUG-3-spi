#include <Arduino.h>
#include <SPI.h>
#include "ISConstants.h"
#include "ISComm.h"

#define BUFFER_SIZE 1024

const int CS_PIN = 7; // Chip Select pin

//buffer and communications for IS-RUG-3-IMX-5
static uint8_t s_buffer[BUFFER_SIZE]; //1024
static is_comm_instance_t comm;

void setup() {
  Serial.begin(115200);
  delay(1000);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH); // Ensure CS is high

  is_comm_init(&comm, s_buffer, BUFFER_SIZE);

  SPI.begin(); // Initialize the SPI library
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE3));
  // SPI.setDataMode(SPI_MODE3); // Set SPI mode to 3
  // SPI.setClockDivider(SPI_CLOCK_DIV64); // Set clock speed (adjust as needed)
  int messageSize = is_comm_get_data_to_buf(s_buffer, sizeof(s_buffer), &comm, DID_INS_1, sizeof( ins_1_t ), 0, 10);
  Serial.print("is_com_get_data_to_buff message size is ");
  Serial.println(messageSize);
  //Serial1.write(comm.rxBuf.start, messageSize); // Transmit the message to the inertialsense device
  portWrite(0, s_buffer, messageSize);

  // messageSize = is_comm_get_data(portWrite, 0, &comm, DID_INS_1, 0, 0, 10);
  // Serial.print("is_com_get_data_to_buff message size is ");
  // Serial.println(messageSize);

}

void loop() {
  
  unsigned int readCount = readSPI();
  if(readCount > 0) {
    // Serial.print("Read ");
    // Serial.print(readCount);
    // Serial.println(" bytes from SPI:");
    // for(int i = 0; i < readCount; i++) {
    //   Serial.print(s_buffer[i], HEX);
    //   Serial.print(", ");
    // }
    // Serial.println();
  }
  delay(100);
}


static int portWrite(unsigned int port, const unsigned char* buf, int len)
{
  byte retVal;
  digitalWrite(CS_PIN, LOW);
  for(int i = 0; i < len; i++) {
    retVal = SPI.transfer(buf[i]);
    Serial.print(retVal, HEX);
    Serial.print(", ");
  }
  Serial.println();
  digitalWrite(CS_PIN, HIGH);
  return len;
}

unsigned int readSPI() {
  unsigned int cnt = 0;
  digitalWrite(CS_PIN, LOW); // Select the SPI device
  byte data;
  //data = SPI.transfer(0x01); // Read the data
  do {
    data = SPI.transfer(0x00);
    // Serial.print(data, HEX);
    // Serial.print(", ");
    uint32_t message_type = is_comm_parse_byte(&comm, data);
    s_buffer[cnt] = data;
     switch(message_type) {
      case _PTYPE_INERTIAL_SENSE_DATA:
        switch (comm.rxPkt.dataHdr.id)
        {
        case DID_NULL:
            break;
        case DID_INS_1:
            Serial.println("DID_INS_1 message received succesfully!");
            //handleINSMessage((ins_1_t *)(comm.rxPkt.data.ptr));
            break;
        default:
            Serial.print("Got an unexpected message DID: ");
            Serial.println(message_type, DEC);
      }
     }
    cnt++;
  } while(data != 0 && cnt < BUFFER_SIZE);
  // Serial.println();
  digitalWrite(CS_PIN, HIGH); // Deselect the SPI device
  return cnt;
}

void handleINSMessage(ins_1_t *ins)
{
    // Serial.print("Lat: ");
    // Serial.print((float)ins->lla[0], 6);
    // Serial.print("\t");
    // Serial.print(", Lon: ");
    // Serial.print((float)ins->lla[1], 6);
    // Serial.print("\t");
    // Serial.print(", Alt: ");
    // Serial.print((float)ins->lla[2], 2);
    // Serial.print("\t");
    // Serial.print("Roll: ");
    // Serial.println(ins->theta[0] * C_RAD2DEG_F, 4);
    // Serial.print("\t");
    // Serial.print(", Pitch: ");
    // Serial.print(ins->theta[1] * C_RAD2DEG_F, 4);
    // Serial.print("\t");
    // Serial.print(", Yaw: ");
    // Serial.print("\t");
    // Serial.println(ins->theta[2] * C_RAD2DEG_F, 4);
    double heading = ins->theta[2] * C_RAD2DEG_F;

    double pitch = ins->theta[1] * C_RAD2DEG_F;
    Serial.print("Pitch: ");
    Serial.print(pitch, 4);
    Serial.print("[deg]. Heading: ");
    Serial.print(heading, 4);
    Serial.println("[deg]");

}
