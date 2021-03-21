#include <Arduino.h>
#include "TeensyThreads.h"
#include "mavlink.h"
#include "sbus.h"
#define SERIAL_GCS Serial1 
#define SERIAL_GCS_BAUD 9600
std::array<uint16_t, 16> channels;
const int LED = 13;
SbusTx sbus_tx(&Serial2);

void comm_receive() {
  mavlink_message_t msg;
  mavlink_status_t statust;
  
  while(SERIAL_GCS.available()>0) {
    uint8_t c = SERIAL_GCS.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &statust)) {

      switch(msg.msgid) {
        case 0:  // #0: Heartbeat
          {
           // check if heartbeat is receive
           // Serial.println("Heartbear received!");
          }
          break;
        case 36:  // #36: Servo Out Raw
          {
            digitalWrite(13,1-digitalRead(13));
            mavlink_servo_output_raw_t servo_out_raw;
            mavlink_msg_servo_output_raw_decode(&msg, &servo_out_raw);
            channels[0] = (uint16_t)servo_out_raw.servo1_raw;
            channels[1] = servo_out_raw.servo2_raw;
            channels[2] = servo_out_raw.servo3_raw;
            channels[3] = servo_out_raw.servo4_raw;
            channels[4] = servo_out_raw.servo5_raw;
            channels[5] = servo_out_raw.servo6_raw;
            channels[6] = servo_out_raw.servo7_raw;
            channels[7] = servo_out_raw.servo8_raw;
            //print out to check wheter
            for (int i = 0; i < 8; i++){
                  Serial.print(channels[i]);
                  Serial.print(',');
              }
            Serial.println();
          }
          break;
        case 4:  // #0 Ping message
          {
 
          }
          break;
       default:
          break;
      }
    }
  }
}

void setup() {
  pinMode(LED, OUTPUT);
  delay(1000);
  Serial.begin(115200);
  SERIAL_GCS.begin(SERIAL_GCS_BAUD);
  sbus_tx.Begin();
  
  delay(500);
  for (int i = 0; i < 16; i++){
    channels[i] = 1000;
  }
  
}

int count = 0;

void loop() {

   comm_receive();
   sbus_tx.tx_channels(channels);
   sbus_tx.Write();
   delay(10);
  
}
