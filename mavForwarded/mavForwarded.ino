#include <Arduino.h>
#include "TeensyThreads.h"
#include "mavlink.h"
#include "common/mavlink_msg_distance_sensor.h"
#define SERIAL_GCS Serial2
#define SERIAL_GCS_BAUD 256000
#define SERIAL_VEHICLE Serial1
#define SERIAL_VEHICLE_BAUD 256000
const int LED = 13;

volatile int blinkcode = 0;

void comm_receive() {
  mavlink_message_t msg;
  mavlink_status_t status;
  
  while (1){
    
  
  
  if(SERIAL_GCS.available()>0) {
    uint8_t c = SERIAL_GCS.read();

    // Try to get a new message
    
    SERIAL_VEHICLE.write(c);
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      // Handle message
      switch(msg.msgid) {
        case MAVLINK_MSG_ID_HEARTBEAT:  // #0: Heartbeat
          {
           digitalWrite(13,1-digitalRead(13));
          }
          break;

      
       default:
          break;
      }
    }
  }

   if(SERIAL_VEHICLE.available()>0)
    SERIAL_GCS.write(SERIAL_VEHICLE.read());
   



  
  //threads.yield();
  }
}

void setup() {
  pinMode(LED, OUTPUT);
  delay(1000);
  //Serial.begin(115200);
  SERIAL_GCS.begin(SERIAL_GCS_BAUD);
  SERIAL_VEHICLE.begin(SERIAL_VEHICLE_BAUD);
  Serial5.begin(256000);
  //threads.addThread(comm_receive);
 
}

int count = 0;

void loop() {
  Serial1.println("^^");
  //Serial2.println("^^");
  //Serial.println("^^");
  delay(100);
  /*
  if (Serial.available()) {      // If anything comes in Serial (USB),

    Serial1.write(Serial.read());   // read it and send it out Serial1 (pins 0 & 1)

  }

  if (Serial1.available()) {     // If anything comes in Serial1 (pins 0 & 1)

    Serial.write(Serial1.read());   // read it and send it out Serial (USB)
  }
  */
   
  
}
