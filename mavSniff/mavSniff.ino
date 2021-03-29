#include <Arduino.h>
#include "common/mavlink.h"
//#include "ardupilotmega/mavlink.h"
#include "sbus.h"
#define SERIAL_GCS Serial1 
#define SERIAL_ARDUPILOT Serial2
#define SERIAL_GCS_BAUD 9600
#define SERIAL_ARDUPILOT_BAUD 115200
std::array<uint16_t, 16> channels;
const int LED = 13;
SbusTx sbus_tx(&Serial2);
int16_t current_battery = 0; //cA
uint16_t voltage_battery = 0; //mV
int32_t alt = 0; //mm
int32_t lat = 0; //degE7
int32_t lon = 0; //degE7
uint16_t vel = 0; //cm/s
int16_t xacc = 0; //
int16_t yacc = 0; //
int16_t zacc = 0; //
int16_t xgyro = 0; //
int16_t ygyro = 0; //
int16_t zgyro = 0; //
uint16_t hdg = 0; //cdeg
uint32_t custom_mode = 0;
uint8_t fix_type = 0;
int32_t current_consumed = 0; //mAh
/*
 * given https://mavlink.io/en/messages/ardupilotmega.html
 * and https://mavlink.io/en/messages/common.html
Batt.number = inav.getRxBatt(); -> <Unknown>
Fuel.number = inav.getFuel(); -> <Unknown>
Current.number = inav.getCurrent(); -> SYS_STATUS ( #1 ) voltage_battery
or BATTERY_STATUS ( #147 ) current_battery
Voltage.number = inav.getVoltage(); -> SYS_STATUS ( #1 ) current_batteru
or BATTERY_STATUS ( #147 ) voltages
Altitude.number = inav.getAltitude(); -> GLOBAL_POSITION_INT ( #33 ) relative_alt
or AHRS2 ( #178 ) -> altitude
Vario.number = inav.getVario(); -> <Unknown>
Lat.number = inav.getLat(); -> GPS_RAW_INT ( #24 ) lat
or AHRS2 ( #178 ) -> lat
or GLOBAL_POSITION_INT ( #33 ) -> lat
Lon.number = inav.getLon(); -> GPS_RAW_INT ( #24 ) lon
or AHRS2 ( #178 ) -> lon
or GLOBAL_POSITION_INT ( #33 ) -> lon
Speed.number = inav.getSpeed(); -> GPS_RAW_INT ( #24 ) vel (vertical speed)
AccX.number = inav.getAccX(); -> RAW_IMU ( #27 ) xacc
AccY.number = inav.getAccY(); -> RAW_IMU ( #27 ) yacc
AccZ.number = inav.getAccZ(); -> RAW_IMU ( #27 ) zacc
Heading.number = inav.getHeading(); -> AHRS2 ( #178 ) yaw
or ATTITUDE ( #30 ) yaw
or GLOBAL_POSITION_INT ( #33 ) hdg
CapacityUsed.number = inav.getCapacityUsed(); -> BATTERY_STATUS ( #147 ) current_consumed
FlightMode.number = inav.getFlightMode(); -> HEARTBEAT ( #0 ) custom_mode
GpsState.number = inav.getGpsState(); -> GPS_RAW_INT ( #24 ) vel fix_type
 */
void comm_receive_from_gcs() {
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
void comm_receive_from_ardupilot() {
  mavlink_message_t msg;
  mavlink_status_t statust;

  while(SERIAL_ARDUPILOT.available()>0) {
    uint8_t c = SERIAL_ARDUPILOT.read();
    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &statust)) {

      switch(msg.msgid) {
        case 0:  // #0: Heartbeat
          {
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&msg, &heartbeat);
            custom_mode = heartbeat.custom_mode;
            break;
          }
        case 1: // SYS_STATUS
          {
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);
            voltage_battery = sys_status.voltage_battery;
            voltage_battery = sys_status.current_battery;
            break;
          }
        case 24: // GPS_RAW_INT
          {
            mavlink_gps_raw_int_t gps_raw_int;
            mavlink_msg_gps_raw_int_decode(&msg, &gps_raw_int);
            lat = gps_raw_int.lat;
            lon = gps_raw_int.lon;
            vel = gps_raw_int.vel;
            fix_type = gps_raw_int.fix_type;
            
            
            break;
          }
        case 27: // RAW_IMU
          {
            mavlink_raw_imu_t raw_imu;
            mavlink_msg_raw_imu_decode(&msg, &raw_imu);
            xacc = raw_imu.xacc;
            yacc = raw_imu.yacc;
            zacc = raw_imu.zacc;
            xgyro = raw_imu.xgyro;
            ygyro = raw_imu.ygyro;
            zgyro = raw_imu.zgyro;
            break;
          }     
        case 30: // ATTITUDE
          {
            mavlink_attitude_t attitude;
            mavlink_msg_attitude_decode(&msg, &attitude);
            break;
          }                 
          //
        case 33: //GLOBAL_POSITION_INT
          {
            mavlink_global_position_int_t global_position_int;
            mavlink_msg_global_position_int_decode(&msg, &global_position_int);
            hdg = global_position_int.hdg;
            alt = global_position_int.alt;
            break;
          }
        case 147: //BATTERY_STATUS
          {
            mavlink_battery_status_t battery_status;
            mavlink_msg_battery_status_decode(&msg, &battery_status);
            current_consumed = battery_status.current_consumed;
            break;
          }
        
          
       
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
  //SERIAL_GCS.begin(SERIAL_GCS_BAUD);
  SERIAL_ARDUPILOT.begin(SERIAL_GCS_BAUD);
  sbus_tx.Begin();
  
  delay(500);
  for (int i = 0; i < 16; i++){
    channels[i] = 1000;
  }
  
}

int count = 0;

void loop() {

   //comm_receive_from_gcs();
   comm_receive_from_ardupilot();
   Serial.print("Current: ");
   Serial.println(current_battery);

   Serial.print("Voltage: ");
   Serial.println(voltage_battery);

   Serial.print("Altitude: ");
   Serial.println(alt);

   Serial.print("Lat: ");
   Serial.println(lat);   

   Serial.print("Lon: ");
   Serial.println(lon);   

   Serial.print("Speed: ");
   Serial.println(vel);

   Serial.print("AccX: ");
   Serial.println(xacc);


   Serial.print("AccY: ");
   Serial.println(yacc);


   Serial.print("AccZ: ");
   Serial.println(zacc);
   
   Serial.print("Xgyro: ");
   Serial.println(xgyro);


   Serial.print("Ygyro: ");
   Serial.println(ygyro);


   Serial.print("Zgyro: ");
   Serial.println(zgyro);
   
   Serial.print("Heading: ");
   Serial.println(hdg);

   Serial.print("CapacityUsed: ");
   Serial.println(current_consumed);   


   Serial.print("FlightMode: ");
   Serial.println(custom_mode);  

   Serial.print("GpsState: ");
   Serial.println(fix_type);        
   //sbus_tx.tx_channels(channels);
   //sbus_tx.Write();
   //delay(10);
  
}
