"""

enum class Number {
    STABILIZE =     0,  // manual airframe angle with manual throttle
    ACRO =          1,  // manual body-frame angular rate with manual throttle
    ALT_HOLD =      2,  // manual airframe angle with automatic throttle
    AUTO =          3,  // fully automatic waypoint control using mission commands
    GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
    LOITER =        5,  // automatic horizontal acceleration with automatic throttle
    RTL =           6,  // automatic return to launching point
    CIRCLE =        7,  // automatic circular flight with automatic throttle
    LAND =          9,  // automatic landing with horizontal position control
    DRIFT =        11,  // semi-automous position, yaw and throttle control
    SPORT =        13,  // manual earth-frame angular rate control with manual throttle
    FLIP =         14,  // automatically flip the vehicle on the roll axis
    AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
    POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
    BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
    THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
    AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
    GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
    SMART_RTL =    21,  // SMART_RTL returns to home by retracing its steps
    FLOWHOLD  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
    FOLLOW    =    23,  // follow attempts to follow another vehicle or ground station
    ZIGZAG    =    24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
    NEW_MODE =     25,  // your new flight mode
};

"""

from pymavlink import mavutil
import time
import math
from pymavlink.dialects.v20 import ardupilotmega as mavlink2
from pymavlink.dialects.v20 import ardupilotmega as mavlink1
SIMULATOR_UDP = 'udp:127.0.0.1:14550'
TCP_650 = 'tcp:192.168.0.210:20002'



master = mavutil.mavlink_connection("COM119", baud=9600)
def sendMessageDistanceSensor(master):
	#master.mav.ping_send(
    #        time.time(), # Unix time
    #        0, # Ping number
    #        0, # Request ping of all systems
    #        0 # Request ping of all components
    #    )
	master.mav.distance_sensor_send(
                        0, #time
                        10, #min distance
                        4000, #max distance
                        290, # current
                        3, #type
                        1, #id
                        0, #ori
                        255)
	#master.mav.send(msss)

def sendRC(master):
	master.mav.servo_output_raw_send(0, 1, 
	1100, 
	1200, 
	1300, 
	1400, 
	1500, 
	1600, 
	1700, 
	1800, 
	force_mavlink1=False)
def sendRC_2(master):
	master.mav.servo_output_raw_send(0, 1, 
	1900, 
	1200, 
	1300, 
	1400, 
	1500, 
	1600, 
	1700, 
	1800, 
	force_mavlink1=False)
def sendHB(master):
	master.mav.heartbeat_send(1, 
	3, 
	2, 
	1, 
	0, 
	mavlink_version=3, 
	force_mavlink1=False)
	#time.sleep(0.5)
def connectToSim(master):
    msg = None
    while not msg:
        master.mav.ping_send(
            time.time(), # Unix time
            0, # Ping number
            0, # Request ping of all systems
            0 # Request ping of all components
        )
        msg = master.recv_match()
        time.sleep(0.5)

connectToSim(master)

#aa = simpleVectorVelocity(10,10)
#moveFollowVector(aa,0.5)
#time.sleep(10)
#bb = simpleVectorVelocity(30,60)
#moveFollowVector(bb,30)
#time.sleep(4)

while True:
	
	#msg = master.recv_match()
	#if not msg:
	#	continue
	#print msg
	#sendMessageDistanceSensor(master)
	time.sleep(0.5);
	sendRC(master);
	time.sleep(0.5);
	sendRC_2(master);
	#master.mav.distance_sensor_send(
    #                    time.time(), #time
    #                    10, #min distance
    #                    4000, #max distance
    #                    290, # current
    #                    3, #type
    #                    1, #id
    #                    0, #ori
    #                    255)
	#if msg.get_type() == 'LOCAL_POSITION_NED':
	#	print msg.x, msg.y, msg.z
  
#while True:
    #print master.mode_mapping()
#moveRightUntil(0.1)

	

