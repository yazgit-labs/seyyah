"""
Simple script for take off and control with arrow keys
"""


import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

#- Importing Tkinter: sudo apt-get install python-tk
import Tkinter as tk
'''
###SITL BLOCK
# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Control Copter and send commands in GUIDED mode ')
parser.add_argument('--connect',
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)
###SITL BLOCK
'''
###REAL VECIHLE BLOCK
# Connect to the Vehicle
print('Connecting to vehicle')
vehicle = connect('udpin:0.0.0.0:14550',wait_ready=True)
###REAL VECIHLE BLOCK

#-- Setup the commanded flying speed
gnd_speed = 1 # [m/s]

#-- Global velocity logs
velocityLogList = []

def printALT():
    #Show altitude
    v_alt = vehicle.location.global_relative_frame.alt
    print(">> Altitude = %.1f m"%v_alt)

def printATT():
    #Show attitude
    v_alt = vehicle.location.global_relative_frame
    print(v_alt)

def logVEL(vx,vy,vz):
    velList = [vx,vy,vz]
    velocityLogList.append(velList)

def specialRTL():
    print("Ozel geri donus fonksyinu devrede!!!")
    for i in reversed(velocityLogList):
        set_velocity_body(vehicle, i[0], i[1], i[2])
        printATT()
        time.sleep(1)
    print("Inis modu devrede!!!")
    vehicle.mode = VehicleMode("LAND")
    

#-- Define arm and takeoff
def arm_and_takeoff(altitude):

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)
   timeout = time.time() + 10
   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude - 1.0:
          print("Target altitude reached")
          break
      elif time.time() > timeout:
          print("Can't reached target altitude !")
          break

      time.sleep(1)
      
 #-- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    
    Bitmask to indicate which dimensions should be ignored by the vehicle 
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
    none of the setpoint dimensions should be ignored). Mapping: 
    bit 1: x,  bit 2: y,  bit 3: z, 
    bit 4: vx, bit 5: vy, bit 6: vz, 
    bit 7: ax, bit 8: ay, bit 9:
    
    
    """
    logVEL(-vx,-vy,-vz)

    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
    
    
#-- Key event function
def key(event):
    printATT()
    if event.char == event.keysym: #-- standard keys
        if event.keysym == 'r':
            print("r pressed >> Set the vehicle to RTL")
            vehicle.mode = VehicleMode("LAND")
        elif event.keysym == 't':
            print("t pressed >> Set the vehicle to STABILIZE")
            vehicle.mode = VehicleMode("STABILIZE")
        elif event.keysym == 'y':
            print("y pressed >> Set the vehicle to takeOFF")
            arm_and_takeoff(2)
        elif event.keysym == 'w':
            print("w pressed >> thrust")
            set_velocity_body(vehicle, 0, 0, -1)
        elif event.keysym == 's':
            print("s pressed >> negative_thrust")
            set_velocity_body(vehicle, 0, 0, 1)
        elif event.keysym == 'a':
            print("a pressed >> try to arm")
            vehicle.armed = True
        elif event.keysym == 'h':
            print("h pressed >> Special RTL is active!")
            specialRTL()
 	    vehicle.armed = True
        time.sleep(1)
	set_velocity_body(vehicle, 0, 0, 0)
            
    else: #-- non standard keys
        if event.keysym == 'Up':
            set_velocity_body(vehicle, gnd_speed, 0, 0)
        elif event.keysym == 'Down':
            set_velocity_body(vehicle,-gnd_speed, 0, 0)
        elif event.keysym == 'Left':
            set_velocity_body(vehicle, 0, -gnd_speed, 0)
        elif event.keysym == 'Right':
            set_velocity_body(vehicle, 0, gnd_speed, 0)
        time.sleep(1)
        set_velocity_body(vehicle, 0, 0, 0)

    
    
#---- MAIN FUNCTION
#- Takeoff

#- Read the keyboard with tkinter
root = tk.Tk()
print(">> Control the drone with the arrow keys. Press r for RTL mode")
root.bind_all('<Key>', key)
root.mainloop()
