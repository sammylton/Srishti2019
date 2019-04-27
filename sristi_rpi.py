from __future__ import print_function
from dronekit import VehicleMode, connect, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import numpy as np
from pyquaternion import Quaternion
import argparse
import sys
from datetime import datetime, timedelta


from http.server import BaseHTTPRequestHandler, HTTPServer
import os


myrequestsk = None
myrequestemp = None
mystatus = None



#--------------------------SET UP CONNECTION TO VEHICLE----------------------------------

# Parse the arguments  
parser = argparse.ArgumentParser(description='Commands vehicle using vehicle.simple_goto.')
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect

# Connect to the physical UAV or to the simulator on the network
if not connection_string:
    print ('Connecting to pixhawk.')
    vehicle = connect('/dev/serial0', baud=57600, wait_ready= True)
else:
    print ('Connecting to vehicle on: %s' % connection_string)
    vehicle = connect(connection_string, wait_ready=True)




#--------------------------FUNCTION DEFINITION FOR SET_ATTITUDE MESSAGE --------------------

def set_attitude (pitch, roll, yaw, thrust):
    # The parameters are passed in degrees
    # Convert degrees to radians
    #degrees = (2*np.pi)/360
    yaw = np.radians(yaw)
    pitch = np.radians(pitch)
    roll = np.radians(roll) 
    
    # Now calculate the quaternion in preparation to command the change in attitude
    # q for yaw is rotation about z axis
    qyaw = Quaternion (axis = [0, 0, 1], angle = yaw )
    qpitch = Quaternion (axis = [0, 1, 0], angle = pitch )
    qroll = Quaternion (axis = [1, 0, 0], angle = roll )

    # We have components, now to combine them into one quaternion
    q = qyaw * qpitch * qroll
    
    a = q.elements
    
    rollRate = (roll * 5)
    yawRate = (yaw * 0.5)
    pitchRate = abs(pitch * 1)
    # print " Yaw: ",yaw, " Yaw Rate: ", yawRate, " Roll: ",roll, "Roll Rate: ", rollRate, " Pitch: ", pitch, " Thrust: ",thrust
   
    msg = vehicle.message_factory.set_attitude_target_encode(
    0,
    0,                #target system
    0,                #target component
    0b0000000,       #type mask
    [a[0],a[1],a[2],a[3]],        #q
    rollRate,                #body roll rate
    pitchRate,                #body pitch rate
    yawRate,                #body yaw rate
    thrust)                #thrust
    
    vehicle.send_mavlink(msg)

#--------------------------------------------------------------------------------

class RequestHandler_httpd(BaseHTTPRequestHandler):
  def do_GET(self):
    global myrequestsk
    g='hi'
    print('you got this requests')
    myrequestsk = self.requestline
    print('cleaned request')
    myrequestsk = myrequestsk[5 : int(len(myrequestsk) - 9)]
    sensor=myrequestsk.split(',')
    #for a in sensor:
    #   print(a)
    p=int(sensor[0])
    print(type(p))
    r=int(sensor[1])
    print(type(r))

   # print(p)
   # print(r)
    #r=sensor[2]
    if vehicle.mode=="GUIDED_NOGPS" :
      print(p)
      print(r)
      set_attitude(p,r,0,0.5)
    print('P R updated')  
    #messagetosend=bytes(g,"utf")
    self.send_response(200)
    self.send_header('Content-Type', 'text/plain')
    #self.send_header('Content-Length', len(messagetosend))
    self.end_headers()
    #self.wfile.write(messagetosend)
    return



# MAIN PROGRAM ********************************************************************





vstate = "tracking" # Set the vehicle state to tracking in the finite state ma


	
server_address_httpd = ('192.168.137.123',8080)
#change this to your pc ip adress (cmd windows -ifconfig,terminal ubuntu -ipconfig)
httpd = HTTPServer(server_address_httpd, RequestHandler_httpd)
print('starting the server ')
httpd.serve_forever()


vstate = "tracking"

#Close vehicle object before exiting script
print ("Close vehicle object")
vehicle.close()
