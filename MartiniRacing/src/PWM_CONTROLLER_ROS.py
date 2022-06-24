#!/usr/bin/env python3

#####################################################
# INCLUDES (Yannik)
#####################################################
import numpy as np
import time
import struct
import curses
import atexit
import pigpio 
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

#####################################################
# ABKUERZUNGEN UND VARIABLEN (Daniele, Yannik)
#####################################################
pi = pigpio.pi()
Modus = 0
Status = 0
#motorspeed = 0
#servolenkung = 0
# winkel = 0 als funktion von servolenkung schreiben
cmd_vel = Twist()
cmd_vel = [[0,0,0],[0,0,0]]
i = 0
GPIO.setmode(GPIO.BCM)                  
GPIO.setup(25, GPIO.OUT, initial=0)

#####################################################
# ROS (Subscriber to pwm channel, Yannik)
####################################################
def pwm_lesen(data):
    if(Modus == 2):
        #print("cmd_vel bekommen")
        global cmd_vel
        cmd_vel =[[data.linear.x,data.linear.y,data.linear.z],[0,0,data.angular.z]]  
        # print("Geschwindigkeit :")
       # print("v ",cmd_vel[0][0])
        # print("Einlenkung :")
       # print("l ",cmd_vel[1][2])
        if(cmd_vel[1][2] != 0.0 and cmd_vel[0][0] == 0.0):
            cmd_vel[0][0] = -1
            cmd_vel[1][2] *=  -1 
        set_param_ros(cmd_vel)
    else:
        pass

#####################################################
# X-BOX CONTROLLER (Yannik)
#####################################################
infile_path = "/dev/input/js0"
EVENT_SIZE = struct.calcsize("LhBB")
file = open(infile_path, "rb")      #Falls Fehler auf dieser linie, x-box kontroller verbinden 
event = file.read(EVENT_SIZE)

#####################################################
# UMRECHNEN VON KARTESISCHE- AUF POLARKOORDINATEN (Daniele) 
#####################################################
speed_par = 0
winkel_par = 0
vmin_vor_pub = 0.4
vmax_vor_pub = 1
vmin_ruk_pub = -0.1
#cmd_vel = [[-0.1,0,0],[0,0,1]]
def v_polar(wert):
    speed_par = 0
    if(wert[0][0] > 0):
        speed_par = np.sqrt((wert[0][0])**2 + (wert[0][1])**2)
    elif(wert[0][0] < 0):
        speed_par = -np.sqrt((wert[0][0])**2 + (wert[0][1])**2)
    #print(speed_par)
    return speed_par

#def phi(wert):    #wird zur zeit nicht gebraucht 
#    winkel_par = np.arctan2(cmd_vel[0][1],cmd_vel[0][0])
#    return winkel_par

#####################################################
# UMRECHNEN VON PUBLISHER VARIABLEN FUR MOTORENTREIBER (Daniele) 
#####################################################
speed = 0
vmin_vor_pub_pol = np.sqrt(vmin_vor_pub**2 + vmin_vor_pub**2)
vmax_vor_pub_pol = np.sqrt(vmax_vor_pub**2 + vmax_vor_pub**2)
vmin_ruk_pub_pol = -np.sqrt(vmin_ruk_pub**2 + vmin_ruk_pub**2)

def v_rechner(wert):
    if(wert > 0):
        speed = ((99*wert)/(vmax_vor_pub_pol-vmin_vor_pub_pol))+100-((99)/(vmax_vor_pub_pol-vmin_vor_pub_pol))
    elif(wert < 0):
        speed = -100
    elif(wert == 0):
        speed = 0
    # print(speed)
    return speed

#####################################################
# TREIBER MOTOR ROS (Daniele) 
#####################################################
MOTOR = 13
dcs_ros = 0
vmin_vor_ros = 867500
vmax_vor_ros = 866450
vmin_ruk_ros = 935000
vmax_ruk_ros = 936000
boost_ros = vmax_vor_ros

def speed_ros(wert):
    dcs_ros = 0
    if(wert > 100):
        wert = 100
    elif(wert < -100):
        wert = -100
    if(wert > 0):
        dcs_ros = round(vmin_vor_ros-(vmin_vor_ros-vmax_vor_ros)*wert/100)
    elif(wert < 0):
        dcs_ros = round(vmin_ruk_ros+(vmin_ruk_ros-vmax_ruk_ros)*wert/100)
    pi.hardware_PWM(MOTOR, 600, int(dcs_ros))

#####################################################
# TREIBER SERVO ROS (Daniele) 
#####################################################
SERVO = 12
dcl_ros = 0
servo_left = 1050
servo_middle = 1325
servo_right = 1600
lenkung = 0
vorfaktor = -66

def steering_ros(wert):
    dcl_ros = 0
    wert = vorfaktor*wert[1][2]
    if(wert > 100):
        wert = 100
    elif(wert < -100):
        wert = -100
    if(wert > 0):
        dcl_ros = servo_middle+(servo_right-servo_middle)*wert/100
    elif(wert < 0):
        dcl_ros = servo_middle+(servo_middle-servo_left)*wert/100
    pi.set_servo_pulsewidth(SERVO, int(dcl_ros))

#####################################################
# GESCHWIENDIGKEIT UND LENKUNG SETZEN ROS (Daniele) 
#####################################################
def set_param_ros(wert):
    global speed
    speed = v_polar(wert)
    speed = v_rechner(speed)
    speed_ros(speed)
    steering_ros(wert)

#####################################################
# AUTONOMES FAHREN ROS (Daniele) 
######################################################
rospy.init_node('communicator', anonymous=True)#
rospy.Subscriber("cmd_vel", Twist, pwm_lesen)
        
#####################################################
# TREIBER MOTOR X-BOX (Daniele) 
#####################################################
dcs_xbox = 0
vmin_vor_xbox = 867500
vmax_vor_xbox = 864000
vmin_ruk_xbox = 935000
vmax_ruk_xbox = 938000
stop = 900000
boost_xbox = vmax_vor_xbox
v_mapping_xbox = 866400

def speed_vor_xbox(wert):
    dcs_xbox = round(vmin_vor_xbox-(vmin_vor_xbox-boost_xbox)/65534*(wert+32767))
    # print(dcs_xbox)    
    pi.hardware_PWM(MOTOR, 600, dcs_xbox) # 600Hz 90% dutycycle

def speed_ruk_xbox(wert):
    dcs_xbox = round(vmin_ruk_xbox+(vmax_ruk_xbox-vmin_ruk_xbox)/65534*(wert+32767))
    # print(dcs_xbox)
    pi.hardware_PWM(MOTOR, 600, dcs_xbox)
    
#####################################################
# TREIBER SERVO X-BOX (Daniele) 
#####################################################
dcl_xbox = 0
def steering_xbox(wert):
    dcl_xbox = servo_middle + (wert/119)
    if(dcl_xbox > servo_right):
        dcl_xbox = servo_right
    elif(dcl_xbox < servo_left):
        dcl_xbox = servo_left
    print(dcl_xbox)
    pi.set_servo_pulsewidth(SERVO, dcl_xbox)
    
#####################################################
# MAIN (Daniele) 
#####################################################
while True:
        while(i == 0):
            print("Fuer Manuelle Modus Taste X druecken ")      # Modus kann ausgewaehlt werden 
            print("Fuer Autonome Modus Taste Y druecken ")
            i = 1
        (tv_msec,  value, type, number) = struct.unpack("LhBB", event)  #Package von X-BOX Controller wird in Struct gespeichert
        if(type == 1):
            if(number == 3):            # X Gedruckt
                if(value == 1):         # Manuelle Modus (gesteuert mit X-Box Kontroller)
                    Modus = 1
                    print("gesteuert mit X-Box Kontroller")        
            elif(number == 4):          # Y Gedruckt
                if(value == 1):         # Autonome Modus (gesteuert mit PC ueber Ros)
                    Modus = 2 
                    print("gesteuert mit PC ueber Ros")
        event = file.read(EVENT_SIZE)
        if(Modus == 1):
            print("Manuelle Modus")
            Status = 0
            while(Status == 0):
                (tv_msec,  value, type, number) = struct.unpack("LhBB", event)  #Package von X-BOX Controller wird in Struct gespeichert
                print("Warte auf Xbox Befehl")
                if(type == 1):  # Alle Tasten werden geprueft
                    if(number == 0):            # A gedrueckt
                        if(value == 1):         # Moduswechsel (Wechsel auf Autonomes Modus)
                            # print("A pressed" '\r')
                            Status = 1
                            GPIO.output(25, GPIO.HIGH)
                            time.sleep(1)
                            GPIO.output(25, GPIO.LOW)
                    elif(number == 1):          # B Gedruckt
                        if(value == 1):         # Solange B gedrueckt Boost wird aktiviert
                            boost_xbox = 800000
                            # print("B Pressed" '\r')
                        else:
                            boost_xbox = 864000
                            # print("B Released" '\r')
                    elif(number == 3):          # X Gedruckt
                        if(value == 1):         # Programm wird abgebrochen
                            print("X Pressed" '\r')
                            dcs_xbox = stop
                            pi.hardware_PWM(MOTOR, 600, dcs_xbox)
                            Modus = 3
                            break
                    elif(number == 4):          # Y Gedruckt
                        if(value == 1):         # Noch keine Funktion 
                            # print("Y Pressed"'\r')
                            dcs_xbox = v_mapping_xbox
                            pi.hardware_PWM(MOTOR, 600, dcs_xbox)
                        else:
                            # print("Y Released"'\r')
                            dcs_xbox = stop
                            pi.hardware_PWM(MOTOR, 600, dcs_xbox)
                     
                else: #immer type==2, Alle Achsen werden ueberprueft
                    if(number == 0):            # links rechts Achse, linke joystick
                        steering_xbox(value)
                        # if(value<0):
                        #     print("Links Steuerung"'\r')
                        # elif(value>0):
                        #     print("Rechts Steuerung"'\r')
                        # else:
                        #     print("Geradeaus"'\r')
                    elif(number == 5):          # hinten links (Rueckwaerts fahren) 
                        if(value == -32767):    # Motor stoppen
                            dcs_xbox = stop
                            pi.hardware_PWM(MOTOR, 600,  dcs_xbox)
                        else:
                            speed_ruk_xbox(value)
                            # gas = 50+50*value/32767
                            # print('Bremsen :')
                            # print(gas)
                            # print('\r')
                    elif(number == 4):          # hinten rechts (Vorwaerts fahren) 
                        if(value == -32767):    # Motor stoppen 
                            dcs_xbox = stop
                            pi.hardware_PWM(MOTOR, 600,  dcs_xbox)
                        else:
                            speed_vor_xbox(value)
                            # gas = 50+50*value/32767
                            # print('Gas :')
                            # print(gas)
                            # print('\r')
                    elif(number == 6):          # Kalibrierung der Lenkachse
                        if(value < 0):
                            servo_middle -= 1
                            dcl_xbox = servo_middle
                        elif(value > 0):
                            servo_middle += 1
                            dcl_xbox = servo_middle
                    elif(number == 7):    
                        if(value > 0):  # Korrekturen von mapping geschwindigkeit
                            v_mapping_xbox += 100
                            if(v_mapping_xbox > vmin_vor_xbox):
                                v_mapping_xbox = vmin_vor_xbox
                        elif(value < 0):
                            v_mapping_xbox -= 100
                            if(v_mapping_xbox < vmax_vor_xbox):
                                v_mapping_xbox = vmax_vor_xbox
                event = file.read(EVENT_SIZE)
            if(Modus == 3):
                break
            else:
                Modus = 2
        elif(Modus == 2):
            Status = 0
            print("Autonome Modus")
            while(Status == 0):
               # print("Warte auf Xbox Taste A oder Publischer Infos")
                (tv_msec,  value, type, number) = struct.unpack("LhBB", event)  #Package von X-BOX Controller wird in Struct gespeichert
                if(type == 1):  # Alle Tasten werden ueberprueft
                    if(number == 0):        # A Gedrueckt
                        if(value == 1):     # Moduswechsel (Wechsel zu Manuelle Modus)
                            # print("A pressed" '\r')
                            Status = 1
                    elif(number == 3):          # X Gedruckt
                        if(value == 1):         # Programm wird abgebrochen
                            print("X Pressed" '\r')
                            dcs_xbox = stop
                            pi.hardware_PWM(MOTOR, 600, int(dcs_xbox))
                            Modus = 3
                            break
                #event = file.read(EVENT_SIZE)
                time.sleep(0.01)
            if(Modus ==3):
                break
            else:
                dcs_ros = stop
                pi.hardware_PWM(MOTOR, 600, dcs_ros)
                Modus = 1
        elif(Modus == 3):
            break

#  Konsole       
#  sudo chmod 777 /dev/input/js0 
#  catkin_ws/src/MartiniRacing/src
#  python3 PWM_CONTROLLER_ROS.py 

#  sudo chmod 666 /dev/ttyUSB0
#  export ROS_MASTER_URI=http://192.168.198.39:11311
#  roslaunch rplidar_ros rplidar.launch 
 



