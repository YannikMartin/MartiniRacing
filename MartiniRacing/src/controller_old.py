#!/usr/bin/env python

#####################################################
# INCLUDES (Yannik)
#####################################################
import time
import struct
import curses
import atexit
import pigpio 
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3

#####################################################
# ABKUERZUNGEN UND VARIABLEN (Daniele, Yannik)
#####################################################
pi = pigpio.pi()
Modus = 0
Status = 0
motorspeed = 0
servolenkung = 0
# winkel = 0 als funktion von servolenkung schreiben

#####################################################
# ROS (Subscriber to pwm channel, Yannik)
#####################################################
def communicate():
    rospy.init_node('communicator', anonymous=True)
    rate = rospy.Rate(0.1) # 10hz
    while not rospy.is_shutdown():
        
        talk()
        listen()
        rate.sleep()

def talk():
    pub = rospy.Publisher('pwm', Vector3, queue_size=1)
    vec = Vector3(1,2,0)
    s = String()
    s.data = "asdjasdbjb"
    rospy.loginfo(vec)
    pub.publish(vec)

def pwm_lesen(data):
    motorspeed = data.x #-100 voll rueckwaerts bis 100 voll forwaerts
    servo_lenkung = data.y #-100 links, 100 rechts
    winkel = data.z #winkel der einzuschlagen ist
    
def listen():
    rospy.Subscriber("pwm", Vector3, pwm_lesen)

#####################################################
# X-BOX CONTROLLER (Yannik)
#####################################################
infile_path = "/dev/input/js0"
EVENT_SIZE = struct.calcsize("LhBB")
file = open(infile_path, "rb")      #Falls Fehler auf dieser linie, x-box kontroller verbinden 
event = file.read(EVENT_SIZE)

#####################################################
# TREIBER MOTOR ROS (Daniele) 
#####################################################
MOTOR = 13
dcs_ros = 0
vmin_vor_ros = 867500
vmax_vor_ros = 864000
vmin_ruk_ros = 935000
vmax_ruk_ros = 938000
boost_ros = vmax_vor_ros

def speed_ros(wert):
    if(wert > 100):
        wert = 100
    elif(wert < -100):
        wert = -100
    if(wert > 0):
        dcs_ros = round(vmin_vor_ros-(vmin_vor_ros-vmax_vor_ros)*wert/100)
    elif(wert < 0):
        dcs_ros = round(vmin_ruk_ros+(vmax_ruk_ros-vmin_ruk_ros)*wert/100)
    pi.hardware_PWM(MOTOR, 600, dcs_ros)

#####################################################
# TREIBER SERVO ROS (Daniele) 
#####################################################
SERVO = 12
dcl_ros = 0
servo_left = 1050
servo_middle = 1325
servo_right = 1600

def steering_ros(wert):
    if(wert > 100):
        wert = 100
    elif(wert < -100):
        wert = -100
    if(wert > 0):
        dcl_ros = servo_middle+(servo_right-servo_middle)*wert/100
    elif(wert < 0):
        dcl_ros = servo_middle-(servo_middle-servo_left)*wert/100
    pi.set_servo_pulsewidth(SERVO, dcl_ros)

#####################################################
# AUTONOMES FAHREN ROS (Daniele) 
#####################################################
def autonomesfahren():
    # while not rospy.is_shutdown():
    #     listen()
    #     rate.sleep()
        steering_ros(servolenkung)
        speed_ros(motorspeed)

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
    #print(dcs_xbox)    
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
#if name == 'main':
    #communicate()

while True:
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
                if(type == 1):  # Alle Tasten werden 
                    if(number == 0):            # A gedrueckt
                        if(value == 1):         # Moduswechsel (Wechsel auf Autonomes Modus)
                           print("A pressed" '\r')
                           Status = 1
                    elif(number == 1):          # B Gedruckt
                        if(value == 1):         # Solange B gedrueckt Boost wird aktiviert
                            boost_xbox = 800000
                            print("B Pressed" '\r')
                        else:
                            boost_xbox = 864000
                            print("B Released" '\r')
                    elif(number == 3):          # X Gedruckt
                        if(value == 1):         # Programm wird abgebrochen
                            print("X Pressed" '\r')
                            dcs_xbox = stop
                            pi.hardware_PWM(MOTOR, 600, dcs_xbox)
                            Modus = 3
                            break
                    elif(number == 4):          # Y Gedruckt
                        if(value == 1):         # Noch keine Funktion 
                            print("Y Pressed"'\r')
                            print(v_mapping_xbox)
                            dcs_xbox = v_mapping_xbox
                            pi.hardware_PWM(MOTOR, 600, dcs_xbox)
                        else:
                            print("Y Released"'\r')
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
                print("Warte auf Xbox Taste A oder Publischer Infos")
                (tv_msec,  value, type, number) = struct.unpack("LhBB", event)  #Package von X-BOX Controller wird in Struct gespeichert
                if(type == 1):  # Alle Tasten werden ueberprueft
                    if(number == 0):        # A Gedrueckt
                        if(value == 1):     # Moduswechsel (Wechsel zu Manuelle Modus)
                           print("A pressed" '\r')
                           Status = 1
                autonomesfahren()
                event = file.read(EVENT_SIZE)
            Modus = 1
        elif(Modus == 3):
            break