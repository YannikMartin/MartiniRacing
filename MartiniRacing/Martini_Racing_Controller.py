#!/usr/bin/env python3

#####################################################
# INCLUDES
#####################################################
import numpy as np
import time
import struct
import pigpio 
import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

#####################################################
# ABKUERZUNGEN UND VARIABLEN
#####################################################
pi = pigpio.pi()            # Library um PWM zu setzen
Modus = 0                   # Variable zum Modus bestimmen (Automes oder Manuell)
Status = 0                  # Boolean um zu while schleifen auszufuehren
cmd_vel = Twist()           # cmd_vel wird als Twist "Vektor/Matrix" initialisiert 
cmd_vel = [[0,0,0],[0,0,0]] # cmd_vel wird mit Nullen befüllt
i = 0                       # Boolean um zu while schleifen auszufuehren
GPIO.setmode(GPIO.BCM)      # PIN 25 wird als output gesetzt               
GPIO.setup(25, GPIO.OUT, initial=0)

#####################################################
# ROS (Funktion nimmt Daten von Publisher und speichert diese in cmd_vel ab)
####################################################
def publ_lesen(data):
    if(Modus == 2):     # Reagiert nur falls das Auto sich in Autonome Modus befindet
        global cmd_vel
        cmd_vel =[[data.linear.x,data.linear.y,data.linear.z],[0,0,data.angular.z]]  # Daten werden abgespeichert
        # Wert anpassung fuer den Spezialfall => Auto zu nahe an einem Objekt
        if(cmd_vel[1][2] != 0.0 and cmd_vel[0][0] == 0.0): 
            cmd_vel[0][0] = -1      # Auto faehrt Rueckwaerts
            cmd_vel[1][2] *=  -1    # und Lenkt in die andere Richtung 
        set_param_ros(cmd_vel)      # Funktion um Parameter zu setzen wird aufgerufen 
    else:               # Falls das Auto in manuelle Modus eingestellt ist werden Publisher infos ingnoriert 
        pass

#####################################################
# Subscriber (ROS)
#####################################################
rospy.init_node('communicator', anonymous=True) 	# Ros Node initialisieren
rospy.Subscriber("cmd_vel", Twist, publ_lesen)      	# Name Msgs, Type Msgs, Funktion (Datensortierung, Variablenanpassung))
#####################################################
# X-BOX CONTROLLER
#####################################################
infile_path = "/dev/input/js0"			#controller Registerpfad
EVENT_SIZE = struct.calcsize("LhBB")			#Registersize
file = open(infile_path, "rb")      			#Register öffnen
event = file.read(EVENT_SIZE)				#Register auslesen

#####################################################
# UMRECHNEN VON KARTESISCHE- AUF POLARKOORDINATEN 
#(Wird benoetigt falls Publisher eine X- und eine Y-Komponente der Geschwindigkeit vorgibt)
#####################################################
speed_par = 0       
vmin_vor_pub = 0.4  # Diese Werte muessen uebereinstimmen mit Einstellungen von Publisher    
vmax_vor_pub = 1    # 
vmin_ruk_pub = -0.1 #

def v_polar(wert):  # Berechnet betrag von V
    speed_par = 0
    if(wert[0][0] > 0):
        speed_par = np.sqrt((wert[0][0])**2 + (wert[0][1])**2)
    elif(wert[0][0] < 0):
        speed_par = -np.sqrt((wert[0][0])**2 + (wert[0][1])**2)
    return speed_par

#####################################################
# UMRECHNEN VON PUBLISHER VARIABLEN FUR MOTORENTREIBER 
# Daten von Publisher koennen verschieden Skalierungen haben. Dieser Funktion Normiert V zwischen -100% und 100%
#####################################################
speed = 0
vmin_vor_pub_pol = np.sqrt(vmin_vor_pub**2 + vmin_vor_pub**2)
vmax_vor_pub_pol = np.sqrt(vmax_vor_pub**2 + vmax_vor_pub**2)
vmin_ruk_pub_pol = -np.sqrt(vmin_ruk_pub**2 + vmin_ruk_pub**2)

def v_rechner(wert):
    if(wert > 0):
        speed = ((99*wert)/(vmax_vor_pub_pol-vmin_vor_pub_pol))+100-((99)/(vmax_vor_pub_pol-vmin_vor_pub_pol))
    elif(wert < 0):     # Rueckwaerts wird immer nur mit eine geringere konstante V (Sicherheit)
        speed = -100
    elif(wert == 0):
        speed = 0
    return speed

#####################################################
# TREIBER MOTOR ROS 
# Motorentreiber benoetigt werte zwischen 0 und 10^6 (PWM)
# bei MartiniRacing: PWM = 9*10^5 = Stillstand.  PWM < 9*10^5 = Vorwaerts. PWM > 9*10^5 = Rueckwaerts  
# Die Geschwindigkeiten im Autonomes Modus sind geringer als im Manuelle Modus 
#####################################################
MOTOR = 13              # PIN Anschluss Motor
dcs_ros = 0             # DUTY CICLE (Speed) fuer Steuerung ueber Publisher (ROS)
vmin_vor_ros = 867500   # Anfahrgeschwindigkeit Vorwaerts ROS
vmax_vor_ros = 866450   # Maximale Geschwindigkeit Vorwaerts ROS
vmin_ruk_ros = 935000   # Anfahrgeschwindigkeit Ruckwaerts ROS
vmax_ruk_ros = 936000   # Maximale Geschwindigkeit Ruckwaerts ROS

def speed_ros(wert):
    dcs_ros = 0
    if(wert > 100):     # Schutz (Laesst keine Werte durch die groesser als 100 sind)
        wert = 100
    elif(wert < -100):  # Schutz (Laesst keine Werte durch die kleiner als -100 sind)
        wert = -100
    if(wert > 0):       # Berechneung Geschwiendigkeit Vorwaerts
        dcs_ros = round(vmin_vor_ros-(vmin_vor_ros-vmax_vor_ros)*wert/100)
    elif(wert < 0):     # Berechneung Geschwiendigkeit Rueckwaerts
        dcs_ros = round(vmin_ruk_ros+(vmin_ruk_ros-vmax_ruk_ros)*wert/100)
    pi.hardware_PWM(MOTOR, 600, int(dcs_ros)) # PWM wird gesetzt (PIN = MOTOR, f = 600Hz, gerechnete DUTY CICLE (muss Integer sein))

#####################################################
# TREIBER SERVO ROS
# bei MartiniRacing: PWM = 1325 = Geradeaus  PWM < 1325 = Links einlenken PWM > 1325 = Rechts einlenken 
#####################################################
SERVO = 12          # PIN Anschluss Servo
dcl_ros = 0         # DUTY CICLE (Lenkung) fuer Steuerung ueber Publisher (ROS)
servo_left = 1050   # Maximale Einschalg Links
servo_middle = 1325 # Geradeaus
servo_right = 1600  # Maximale Einschalg Rechts
vorfaktor = 66      # Wird benoetigt um Lenkung zu justieren und uebereinstimmung mit ROS zu gewaehrleisten

def steering_ros(wert):
    dcl_ros = 0
    wert = vorfaktor*wert[1][2]
    if(wert > 100):     # Schutz (Laesst keine Werte durch die groesser als 100 sind)
        wert = 100
    elif(wert < -100):  # Schutz (Laesst keine Werte durch die kleiner als -100 sind)
        wert = -100
    if(wert < 0):       # Berechneung Einlenkung Links
        dcl_ros = servo_middle+(servo_middle-servo_left)*wert/100
    elif(wert > 0):     # Berechneung Einlenkung Rechts
        dcl_ros = servo_middle+(servo_right-servo_middle)*wert/100
    pi.set_servo_pulsewidth(SERVO, int(dcl_ros)) # PWM wird gesetzt (PIN = SERVO, gerechnete DUTY CICLE (muss Integer sein))

#####################################################
# GESCHWIENDIGKEIT UND LENKUNG SETZEN ROS
# Nimmt eine Matrix (cmd_vel) als input und ruft die noetige Funktionen auf 
# Um Geschwindigkeit und Einlenkung zu setzen muss man nur diese Funktion aufrufen und cmd_vel uebergeben
#####################################################
def set_param_ros(wert):
    global speed
    speed = v_polar(wert)
    speed = v_rechner(speed)
    speed_ros(speed)
    steering_ros(wert)
        
#####################################################
# TREIBER MOTOR X-BOX 
# X-Box Kontroller liefert werte zwischen -32767 und 32767
# Motorentreiber benoetigt werte zwischen 0 und 10^6 (PWM)
# bei MartiniRacing: PWM = 9*10^5 = Stillstand.  PWM < 9*10^5 = Vorwaerts. PWM > 9*10^5 = Rueckwaerts 
# Die Geschwindigkeiten im Manuelle Modus sind groesser als im Autonomes Modus 
#####################################################
dcs_xbox = 0            # DUTY CICLE (Speed) fuer Steuerung ueber X-BOX Kontroller
vmin_vor_xbox = 867500  # Anfahrgeschwindigkeit Vorwaerts X-Box
vmax_vor_xbox = 864000  # Maximale Geschwindigkeit Vorwaerts X-Box
vmin_ruk_xbox = 935000  # Anfahrgeschwindigkeit Ruckwaerts X-Box
vmax_ruk_xbox = 938000  # Maximale Geschwindigkeit Ruckwaerts X-Box
stop = 900000
boost_xbox = vmax_vor_xbox  # Erlaubt Bediener jegliche Geschwiendigkeitabgrenzungen (Software) zu ueberwinden (ca. 4 bis 5 Mal Schneller)
v_mapping_xbox = 866400 # Beste Geschwindigkeit um die Umgebung zu mappen

def speed_vor_xbox(wert): # Umrechneungsfunkion Geschwiendigkeit Vorwaerts
    dcs_xbox = round(vmin_vor_xbox-(vmin_vor_xbox-boost_xbox)/65534*(wert+32767))
    pi.hardware_PWM(MOTOR, 600, int(dcs_xbox)) # PWM wird gesetzt (PIN = MOTOR, f = 600Hz, gerechnete DUTY CICLE (muss Integer sein))

def speed_ruk_xbox(wert): # Umrechneungsfunkion Geschwiendigkeit Ruckwaerts
    dcs_xbox = round(vmin_ruk_xbox+(vmax_ruk_xbox-vmin_ruk_xbox)/65534*(wert+32767))
    pi.hardware_PWM(MOTOR, 600, int(dcs_xbox)) # PWM wird gesetzt (PIN = MOTOR, f = 600Hz, gerechnete DUTY CICLE (muss Integer sein))
    
#####################################################
# TREIBER SERVO X-BOX 
# X-Box Kontroller liefert werte zwischen -32767 und 32767
# bei MartiniRacing: PWM = 1325 = Geradeaus  PWM < 1325 = Links einlenken PWM > 1325 = Rechts einlenken 
#####################################################
dcl_xbox = 0
servo_left = 1050   # Maximale Einschalg Links
servo_middle = 1325 # Geradeaus
servo_right = 1600  # Maximale Einschalg Rechts

def steering_xbox(wert):
    dcl_xbox = servo_middle + (wert/(servo_middle-servo_left))    # Umrechnung von DUTY CICLE
    if(dcl_xbox > servo_right):
        dcl_xbox = servo_right
    elif(dcl_xbox < servo_left):
        dcl_xbox = servo_left
    pi.set_servo_pulsewidth(SERVO, int(dcl_xbox)) # PWM wird gesetzt (PIN = SERVO, gerechnete DUTY CICLE (muss Integer sein))
    
#####################################################
# MAIN 
#####################################################
while True:
        while(i == 0):  # Wird immer nur 1 mal ausgefuehrt
            print("Fuer Manuelle Modus Taste X druecken ")              # Modus kann ausgewaehlt werden 
            print("Fuer Autonome Modus Taste Y druecken ")
            i = 1
        (tv_msec,  value, type, number) = struct.unpack("LhBB", event)  # Package von X-BOX abspeichern, Controllerinfo wird in Struct gespeichert
        if(type == 1): # Type = 1 bedeutet dass ein Tasten (nur on oder off) gedrueckt wurde (number bestimmt welches Taste)
            if(number == 3):            # X Gedruckt
                if(value == 1):         # Manuelle Modus (gesteuert mit X-Box Kontroller)
                    Modus = 1
                    print("gesteuert mit X-Box Kontroller")        
            elif(number == 4):          # Y Gedruckt
                if(value == 1):         # Autonome Modus (gesteuert mit PC ueber Ros)
                    Modus = 2 
                    print("gesteuert mit PC ueber Ros")
        event = file.read(EVENT_SIZE)	 #Register auslesen
        
        # Manuelle Modus (roboter wird mit x_box Kontroller gesteuert)        
        if(Modus == 1):
            print("Manuelle Modus")
            Status = 0 
            while(Status == 0): # Solange Status = 0 wird dauernd ueberprueft ob Befehle von x_box Kontroller eintreffen 
                (tv_msec,  value, type, number) = struct.unpack("LhBB", event)  #Package von X-BOX Controller wird in Struct gespeichert
                print("Warte auf Xbox Befehl")
                if(type == 1):  # Alle Tasten werden geprueft
                    if(number == 0):            # A gedrueckt
                        if(value == 1):         # Moduswechsel (Wechsel auf Autonomes Modus)
                            Status = 1
                            GPIO.output(25, GPIO.HIGH) # Beim Moduswechsel wird eine kurze Tonsignal ausgeloest
                            time.sleep(1)
                            GPIO.output(25, GPIO.LOW)
                    elif(number == 1):          # B Gedruckt
                        if(value == 1):         # Solange B gedrueckt Boost wird aktiviert
                            boost_xbox = 800000
                        else:
                            boost_xbox = 864000
                    elif(number == 3):          # X Gedruckt
                        if(value == 1):         # Auto wird gestoppt und das Programm wird komplett abgebrochen
                            print("X Pressed" '\r')
                            dcs_xbox = stop     # Auto wird gestoppt
                            pi.hardware_PWM(MOTOR, 600, int(dcs_xbox)) # PWM wird gesetzt (PIN = MOTOR, f = 600Hz, gerechnete DUTY CICLE (muss Integer sein))
                            Modus = 3
                            break
                    elif(number == 4):          # Y Gedruckt
                        if(value == 1):         # Solange Taste gedruekt Auto faehrt mit optimale Geschwindigkeit um Umgebung zu mappen
                            dcs_xbox = v_mapping_xbox
                            pi.hardware_PWM(MOTOR, 600, int(dcs_xbox)) # PWM wird gesetzt (PIN = MOTOR, f = 600Hz, gerechnete DUTY CICLE (muss Integer sein))
                        else:
                            dcs_xbox = stop     # Auto wird gestoppt
                            pi.hardware_PWM(MOTOR, 600, dcs_xbox) # PWM wird gesetzt (PIN = MOTOR, f = 600Hz, DUTY CICLE = 9*10^5)
                     
                else: # Type = 2 bedeutet dass eine Achse (wert zwischen -32767 und 32767) gedrueckt wurde (number bestimmt welche Achse)
                    if(number == 0):            # linke Joystick 
                        steering_xbox(value)    # Lenkung wird angepasst
                    elif(number == 5):          # hinten links (Rueckwaerts fahren) 
                        if(value == -32767):    # Motor stoppen falls dieser Wert eintrifft (Taste ist Losgelassen)
                            dcs_xbox = stop     # Auto wird gestoppt
                            pi.hardware_PWM(MOTOR, 600, dcs_xbox) # PWM wird gesetzt (PIN = MOTOR, f = 600Hz, DUTY CICLE = 9*10^5)
                        else:
                            speed_ruk_xbox(value)
                    elif(number == 4):          # hinten rechts (Vorwaerts fahren) 
                        if(value == -32767):    # Motor stoppen 
                            dcs_xbox = stop     # Auto wird gestoppt
                            pi.hardware_PWM(MOTOR, 600,  dcs_xbox) # PWM wird gesetzt (PIN = MOTOR, f = 600Hz, DUTY CICLE = 9*10^5)
                        else:
                            speed_vor_xbox(value) # Geschwindigkeit wird gesetzt
                    elif(number == 6):          # Kalibrierung der Lenkachse 
                        if(value < 0):
                            servo_middle -= 1   # Kleine Korrektur nach Links
                            dcl_xbox = servo_middle
                        elif(value > 0):
                            servo_middle += 1   # Kleine Korrektur nach Rechts
                            dcl_xbox = servo_middle
                    elif(number == 7):    
                        if(value > 0):  # Korrekturen von mapping geschwindigkeit
                            v_mapping_xbox += 100  # Auto wird ein Wenig langsamer fahren
                            if(v_mapping_xbox > vmin_vor_xbox): #(Schutz)
                                v_mapping_xbox = vmin_vor_xbox
                        elif(value < 0):
                            v_mapping_xbox -= 100 # Auto wird ein Wenig schneller fahren
                            if(v_mapping_xbox < vmax_vor_xbox): #(Schutz)
                                v_mapping_xbox = vmax_vor_xbox
                event = file.read(EVENT_SIZE)
            if(Modus == 3):
                break
            else:
                Modus = 2
        # Autonome Modus (roboter wird mit ROS gesteuert sobald ein msgs eintrifft, wird ein Interrupt ausgeloest und die Werte werden automatisch angepasst)  
        elif(Modus == 2):
            Status = 0
            print("Autonome Modus")
            while(Status == 0):  # Solange Status = 0 wird dauernd ueberprueft ob Befehle von ROS oder x_box Kontroller eintreffen 
                (tv_msec,  value, type, number) = struct.unpack("LhBB", event)  #Package von X-BOX Controller wird in Struct gespeichert
                if(type == 1):  # Alle Tasten werden ueberprueft (Reagiert jedoch nur auf A und X)
                    if(number == 0):        # A Gedrueckt
                        if(value == 1):     # Moduswechsel (Wechsel zu Manuelle Modus)
                            # print("A pressed" '\r')
                            Status = 1
                    elif(number == 3):          # X Gedruckt
                        if(value == 1):         # Auto wird gestopt und das Programm wird komplett abgebrochen
                            print("X Pressed" '\r')
                            dcs_xbox = stop     # Auto wird gestoppt
                            pi.hardware_PWM(MOTOR, 600, dcs_xbox) # PWM wird gesetzt (PIN = MOTOR, f = 600Hz, DUTY CICLE = 9*10^5)
                            Modus = 3
                            break
                #event = file.read(EVENT_SIZE)
                time.sleep(0.01)
            if(Modus ==3):      # Programm wird komplett abgebrochen
                break
            else:
                dcs_ros = stop  # Auto wird gestoppt
                pi.hardware_PWM(MOTOR, 600, dcs_ros) # PWM wird gesetzt (PIN = MOTOR, f = 600Hz, DUTY CICLE = 9*10^5)
                Modus = 1
        elif(Modus == 3): # Programm wird komplett abgebrochen
            break
 



