#!/usr/bin/env python3
import rospy
from threading import Thread, Timer
import serial
import time

# Puertos Raspberry Pi:
LINE_PORT = '/dev/ttyACM0'
DISP_PORT = '/dev/ttyACM1'
BAUD = 9600

rospy.init_node('line_and_disp_manager', anonymous=True)

ser_line = serial.Serial(LINE_PORT, BAUD, timeout=0.1)
ser_disp = serial.Serial(DISP_PORT, BAUD, timeout=0.1)  # Corregido: debe ser DISP_PORT

state = 'RUN'
disp_timer = None
patient_timer = None

def send_go():
    global state
    ser_line.write(b"go\n")
    rospy.loginfo(">> go")
    state = 'RUN'

def start_disp_timer():
    global state, disp_timer
    rospy.loginfo("Start 10s dispenser wait")
    disp_timer = Timer(10.0, send_go)
    disp_timer.start()
    state = 'WAIT_DISP'

def start_patient_timer():
    global state, patient_timer
    rospy.loginfo("Start 20s patient wait")
    patient_timer = Timer(20.0, send_go)
    patient_timer.start()
    state = 'WAIT_PATIENT'

def read_line_messages():
    """Leer mensajes del seguidor de línea."""
    global state
    while not rospy.is_shutdown():
        try:
            line_msg = ser_line.readline().decode().strip()
            if line_msg == 'stop' and state == 'RUN':
                rospy.loginfo("<< stop (line follower)")
                start_disp_timer()
        except serial.SerialException as e:
            rospy.logwarn("Error leyendo del seguidor de línea: %s", e)

def read_disp_messages():
    """Leer mensajes del dispensador."""
    global state
    while not rospy.is_shutdown():
        try:
            disp_msg = ser_disp.readline().decode().strip()
            if disp_msg == 'dispensed' and state == 'WAIT_DISP':
                rospy.loginfo("<< dispensed")
                disp_timer.cancel()
                start_patient_timer()
        except serial.SerialException as e:
            rospy.logwarn("Error leyendo del dispensador: %s", e)

# Al arrancar, damos "go" para que el seguidor empiece
send_go()

# Iniciar los hilos de lectura para line follower y dispensador
line_thread = Thread(target=read_line_messages)
disp_thread = Thread(target=read_disp_messages)

line_thread.start()
disp_thread.start()

# Mantener el programa principal corriendo mientras los hilos hacen el trabajo
try:
    while not rospy.is_shutdown():
        rospy.sleep(1)  # Esto mantiene al nodo activo mientras los hilos leen los puertos
except rospy.ROSInterruptException:
    pass