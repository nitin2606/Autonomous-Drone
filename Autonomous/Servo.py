import serial
import time
from dronekit import connect


# Open the serial port (replace 'your_port' with the actual port name)
ser = serial.Serial('/dev/ttyUSB0', 9600)  # Replace with the correct port name
time.sleep(2)

#vehicle = connect('127.0.0.1:14550')
vehicle = connect('/dev/ttyACM0', wait_ready=True)


def action(command):
    try:
        
            if command == 'O' or command == 'C':
                ser.write(command.encode())
    except KeyboardInterrupt:
        pass


print("Connected")

@vehicle.on_message('RC_CHANNELS')
def RC_CHANNEL_listener(vehicle, name, message):

    set_rc(1, message.chan1_raw)
    set_rc(2, message.chan2_raw)
    set_rc(3, message.chan3_raw)
    set_rc(4, message.chan4_raw)
    set_rc(5, message.chan5_raw)
    set_rc(6, message.chan6_raw)
    set_rc(7, message.chan7_raw)
    set_rc(8, message.chan8_raw)
    vehicle.notify_attribute_listeners('channels', vehicle.channels)


def set_rc(chnum, v):

    vehicle._channels._update_channel(str(chnum), v)


while True:
    print(" Ch7 override: %s" % vehicle.channels[7])
    print(type(vehicle.channels[7]))

    try:


        if vehicle.channels[7] >=1600:
            print("OPEN")
            action('O')

        
        elif vehicle.channels[7] <=1400:
            action('C')
    
    except:
        pass

vehicle.close()






