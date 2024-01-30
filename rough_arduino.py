import serial
import time



# Open the serial port (replace 'your_port' with the actual port name)
ser = serial.Serial('/dev/ttyUSB0', 9600)  # Replace with the correct port name
time.sleep(2)




def action(command):
    try:
        
            if command == 'O' or command == 'C':
                ser.write(command.encode())
    except KeyboardInterrupt:
        pass

  


action('C')

