import serial

def send_hexadecimal_value(serial_port, value):
    try:
        # Open the serial connection
        ser = serial.Serial(serial_port, baudrate=230400, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1)
        
        # Convert the value to bytes (assuming it is a hexadecimal string)
        #value_bytes = bytes.fromhex(value)
        
        #command_with_newline = value_bytes + b'\n'

        # Send the bytes over the serial connection
        ser.write(value)

         # Close the serial connection
        ser.close()
        #print("Successfully sent hexadecimal value:", value)
    except serial.SerialException as e:
        print("Error:", e)
        
def send_hexadecimal_value_return(serial_port, value):
    try:
        # Open the serial connection
        ser = serial.Serial(serial_port, baudrate=230400, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE, timeout=1)
        
        # Convert the value to bytes (assuming it is a hexadecimal string)
        #value_bytes = bytes.fromhex(value)
        
        #command_with_newline = value_bytes + b'\n'

        # Send the bytes over the serial connection
        ser.write(value)

        response = ser.read(ser.in_waiting)  # Read all available bytes from the input buffer
        print("Response from the device: ",  response)

        # Close the serial connection
        ser.close()
        print("Successfully sent hexadecimal value:", value)
    except serial.SerialException as e:
        print("Error:", e)

# Example usage
if __name__ == "__main__":
    # Replace '/dev/ttyUSB0' with the actual serial port of your device
    serial_port = '/dev/serial/by-path/platform-fc800000.usb-usb-0:1.3:1.0-port0'
    #serial_port = '/dev/ttyUSB1'
    
    hexadecimal_value = b'\xA5\x0B\n'
    send_hexadecimal_value(serial_port, hexadecimal_value)
    send_hexadecimal_value(serial_port, hexadecimal_value)
    send_hexadecimal_value(serial_port, hexadecimal_value)
    send_hexadecimal_value(serial_port, hexadecimal_value)
    send_hexadecimal_value(serial_port, hexadecimal_value)

    #hexadecimal_value = 'A50D'
    hexadecimal_value = b'\xA5\x0D\n'
    send_hexadecimal_value_return(serial_port, hexadecimal_value)


    #hexadecimal_value = 'A50B'
    hexadecimal_value = b'\xA5\x0B\n'
    serial_port = '/dev/serial/by-path/platform-fc800000.usb-usb-0:1.4:1.0-port0'
    #serial_port = '/dev/ttyUSB2'
    send_hexadecimal_value(serial_port, hexadecimal_value)
    send_hexadecimal_value(serial_port, hexadecimal_value)
    send_hexadecimal_value(serial_port, hexadecimal_value)
    send_hexadecimal_value(serial_port, hexadecimal_value)
    send_hexadecimal_value(serial_port, hexadecimal_value)

    hexadecimal_value = b'\xA5\x0D\n'
    send_hexadecimal_value_return(serial_port, hexadecimal_value)