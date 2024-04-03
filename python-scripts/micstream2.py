import serial
import struct
import numpy as np
import sounddevice as sd
from scipy.io import wavfile

ser = serial.Serial('/dev/ttyACM0', 460800, timeout=1)
bytes_to_read = 100000
possible_error = False

def convert_int16(original_bytes):
    int16_list = []
    global possible_error

    for i in range(0, len(original_bytes), 2):
        int16_value = (original_bytes[i] << 8) | original_bytes[i + 1]
        if int16_value & 0x8000:  
            int16_value -= 0x10000
        if int16_value < -10000 or int16_value > 1000:
            possible_error = True            
        print(int16_value)
        int16_list.append(int16_value)
    
    return np.array(int16_list)

if __name__ == '__main__':
    raw_byte_array = bytearray()
    while True:
        peek = ser.read()
        if peek == b'\xf9':
            break
    peek = ser.read()
    while len(raw_byte_array) < bytes_to_read:
        raw_byte_array.append(ord(ser.read(1)))
            
    print(raw_byte_array)
    int_array = convert_int16(raw_byte_array)

    # print(int_array)
    print("byte array length:" + str(len(raw_byte_array)))
    print("converted int length:" + str(len(int_array)))
    if(possible_error):
        print("SUMTING MIGHT BE WONG")
    np.save('raw_i2s.npy', int_array) 
    print("Data saved to 'raw_i2s.npy'")    
        