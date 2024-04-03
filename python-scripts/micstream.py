import serial
import struct
import numpy as np
import sounddevice as sd
from scipy.io import wavfile

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

def convert_int16(original_bytes):
    int16_list = []
    for i in range(0, len(original_bytes) - 2, 2):
        combined_bytes = original_bytes[i+1:i+2] + original_bytes[i:i+1]
        int16_value = struct.unpack('<h', combined_bytes)[0]
        int16_list.append(int16_value)
    return np.array(int16_list)

try:
    data_list = []
    while True:
        peek = ser.read()
        if peek == b'\x01':
            signature = ser.read()
            if signature == b'\x02':
                print(peek + signature)
                byte_data = ser.read_until(b'\x03\x04')
                
                print(byte_data[-2:])
                print(byte_data[:-2])
                print(len(byte_data[:-2]))
                
                if len(byte_data[:-2]) != 640:
                    print("ERROR: corrupted frame")
                    continue

                converted_data = convert_int16(byte_data)
                data_list.append(converted_data)

                print(converted_data)
                print(len(convert_int16(byte_data)))

except KeyboardInterrupt:
    print("\nKeyboard interrupt received. Exiting...")
    if data_list:  
        data_array = np.vstack(data_list)
        np.save('raw_i2s.npy', data_array) 
        print("Data saved to 'raw_i2s.npy'")    
        