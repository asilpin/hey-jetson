import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation 
import serial
from threading import Thread

uart_mcu = serial.Serial('/dev/ttyACM0', 115200)

N = 10 # frequency bins

start_byte_received = False
data_buffer=np.zeros(N)

plt.figure()

position = np.arange(N) + 0.5
plt.xticks(position, ('31 Hz', '63 Hz', '125 Hz', '250 Hz', '500 Hz', '1 kHz', '2.2 kHz', '4.5 kHz', '9 kHz', '15 kHz'))

plt.xlabel('Frequencies', color = '#072b57')
plt.ylabel('Amplitude (dB)', color = '#072b57')
plt.title('FFT Spectrum of the Microphone Data', color = '#072b57')

plt.ylim((0,100))
plt.xlim((0,N))

plt.grid(True)

def plotBarUART():
    global start_byte_received
    global data_buffer
    while True:
        current_byte = uart_mcu.read()  
        # print(current_byte)  
        if not current_byte:
            continue
        if current_byte == b'\xff':
            start_byte_received = True
            data_buffer = []
        elif current_byte == b'\xdd':
            start_byte_received = False
            print(data_buffer)
            plt.cla()
            plt.bar(position, data_buffer, align = 'center', color = '#b8ff5c')
        elif start_byte_received:
            data_buffer.append(int.from_bytes(current_byte))
    

t1 = Thread(target=plotBarUART, daemon=True)
t1.start()
plt.show()