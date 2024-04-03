import sounddevice as sd
import numpy as np
from scipy.io import wavfile
import threading
import time

class AudioRecorder:
    def __init__(self, filename, sample_rate=8000):
        self.filename = filename
        self.sample_rate = sample_rate
        self.should_record = False
        self.thread = threading.Thread(target=self._record_thread)

    def _record_thread(self):
        while self.should_record:
            print("Recording...")
            audio = sd.rec(10 * self.sample_rate, samplerate=self.sample_rate, channels=1, dtype='int16')
            sd.wait()
            print("Recording complete.")

            # Save audio to WAV file
            wavfile.write(self.filename, self.sample_rate, audio)

    def start_recording(self):
        self.should_record = True
        self.thread.start()

    def stop_recording(self):
        self.should_record = False
        self.thread.join()

if __name__ == "__main__":

    output_file = "recorded_audio.wav"
    recorder = AudioRecorder(output_file)

    try:
        recorder.start_recording()
        print("Recording started. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)  # Keep the program running
    except KeyboardInterrupt:
        print("Recording stopped.")
        recorder.stop_recording()
