import sounddevice as sd
import numpy as np

print(sd.query_devices())
sample_rate = 16000
duration = 5
print("Recording for 5 seconds, say something...")
recording = sd.rec(int(duration * sample_rate), samplerate=sample_rate, channels=1, dtype='float32', device=1)
sd.wait()
print(f"Max volume: {np.max(np.abs(recording))}")
print("Done")