import numpy as np
from scipy.signal import chirp, windows
from scipy.io.wavfile import write

# Parameters
sample_rate = 16000  # 16kHz
fft_size = 1024      # Matches ESP-DSP FFT size
duration = fft_size / sample_rate  # Duration in seconds
f_start = 300        # Start frequency of sweep (Hz)
f_end = 7000         # End frequency of sweep (Hz)

# Time vector
t = np.linspace(0, duration, fft_size, endpoint=False)

# Generate logarithmic sweep (chirp)
sweep = chirp(t, f0=f_start, f1=f_end, t1=duration, method='logarithmic')

# Apply window to reduce edge artifacts
window = windows.hann(fft_size)
sweep *= window

# Normalize to -1.0 to 1.0 range
sweep /= np.max(np.abs(sweep))

# Optional: Save to WAV file
write("sweep_1024.wav", sample_rate, (sweep * 32767).astype(np.int16))

