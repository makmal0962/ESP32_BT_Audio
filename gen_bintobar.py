import math
import os

Import("env")

FFT_SAMPLES = 1024
NUM_BARS    = 16
SAMPLE_RATE = 44100.0
NYQUIST_BINS = FFT_SAMPLES // 2

f_min   = SAMPLE_RATE / FFT_SAMPLES
f_max   = SAMPLE_RATE / 2.0
log_min = math.log10(f_min)
log_max = math.log10(f_max)

bins = []
for k in range(1, NYQUIST_BINS):
    freq = k * SAMPLE_RATE / FFT_SAMPLES
    t    = (math.log10(freq) - log_min) / (log_max - log_min)
    bar  = max(0, min(NUM_BARS - 1, int(t * NUM_BARS)))
    bins.append(bar)

out_path = os.path.join("src", "bintobar_generated.h")
with open(out_path, "w") as f:
    f.write("#pragma once\n")
    f.write("#include <pgmspace.h>\n\n")
    f.write(f"// auto-generated - do not edit\n")
    f.write(f"// FFT_SAMPLES={FFT_SAMPLES} NUM_BARS={NUM_BARS} SR={SAMPLE_RATE}\n\n")
    f.write(f"const int8_t binToBar[{NYQUIST_BINS - 1}] PROGMEM = {{\n    ")
    f.write(", ".join(str(b) for b in bins))
    f.write("\n};\n")

print(f"Generated {out_path} ({NYQUIST_BINS - 1} entries)")