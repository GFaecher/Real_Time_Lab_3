#!/usr/bin/env python3

import sys
import numpy as np
import matplotlib.pyplot as plt

# --------------------------------------------------------------
# Globals
# --------------------------------------------------------------

headers = []
data_cols = {}
samples = None

# --------------------------------------------------------------
# Parse input file
# --------------------------------------------------------------

def parse_inputfile(fname):
    global headers, data_cols, samples

    with open(fname, "r") as f:
        lines = f.readlines()

    headers = []
    data_cols = {}

    for line in lines:
        line = line.strip()
        if line == "":
            continue

        # detect header
        if line.lower().startswith("sample"):
            headers = line.split()
            for h in headers:
                data_cols[h] = []
            continue

        # data line
        fields = line.split()
        if len(fields) != len(headers):
            continue

        # convert to float
        try:
            nums = [float(f) for f in fields]
        except ValueError:
            continue

        for h, val in zip(headers, nums):
            data_cols[h].append(val)

    # convert lists to numpy arrays
    for h in headers:
        data_cols[h] = np.array(data_cols[h])

    samples = data_cols[headers[0]]

# --------------------------------------------------------------
# Plotting
# --------------------------------------------------------------

def plot_signal(name):
    if name not in headers:
        print(f"Error: '{name}' not in columns:")
        print(headers)
        return

    x = samples
    y = data_cols[name]

    plt.figure(figsize=(8, 4))
    plt.plot(x, y)
    plt.title(name)
    plt.xlabel("sample")
    plt.ylabel(name)
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# --------------------------------------------------------------
# Main
# --------------------------------------------------------------

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: lab7_host_plot.py <file>")
        sys.exit(1)

    fname = sys.argv[1]
    parse_inputfile(fname)

    # Example plots. Adjust as needed.
    plot_signal("notch60")
    plot_signal("hp_5Hz")
    plot_signal("ttm")
