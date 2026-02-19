import csv
import sys

import matplotlib.pyplot as plt


def load_csv(path):
    with open(path, newline="") as csv_file:
        reader = csv.DictReader(csv_file, delimiter=";")
        rows = list(reader)

    time_ms = [float(row["Time(ms)"]) for row in rows]
    position = [int(row["Position"]) for row in rows]
    speed = [float(row["Speed"]) for row in rows]

    return time_ms, position, speed


def plot_csv(path):
    time_ms, position, speed = load_csv(path)

    fig, ax1 = plt.subplots(figsize=(10, 6))
    
    # Plot speed on left y-axis
    color_speed = 'tab:orange'
    ax1.set_xlabel("Time (ms)")
    ax1.set_ylabel("Speed (steps/second)", color=color_speed)
    line1 = ax1.plot(time_ms, speed, color=color_speed, label="Speed")
    ax1.tick_params(axis='y', labelcolor=color_speed)
    ax1.grid(True, alpha=0.3)
    
    # Create right y-axis for position
    ax2 = ax1.twinx()
    color_pos = 'tab:blue'
    ax2.set_ylabel("Position (steps)", color=color_pos)
    line2 = ax2.plot(time_ms, position, color=color_pos, label="Position")
    ax2.tick_params(axis='y', labelcolor=color_pos)
    
    # Add title and combine legends
    fig.suptitle(path)
    lines = line1 + line2
    labels = [l.get_label() for l in lines]
    ax1.legend(lines, labels, loc='upper left')
    
    fig.tight_layout()

    return fig


def main(argv):
   
    plot_csv("./test/motion_full.csv")

    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
