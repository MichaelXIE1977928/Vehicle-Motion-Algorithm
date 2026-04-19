"""Save and plot simulation results."""
import csv
import matplotlib.pyplot as plt


def save_csv(results: dict, path):
    t = results["time"]
    states = results["states"]
    names = results["state_names"]
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["time"] + names)
        for i, time in enumerate(t):
            writer.writerow([time] + list(states[i]))


def plot_results(results: dict):
    t = results["time"]
    states = results["states"]
    names = results["state_names"]
    n_states = states.shape[1]
    fig, axes = plt.subplots(n_states, 1, sharex=True, figsize=(8, 2*n_states))
    if n_states == 1:
        axes = [axes]
    for i, name in enumerate(names):
        axes[i].plot(t, states[:, i])
        axes[i].set_ylabel(name)
        axes[i].grid(True)
    axes[-1].set_xlabel("Time (s)")
    plt.tight_layout()
    plt.show()
