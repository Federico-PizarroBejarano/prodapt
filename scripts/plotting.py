import pickle

import matplotlib.pyplot as plt
import numpy as np

plot = True
save_figs = False

models = [
    "cube_ours",
    "cube_3",
    "cube_6",
    "cube_20",
    "cube_50",
]
colors = {
    "cube_ours": "cornflowerblue",
    "cube_3": "lightgreen",
    "cube_6": "limegreen",
    "cube_20": "forestgreen",
    "cube_50": "darkgreen",
}

real = True
experiments = "real" if real else "simulation"
setups = ["clear", "wall", "bucket", "J"]


def load_all_models():
    all_results = {}

    for model in models:
        all_results[model] = {"done": {}, "iters": {}, "time": []}
        for setup in setups:
            with open(f"./results/{experiments}/{setup}/{model}.pkl", "rb") as f:
                data = pickle.load(f)
            all_results[model]["done"][setup] = np.array(data["iters"]) < 1000
            all_results[model]["iters"][setup] = np.array(data["iters"])[
                np.array(data["iters"]) < 1000
            ]
            all_results[model]["time"].append(sum(data["diff_times"], []))

        all_results[model]["time"] = sum(all_results[model]["time"], [])

    return all_results


def plot_iters():
    all_results = load_all_models()

    fig = plt.figure(figsize=(16.0, 10.0))
    ax = fig.add_subplot(111)

    ylabel = "Time to Success (s)"
    ax.set_ylabel(ylabel, weight="bold", fontsize=25, labelpad=10)

    width = 0.5 / (len(models) + 1)

    medianprops = dict(linestyle="--", linewidth=2.5, color="black", markersize=2)
    box_plots = {key: {} for key in models}
    for idx, model in enumerate(models):
        position = ((len(models) - 1) / 2.0 - idx) * width
        for idx2, setup in enumerate(setups):
            box_plots[model][setup] = ax.boxplot(
                all_results[model]["iters"][setup] / 10.0,
                positions=[idx2 * 0.5 - position],
                widths=[width],
                patch_artist=True,
                medianprops=medianprops,
            )

        for patch in [box_plots[model][setup]["boxes"][0] for setup in setups]:
            patch.set_facecolor(colors[model])

    plt.xticks(weight="bold", fontsize=10, rotation=30, ha="right")
    ax.set_xticks(np.arange(len(setups)) * 0.5)
    ax.set_xticklabels([key.title() for key in setups])
    fig.tight_layout()

    ax.set_ylim(ymin=0)
    ax.yaxis.grid(True)

    plt.tick_params(axis="x", which="both", bottom=False, top=False)

    if plot is True:
        plt.show()
    if save_figs:
        fig.savefig(f"./results/{experiments}/iters.png", dpi=300)
    plt.close()


def plot_done():
    all_results = load_all_models()

    fig = plt.figure(figsize=(16.0, 10.0))
    ax = fig.add_subplot(111)

    ylabel = "Percentage of Completed Trials"
    ax.set_ylabel(ylabel, weight="bold", fontsize=25, labelpad=10)

    width = 0.5 / (len(models) + 1)

    for idx, model in enumerate(models):
        position = ((len(models) - 1) / 2.0 - idx) * width
        for idx2, setup in enumerate(setups):
            ax.bar(
                height=np.mean(all_results[model]["done"][setup]),
                x=idx2 * 0.5 - position,
                width=width,
                color=colors[model],
            )

    plt.xticks(weight="bold", fontsize=15, rotation=30, ha="right")
    ax.set_xticks(np.arange(len(setups)) * 0.5)
    ax.set_xticklabels([key.title() for key in setups])
    fig.tight_layout()

    ax.set_ylim(ymin=0)
    ax.yaxis.grid(True)

    plt.tick_params(axis="x", which="both", bottom=False, top=False)

    if plot is True:
        plt.show()
    if save_figs:
        fig.savefig(f"./results/{experiments}/done.png", dpi=300)
    plt.close()


def plot_time():
    all_results = load_all_models()

    fig = plt.figure(figsize=(16.0, 10.0))
    ax = fig.add_subplot(111)

    ylabel = "Diffusion Inference Time (s)"
    ax.set_ylabel(ylabel, weight="bold", fontsize=25, labelpad=10)

    medianprops = dict(linestyle="--", linewidth=2.5, color="black")
    for idx, model in enumerate(models):
        box_plot = ax.boxplot(
            all_results[model]["time"],
            positions=[idx],
            widths=[0.8],
            patch_artist=True,
            medianprops=medianprops,
            showfliers=False,
        )

        box_plot["boxes"][0].set_facecolor(colors[model])

    plt.xticks(weight="bold", fontsize=10, rotation=30, ha="right")
    ax.set_xticks(np.arange(len(models)))
    ax.set_xticklabels([key.title() for key in models])
    fig.tight_layout()

    ax.yaxis.grid(True)
    ax.set_ylim(ymin=0)

    plt.tick_params(axis="x", which="both", bottom=False, top=False)

    if plot is True:
        plt.show()
    if save_figs:
        fig.savefig(f"./results/{experiments}/time.png", dpi=300)
    plt.close()


if __name__ == "__main__":
    plot_iters()
    plot_done()
    plot_time()
