import pickle

import matplotlib.pyplot as plt

import numpy as np

import tikzplotlib
from matplotlib.lines import Line2D
from matplotlib.legend import Legend

Line2D._us_dashSeq = property(lambda self: self._dash_pattern[1])
Line2D._us_dashOffset = property(lambda self: self._dash_pattern[0])
Legend._ncol = property(lambda self: self._ncols)

plot = True
save_figs = False

baseline = "baseline_med"
config = "medium"
diff_iters = 100

models = [
    f"{config}_{diff_iters}_3_10",
    f"{baseline}_{diff_iters}_3",
    f"{baseline}_{diff_iters}_6",
    f"{baseline}_{diff_iters}_20",
    f"{baseline}_{diff_iters}_50",
]
colors = {
    f"{config}_{diff_iters}_3_10": "cornflowerblue",
    f"{baseline}_{diff_iters}_3": "lightgreen",
    f"{baseline}_{diff_iters}_6": "limegreen",
    f"{baseline}_{diff_iters}_20": "forestgreen",
    f"{baseline}_{diff_iters}_50": "darkgreen",
}

results = "results8"


def load_all_models():
    all_results = {}

    for model in models:
        with open(f"./{results}/{model}.pkl", "rb") as f:
            all_results[model] = pickle.load(f)

    return all_results


def plot_iters():
    all_results = load_all_models()

    fig = plt.figure(figsize=(16.0, 10.0))
    ax = fig.add_subplot(111)

    data = {}

    for model in models:
        exp_data = all_results[model]
        exp_done = {
            "clear": exp_data["done"][:10],
            "cube": exp_data["done"][10:20],
            "wall": exp_data["done"][20:30],
            "L_right": exp_data["done"][30:40],
            # 'L_left': exp_data['done'][40:50],
            "bucket": exp_data["done"][50:],
        }
        exp_iters = {
            "clear": np.array(exp_data["iters"][:10])[exp_done["clear"]],
            "cube": np.array(exp_data["iters"][10:20])[exp_done["cube"]],
            "wall": np.array(exp_data["iters"][20:30])[exp_done["wall"]],
            "L_right": np.array(exp_data["iters"][30:40])[exp_done["wall"]],
            # 'L_left': np.array(exp_data['iters'][40:50])[exp_done['wall']],
            "bucket": np.array(exp_data["iters"][50:])[exp_done["bucket"]],
        }
        data[model] = exp_iters

    ylabel = "Number of Iterations"
    ax.set_ylabel(ylabel, weight="bold", fontsize=25, labelpad=10)

    width = 0.5 / (len(models) + 1)

    medianprops = dict(linestyle="--", linewidth=2.5, color="black", markersize=2)
    box_plots = {key: {} for key in models}
    for idx, model in enumerate(models):
        position = ((len(models) - 1) / 2.0 - idx) * width
        for idx2, setup in enumerate(exp_iters.keys()):
            box_plots[model][setup] = ax.boxplot(
                data[model][setup],
                positions=[idx2 * 0.5 - position],
                widths=[width],
                patch_artist=True,
                medianprops=medianprops,
            )

        for patch in [
            box_plots[model][setup]["boxes"][0] for setup in exp_iters.keys()
        ]:
            patch.set_facecolor(colors[model])

    plt.xticks(weight="bold", fontsize=10, rotation=30, ha="right")
    ax.set_xticks(np.arange(len(exp_iters.keys())) * 0.5)
    ax.set_xticklabels([key.title() for key in exp_iters.keys()])
    fig.tight_layout()

    ax.set_ylim(ymin=0)
    ax.set_xlim(xmin=-2.5 * width, xmax=3)
    ax.yaxis.grid(True)

    plt.tick_params(
        axis="x",
        which="both",
        bottom=False,
        top=False,
    )

    if plot is True:
        plt.show()
    if save_figs:
        fig.savefig(f"./{results}/iters.png", dpi=300)
        tikzplotlib.save(
            f"./{results}/iters.tex", axis_height="2.2in", axis_width="3.25in"
        )
    plt.close()


def plot_done():
    all_results = load_all_models()

    fig = plt.figure(figsize=(16.0, 10.0))
    ax = fig.add_subplot(111)

    data = {}

    for model in models:
        exp_data = all_results[model]
        exp_done = {
            "clear": np.mean(exp_data["done"][:10]),
            "cube": np.mean(exp_data["done"][10:20]),
            "wall": np.mean(exp_data["done"][20:30]),
            "L_right": np.mean(exp_data["done"][30:40]),
            # 'L_left': np.mean(exp_data['done'][40:50]),
            "bucket": np.mean(exp_data["done"][50:]),
        }
        data[model] = exp_done

    ylabel = "Percentage of Completed Trials"
    ax.set_ylabel(ylabel, weight="bold", fontsize=25, labelpad=10)

    width = 0.5 / (len(models) + 1)

    for idx, model in enumerate(models):
        position = ((len(models) - 1) / 2.0 - idx) * width
        for idx2, setup in enumerate(exp_done.keys()):
            ax.bar(
                height=data[model][setup],
                x=idx2 * 0.5 - position,
                width=width,
                color=colors[model],
            )

    plt.xticks(weight="bold", fontsize=15, rotation=30, ha="right")
    ax.set_xticks(np.arange(len(exp_done.keys())) * 0.5)
    ax.set_xticklabels([key.title() for key in exp_done.keys()])
    fig.tight_layout()

    ax.set_ylim(ymin=0)
    ax.set_xlim(xmin=-2.5 * width, xmax=3)
    ax.yaxis.grid(True)

    plt.tick_params(
        axis="x",
        which="both",
        bottom=False,
        top=False,
    )

    if plot is True:
        plt.show()
    if save_figs:
        fig.savefig(f"./{results}/done.png", dpi=300)
        tikzplotlib.save(
            f"./{results}/done.tex", axis_height="2.2in", axis_width="3.25in"
        )
    plt.close()


if __name__ == "__main__":
    plot_iters()
    plot_done()
