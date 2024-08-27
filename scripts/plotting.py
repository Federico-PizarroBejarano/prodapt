import pickle

import matplotlib.pyplot as plt
import numpy as np

plot = True
save_figs = False

baseline = "baseline"
config = "medium"
diff_iters = 45

models = [
    f"{config}_{diff_iters}_3_10",
    f"{baseline}_{diff_iters}_3",
    # f"{baseline}_{diff_iters}_6",
    # f"{baseline}_{diff_iters}_20",
    # f"{baseline}_{diff_iters}_50",
]
colors = {
    f"{config}_{diff_iters}_3_10": "cornflowerblue",
    f"{baseline}_{diff_iters}_3": "lightgreen",
    f"{baseline}_{diff_iters}_6": "limegreen",
    f"{baseline}_{diff_iters}_20": "forestgreen",
    f"{baseline}_{diff_iters}_50": "darkgreen",
}

setups = ["clear", "cube", "wall", "bucket"]

results = "results_real"
REAL = True


def load_all_models():
    all_results = {}

    for model in models:
        if REAL:
            all_results[model] = {"done": {}, "iters": {}}
            for setup in setups:
                with open(f"./{results}/{model}/{setup}.pkl", "rb") as f:
                    data = pickle.load(f)
                    all_results[model]["done"][setup] = data["done"]
                    all_results[model]["iters"][setup] = np.array(data["iters"])[
                        data["done"]
                    ]
        else:
            with open(f"./{results}/{model}.pkl", "rb") as f:
                exp_data = pickle.load(f)
            exp_done = {
                "clear": exp_data["done"][:10],
                "cube": exp_data["done"][10:20],
                "wall": exp_data["done"][20:30],
                "bucket": exp_data["done"][30:],
            }
            exp_iters = {
                "clear": np.array(exp_data["iters"][:10])[exp_done["clear"]],
                "cube": np.array(exp_data["iters"][10:20])[exp_done["cube"]],
                "wall": np.array(exp_data["iters"][20:30])[exp_done["wall"]],
                "bucket": np.array(exp_data["iters"][30:])[exp_done["bucket"]],
            }
            all_results[model] = {}
            all_results[model]["done"] = exp_done
            all_results[model]["iters"] = exp_iters

    return all_results


def plot_iters():
    all_results = load_all_models()

    fig = plt.figure(figsize=(16.0, 10.0))
    ax = fig.add_subplot(111)

    ylabel = "Number of Iterations"
    ax.set_ylabel(ylabel, weight="bold", fontsize=25, labelpad=10)

    width = 0.5 / (len(models) + 1)

    medianprops = dict(linestyle="--", linewidth=2.5, color="black", markersize=2)
    box_plots = {key: {} for key in models}
    for idx, model in enumerate(models):
        position = ((len(models) - 1) / 2.0 - idx) * width
        for idx2, setup in enumerate(setups):
            box_plots[model][setup] = ax.boxplot(
                all_results[model]["iters"][setup],
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
        fig.savefig(f"./{results}/iters.png", dpi=300)

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
        fig.savefig(f"./{results}/done.png", dpi=300)
    plt.close()


if __name__ == "__main__":
    plot_iters()
    plot_done()
