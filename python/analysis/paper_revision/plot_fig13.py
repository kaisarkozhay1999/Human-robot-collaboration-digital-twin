"""Figure 13: propagated distance uncertainty and message-loss sensitivity."""
import json
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

unc = json.load(open(sys.argv[1]))
loss = json.load(open(sys.argv[2]))
out = sys.argv[3]

TEAL, GREY, TITLE = "#33617d", "#8a8a8a", "#234e63"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 10.5,
                     "axes.spines.top": False, "axes.spines.right": False})
fig, ax = plt.subplots(1, 3, figsize=(10.24, 3.15))

# (a) distance uncertainty
s = [r["sigma_px"] for r in unc["sweep"]]
sd = [r["distance_error_sd_m"] * 1000 for r in unc["sweep"]]
p95 = [r["distance_error_abs_p95_m"] * 1000 for r in unc["sweep"]]
a = ax[0]
a.plot(s, sd, "-o", color=TEAL, lw=2, ms=5, label="SD")
a.plot(s, p95, "--s", color=GREY, lw=2, ms=5, label="95th percentile |error|")
a.axhline(150, color="black", ls=":", lw=1.2)
a.axhline(75, color="black", ls=":", lw=1.2)
a.text(0.55, 158, "hysteresis band 150 mm", fontsize=8.5)
a.text(0.55, 83, "half band 75 mm", fontsize=8.5)
a.set_xscale("log", base=2)
a.set_xticks(s)
a.set_xticklabels([str(x).rstrip("0").rstrip(".") for x in s])
a.set_ylim(0, 190)
a.set_xlabel("Keypoint noise per image, SD (px)")
a.set_ylabel("Distance error (mm)")
a.set_title("(a) Stereo distance uncertainty", color=TITLE, fontweight="bold", fontsize=11)
a.legend(frameon=False, fontsize=8.5, loc="upper left", bbox_to_anchor=(0.0, 0.72))

# (b) stale-input share and (c) STOP delivery delay
p = [r["loss_rate"] * 100 for r in loss["rows"]]
for model, style, col, lab in (("independent", "-o", TEAL, "Independent loss"),
                                ("bursty", "--s", GREY, "Bursty loss (1 s bursts)")):
    st = [r[model]["stale_time_share"] * 100 for r in loss["rows"]]
    ax[1].plot(p, st, style, color=col, lw=2, ms=5, label=lab)
    m = [r[model]["stop_delay_mean_s"] for r in loss["rows"]]
    q = [r[model]["stop_delay_p95_s"] for r in loss["rows"]]
    ax[2].plot(p, m, style, color=col, lw=2, ms=5, label=lab + ", mean")
    ax[2].plot(p, q, style[0:-1] + "^" if style.startswith("-o") else ":^", color=col, lw=1.2, ms=4,
               alpha=0.8, label=lab + ", P95")
ax[1].set_xlabel("Message loss rate (%)")
ax[1].set_ylabel("Time with stale input (%)")
ax[1].set_title("(b) Input older than 0.5 s", color=TITLE, fontweight="bold", fontsize=11)
ax[1].legend(frameon=False, fontsize=8.5)
ax[2].set_xlabel("Message loss rate (%)")
ax[2].set_ylabel("STOP delivery delay (s)")
ax[2].set_title("(c) New STOP request delay", color=TITLE, fontweight="bold", fontsize=11)
ax[2].legend(frameon=False, fontsize=7.5, loc="upper left")
for x in ax:
    x.grid(axis="y", color="#e6e6e6", lw=0.8)
fig.tight_layout()
fig.savefig(out, dpi=300)
print("saved", out)
