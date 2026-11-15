"""Simulation of UDP message loss on the proximity channel (UDP 5006).

Rules taken from the implementation (Section 3.5 of the manuscript):
* the Python sender transmits on every state change and re-sends the current
  state every 0.25 s while it processes valid input;
* the Unity evaluator accepts an input only if it is at most tau = 0.5 s old.

Loss models (same average loss rate p):
* independent: each message is lost with probability p;
* bursty: two-state Gilbert-Elliott chain with all messages lost in the bad
  state and a mean burst length of 4 messages (1 s).

Outputs per loss rate:
* share of time in which the newest received message is older than tau
  (input classified as stale by the Unity evaluator);
* delivery delay of a newly issued STOP request (time from the state change to
  the first received message carrying it): mean and 95th percentile.
"""
import json
import sys

import numpy as np

out_json = sys.argv[1] if len(sys.argv) > 1 else "message_loss_results.json"
RESEND, TAU, BURST = 0.25, 0.5, 4.0
rng = np.random.default_rng(0)


def losses(n, p, model):
    if p == 0:
        return np.zeros(n, bool)
    if model == "independent":
        return rng.random(n) < p
    # Gilbert-Elliott: P(bad->good) = 1/BURST, stationary bad share = p
    q = 1.0 / BURST
    g2b = p * q / (1 - p)
    out = np.empty(n, bool)
    bad = rng.random() < p
    u = rng.random(n)
    for i in range(n):
        out[i] = bad
        bad = (u[i] >= q) if bad else (u[i] < g2b)
    return out


def stale_share(p, model, n=400000):
    lost = losses(n, p, model)
    # messages every RESEND seconds; at any time the age of the newest received
    # message exceeds TAU iff the last floor(TAU/RESEND)=2 messages were lost
    k = int(round(TAU / RESEND))
    run = 0
    stale = 0
    for i in range(n):
        run = run + 1 if lost[i] else 0
        stale += run >= k
    return stale / n


def stop_delay(p, model, n=200000):
    lost = losses(n * 3, p, model)
    # a STOP is issued at a random message slot; it is re-sent every RESEND s
    starts = rng.integers(0, n * 3 - 200, size=n)
    delays = np.empty(n)
    for j, s in enumerate(starts):
        k = 0
        while lost[s + k]:
            k += 1
        delays[j] = k * RESEND
    return float(delays.mean()), float(np.percentile(delays, 95))


res = {"resend_s": RESEND, "tau_s": TAU, "burst_mean_messages": BURST, "rows": []}
for p in (0.0, 0.01, 0.05, 0.10, 0.20, 0.30):
    row = {"loss_rate": p}
    for model in ("independent", "bursty"):
        m, p95 = stop_delay(p, model, n=50000)
        row[model] = {
            "stale_time_share": stale_share(p, model, n=200000),
            "stop_delay_mean_s": m,
            "stop_delay_p95_s": p95,
        }
    res["rows"].append(row)
    print(row)
json.dump(res, open(out_json, "w"), indent=2)
