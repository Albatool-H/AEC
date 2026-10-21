#!/usr/bin/env python3
import argparse
import os
import re
import signal
import sys
import time
from pathlib import Path

try:
    import vowpalwabbit
except ImportError:
    sys.stderr.write(
        "vowpalwabbit not installed. Install with: pip install vowpalwabbit\n")
    sys.exit(1)

_RUNNING = True

def _on_signal(signum, frame):
    global _RUNNING
    _RUNNING = False

class BranchAgent:
    """One VW workspace per branch ('dead' or 'live').

    Each workspace is independent so the dead-branch features (load_inv,
    count_inv) and live-branch features (gap, neg_cost) get their own
    theta. This matches the per-branch design in project-plan.md 5d.
    """

    BRANCH_RE = re.compile(r"^shared \|Branch (\w+)", re.MULTILINE)

    def __init__(self, epsilon: float):
        args = f"--cb_explore_adf --epsilon {epsilon} --quiet"
        self._dead = vowpalwabbit.Workspace(args)
        self._live = vowpalwabbit.Workspace(args)

    def _route(self, payload: str):
        m = self.BRANCH_RE.search(payload)
        if not m:
            return self._dead
        return self._dead if m.group(1) == "dead" else self._live

    def predict(self, payload: str):
        """Returns (chosen_idx, probability) for this payload."""
        vw = self._route(payload)

        probs = vw.predict(payload.strip().split("\n"))

        import random
        r = random.random()
        cum = 0.0
        for idx, p in enumerate(probs):
            cum += float(p)
            if r <= cum:
                return idx, float(p)

        return len(probs) - 1, float(probs[-1])

    def learn(self, last_payload: str, chosen_idx: int,
              reward: float, probability: float):
        """Apply gradient update using last_payload as the example.

        VW's --cb_explore_adf learn API needs the action label set on
        the chosen action line in the form: "<idx>:<cost>:<prob>".
        Cost = -reward (VW minimises cost).
        """
        vw = self._route(last_payload)
        lines = last_payload.strip().split("\n")
        cost = -float(reward)

        target_line_idx = chosen_idx + 1
        if 0 <= target_line_idx < len(lines):
            line = lines[target_line_idx]
            lines[target_line_idx] = f"{chosen_idx}:{cost}:{probability} {line}"
        vw.learn(lines)

def _parse_id(name: str, prefix: str) -> int:

    base = name[len(prefix):]
    base = base.split(".", 1)[0]
    return int(base)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bridge-dir", required=True,
                    help="Bridge root (matches controller's --rlBridgeDir)")
    ap.add_argument("--epsilon", type=float, default=0.05,
                    help="ε-greedy exploration rate")
    ap.add_argument("--poll-seconds", type=float, default=0.005,
                    help="File-poll cadence")
    args = ap.parse_args()

    bridge = Path(args.bridge_dir)
    bridge.mkdir(parents=True, exist_ok=True)
    agent = BranchAgent(args.epsilon)

    signal.signal(signal.SIGINT,  _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)
    print(f"[vw-agent] watching {bridge} (epsilon={args.epsilon})", flush=True)

    pending = {}

    while _RUNNING:
        seen_anything = False
        for ctrl_dir in sorted(bridge.iterdir()):
            if not ctrl_dir.is_dir():
                continue
            ctrl = ctrl_dir.name

            for f in sorted(ctrl_dir.glob("decision_*.req")):
                seen_anything = True
                did = _parse_id(f.name, "decision_")
                try:
                    payload = f.read_text()
                    chosen_idx, prob = agent.predict(payload)
                    resp = ctrl_dir / f"decision_{did}.resp"
                    resp_tmp = ctrl_dir / f"decision_{did}.resp.tmp"
                    resp_tmp.write_text(f"{chosen_idx} {prob}\n")
                    os.rename(resp_tmp, resp)
                    pending[(ctrl, did)] = payload
                except Exception as e:
                    sys.stderr.write(f"[vw-agent] decision {ctrl}/{did} error: {e}\n")
                finally:
                    try: f.unlink()
                    except FileNotFoundError: pass

            for f in sorted(ctrl_dir.glob("feedback_*.req")):
                seen_anything = True
                did = _parse_id(f.name, "feedback_")
                try:
                    parts = f.read_text().strip().split()
                    if len(parts) != 3:
                        continue
                    chosen_idx = int(parts[0])
                    reward     = float(parts[1])
                    prob       = float(parts[2])
                    key = (ctrl, did)
                    if key in pending:
                        agent.learn(pending[key], chosen_idx, reward, prob)
                        del pending[key]
                except Exception as e:
                    sys.stderr.write(f"[vw-agent] feedback {ctrl}/{did} error: {e}\n")
                finally:
                    try: f.unlink()
                    except FileNotFoundError: pass

        if not seen_anything:
            time.sleep(args.poll_seconds)

    print("[vw-agent] exiting", flush=True)

if __name__ == "__main__":
    main()
