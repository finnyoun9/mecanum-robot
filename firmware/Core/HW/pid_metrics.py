#!/usr/bin/env python3
"""Turn a pid_step GDB dump into step-response metrics.

Reports rise time, overshoot, steady-state error and IAE so gain choices
can be compared on numbers rather than on how the trace looks. Also reports
loop-period jitter, since a control loop that misses its deadline
invalidates any tuning done on top of it.

Usage: pid_metrics.py <gdb_log> <kp> <ki> <setpoint>
"""
import re
import sys

DT = 0.01          # 100 Hz control loop
PRE_SAMPLES = 20   # samples held at setpoint 0, matching pid_step_main.c


def parse_array(log: str, name: str):
    """Pull one `$n = {a, b, c}` GDB array out of the log."""
    m = re.search(re.escape(name) + r"\s*=\s*\{(.*?)\}", log, re.S)
    if not m:
        return None
    body = m.group(1).replace("\n", " ")
    out = []
    for tok in body.split(","):
        tok = tok.strip()
        if not tok:
            continue
        # GDB may abbreviate runs as "value <repeats N times>".
        rep = re.match(r"(-?[\d.eE+]+)\s*<repeats\s+(\d+)\s+times>", tok)
        if rep:
            out.extend([float(rep.group(1))] * int(rep.group(2)))
            continue
        try:
            out.append(float(tok))
        except ValueError:
            pass
    return out


def main():
    log_path, kp, ki, sp = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4])
    log = open(log_path, encoding="utf-8", errors="replace").read()

    speed = parse_array(log, "log_speed")
    pwm = parse_array(log, "log_pwm")
    period = parse_array(log, "log_period_us")

    if not speed:
        print("Could not parse log_speed — raw log follows:\n")
        print(log[-1500:])
        sys.exit(1)

    resp = speed[PRE_SAMPLES:]
    if not resp:
        print("No post-step samples captured.")
        sys.exit(1)

    # Steady state: mean of the last 30% of the response.
    tail_start = max(0, int(len(resp) * 0.7))
    tail = resp[tail_start:]
    ss = sum(tail) / len(tail)
    ss_err = sp - ss
    ss_err_pct = (ss_err / sp * 100.0) if sp else 0.0

    peak = max(resp)
    overshoot_pct = ((peak - sp) / sp * 100.0) if sp else 0.0

    # Rise time 10% -> 90% of the setpoint.
    def first_cross(frac):
        target = sp * frac
        for i, v in enumerate(resp):
            if v >= target:
                return i
        return None

    i10, i90 = first_cross(0.1), first_cross(0.9)
    rise = f"{(i90 - i10) * DT * 1000:.0f} ms" if (i10 is not None and i90 is not None) else "not reached"

    iae = sum(abs(sp - v) for v in resp) * DT

    print(f"=== Kp={kp}  Ki={ki}  setpoint={sp} rad/s ===")
    print(f"samples          {len(resp)} post-step ({len(resp)*DT:.2f} s)")
    print(f"steady state     {ss:.2f} rad/s")
    print(f"steady-state err {ss_err:+.2f} rad/s  ({ss_err_pct:+.1f}%)")
    print(f"peak             {peak:.2f} rad/s")
    print(f"overshoot        {overshoot_pct:+.1f}%")
    print(f"rise 10-90%      {rise}")
    print(f"IAE              {iae:.2f} rad")

    if pwm:
        pr = pwm[PRE_SAMPLES:]
        sat = sum(1 for v in pr if abs(v) >= 1000)
        print(f"duty final       {pr[-1]:.0f}   max {max(pr):.0f}   "
              f"saturated {sat}/{len(pr)} samples")

    if period:
        pp = [p for p in period[1:] if p > 0]
        if pp:
            print(f"loop period      mean {sum(pp)/len(pp):.0f} us, "
                  f"min {min(pp):.0f}, max {max(pp):.0f}  (target 10000)")


if __name__ == "__main__":
    main()
