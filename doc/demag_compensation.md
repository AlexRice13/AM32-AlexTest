# Demag Compensation

This document explains how demagnetization (demag) compensation works in AM32, based on `Src/main.c` and `Src/dshot.c`.

---

## What is demagnetization?

When a BLDC motor phase is switched off, the current stored in the stator winding cannot stop instantly.
The residual magnetic flux ("demag current") can corrupt the back-EMF (BEMF) zero-crossing signal that
the firmware relies on to time the next commutation.  If a zero cross is missed or skewed, the motor
can desynchronise and lose torque or stall.

---

## Key variables

| Variable | Type | Description |
|---|---|---|
| `demag_metric` | `uint8_t` | Sliding exponential average of demag events. Range: **120** (healthy) … **255** (heavy demag). |
| `demag_metric_max` | `uint8_t` | Highest `demag_metric` value seen since power-on; never decreases. |
| `demag_pwr_off_thresh` | `uint8_t` | Power-cutoff threshold. If `demag_metric` exceeds this value during commutation, drive power is cut briefly. **255** = compensation disabled. |
| `flag_demag_detected` | `uint8_t` | Set to **1** at the start of each commutation cycle (pessimistic assumption). Cleared to **0** if a valid zero-cross is found before the next commutation. |
| `flag_demag_notify` | `uint8_t` | Set to **1** whenever a demag event is registered; cleared after it has been reported over the EDT telemetry channel. |

---

## Configuration (`eepromBuffer.demag_comp`)

The user-configurable `demag_comp` EEPROM setting maps to `demag_pwr_off_thresh` at startup:

```c
demag_pwr_off_thresh = 160; // default: low compensation
if (eepromBuffer.demag_comp == 1) {
    demag_pwr_off_thresh = 255; // off
} else if (eepromBuffer.demag_comp == 3) {
    demag_pwr_off_thresh = 130; // high compensation
}
```

| `demag_comp` | `demag_pwr_off_thresh` | Effect |
|---|---|---|
| 1 | 255 | Compensation disabled |
| 2 (default) | 160 | Low sensitivity – cuts power only under significant demag |
| 3 | 130 | High sensitivity – cuts power earlier, protects more aggressively |

---

## How the demag metric is updated — `commutate()`

Every time the firmware commutates (switches to the next motor phase), `commutate()` runs.
Its first action is to update `demag_metric` using a **7/8 exponential moving average (EMA)**:

```
new = (old × 7  +  event × 256) / 8
```

- If `flag_demag_detected == 1` (no zero cross was found in the previous cycle),
  `event = 1` → the metric is pushed toward **255**.
- If `flag_demag_detected == 0` (a zero cross was found), `event = 0` → the metric
  decays back toward **120** (the floor clamp).

```c
uint16_t metric = (uint16_t)demag_metric * 7;
if (flag_demag_detected) {
    metric += 256;          // push toward 255
    flag_demag_notify = 1;  // signal EDT telemetry
}
metric >>= 3;               // divide by 8
if (metric < 120) metric = 120;  // floor clamp
demag_metric = (uint8_t)metric;
```

After updating the metric, `commutate()` sets `flag_demag_detected = 1` as a **pessimistic
default** for the upcoming cycle — it will be cleared only if a zero cross is confirmed.

---

## Power cut and step skipping on heavy demag

Immediately after the metric update, if the metric exceeds the user threshold, all FETs are
turned off, phase interrupts are masked, and a number of **extra commutation steps to skip** is
calculated based on how far the metric exceeds the threshold:

```c
if (demag_metric > demag_pwr_off_thresh) {
    allOff();
    maskPhaseInterrupts();
    // Skip 1 or more extra commutation steps based on demag severity.
    // Each 32 counts above the threshold adds one extra skip, capped at 2.
    uint8_t excess = demag_metric - demag_pwr_off_thresh;
    extra_steps = excess >> 5;
    if (extra_steps > 2) extra_steps = 2;
}
```

After the normal single-step advance (step+1 / step-1), any extra skips are applied:

```c
while (extra_steps > 0) {
    // advance step one more position in the same direction
    ...
    extra_steps--;
}
```

| `demag_metric − demag_pwr_off_thresh` | `extra_steps` | Total step advance |
|---|---|---|
| 1 – 31 | 0 | 1 step (normal) |
| 32 – 63 | 1 | 2 steps |
| 64 – 95 | 2 | 3 steps |
| ≥ 96 | 2 (capped) | 3 steps |

Power is restored automatically in the same call to `commutate()`: `comStep(step)` applies the
new (post-skip) commutation phase, and `changeCompInput()` re-enables BEMF zero-cross sensing.
The motor freewheels briefly, then self-resynchronises via back-EMF tracking on the advanced step.

---

## Advance timing reduction — `adjust_comm_timing()`

Even when demag does not reach the power-cut threshold, elevated demag causes commutation to
happen too early (the advance angle is too large), which worsens the problem.
`adjust_comm_timing()` reduces the advance angle proportionally to how far `demag_metric`
exceeds the healthy baseline (120):

```c
static inline void adjust_comm_timing(void)
{
    if (demag_pwr_off_thresh >= 255 || demag_metric <= 120) return;
    uint16_t reduction = (uint16_t)(((uint32_t)(demag_metric - 120) * advance) >> 7);
    advance = (reduction < advance) ? advance - reduction : 0;
}
```

| `demag_metric` | Reduction of `advance` |
|---|---|
| 120 | 0 % |
| 184 | ~50 % |
| 248 | ~100 % |
| 255 | 100 % (floored at 0) |

This function is called in `PeriodElapsedCallback()` after `advance` is computed and before
`waitTime` is set.

---

## How `flag_demag_detected` is managed

### Set to 1 (demag assumed / confirmed)

| Location | Reason |
|---|---|
| `commutate()` (line 875) | Pessimistic default — assume demag unless zero cross clears it |
| `interruptRoutine()` timeout (line 2171) | Commutation interval counter exceeded 45 000 ticks without a zero cross |
| Desync detection block (line 1999) | `getAbsDif` check detects a sudden >50 % change in `average_interval` |

### Cleared to 0 (healthy zero cross found)

| Location | Reason |
|---|---|
| `interruptRoutine()` (line 981) | Interrupt-mode: valid BEMF zero cross detected |
| `zcfoundroutine()` (line 1613) | Polling-mode: valid BEMF zero cross detected |

---

## EDT telemetry reporting

When DShot Extended Telemetry (EDT) is active, `demag_metric` is periodically reported on
frame ID `0x0C` and `flag_demag_notify` is cleared:

```c
extended_frame_to_send = 0b1100 << 8 | demag_metric;
flag_demag_notify = 0; // clear notify after reporting
```

The flight controller can use this value to monitor motor health in real time.

---

## Summary — end-to-end flow

```
Each commutation (commutate()):
  1. Compute EMA:  demag_metric ← (demag_metric×7 + event×256) / 8, clamped ≥ 120
  2. If demag_metric > demag_pwr_off_thresh:
       a. allOff()  (brief freewheel)
       b. extra_steps = (demag_metric − thresh) >> 5, capped at 2
  3. Set flag_demag_detected = 1  (pessimistic default for next cycle)
  4. Advance step by 1 (normal) + extra_steps (demag skip) in the motor direction
  5. comStep(step) — restore power on the new step   (motor auto-continues)
  6. changeCompInput() — re-enable BEMF zero-cross sensing on the new step

Between commutations:
  - If a zero cross is found  →  flag_demag_detected = 0
  - If timeout or desync      →  flag_demag_detected stays / is set to 1

After commutation timer fires (PeriodElapsedCallback()):
  7. adjust_comm_timing() — reduce advance angle proportionally to demag excess
  8. Set waitTime = commutation_interval/2 − advance
```
