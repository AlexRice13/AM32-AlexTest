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
| `demag_metric` | `uint8_t` | Sliding exponential average of demag events. Range: **0** (healthy) … **255** (heavy demag). |
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
  decays back toward **0** (natural lower bound for `uint8_t`).

```c
uint16_t metric = (uint16_t)demag_metric * 7;
if (flag_demag_detected) {
    metric += 256;          // push toward 255
    flag_demag_notify = 1;  // signal EDT telemetry
}
metric >>= 3;               // divide by 8
demag_metric = (uint8_t)metric;
```

After updating the metric, `commutate()` sets `flag_demag_detected = 1` as a **pessimistic
default** for the upcoming cycle — it will be cleared only if a zero cross is confirmed.

---

## Power cut on heavy demag (step skipping disabled)

Immediately after the metric update, if the metric exceeds the user threshold, all FETs are
turned off and phase interrupts are masked:

```c
if (demag_metric > demag_pwr_off_thresh) {
    allOff();
    maskPhaseInterrupts();
    // Step-skip disabled: caused motor stall during flight testing.
    // uint8_t excess = demag_metric - demag_pwr_off_thresh;
    // uint8_t extra_steps = excess >> 5;
    // if (extra_steps > 2) { extra_steps = 2; }
}
```

> **Note:** The extra commutation step-skip feature (which skipped 1–2 extra steps based on
> demag severity) was disabled after flight testing revealed it caused motor stall and prevented
> acceleration. Power is cut briefly and then restored by `comStep(step)` + `changeCompInput()`
> in the same `commutate()` call.

---

## Advance timing reduction — `adjust_comm_timing()` (disabled)

`adjust_comm_timing()` was designed to reduce the advance angle proportionally when demag is
elevated. It was **disabled** after flight testing revealed it caused motor stall:

```c
//adjust_comm_timing(); // disabled: auto timing adjust caused motor stall during flight testing
```

The function definition is retained in the source for future reference.

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

When DShot Extended Telemetry (EDT) is active, the demag activity level is reported on
frame ID `0x0C` (EDT Debug [3]) whenever `flag_demag_notify` is set (i.e., a demag event
occurred since the last report) and the scheduler rate divisor has elapsed:

```c
telem_scheduler.demag_count = 0; // always reset to prevent counter overflow
if (flag_demag_notify) {
    // Capture demag_metric into the frame BEFORE clearing the notify flag
    // to guarantee the value is read before any potential zero-clear.
    extended_frame_to_send = 0b1100 << 8 | demag_metric;
    flag_demag_notify = 0; // clear only after capture
}
```

The value ranges from **0** (healthy motor, no demag activity) to **255** (maximum
demagnetization). Sending only when `flag_demag_notify` is set means a frame is only
emitted when a real demag event has occurred; the counter is always reset to prevent
`uint16_t` overflow regardless.

---

## Summary — end-to-end flow

```
Each commutation (commutate()):
  1. Compute EMA:  demag_metric ← (demag_metric×7 + event×256) / 8, range [0, 255]
  2. If demag_metric > demag_pwr_off_thresh:
       a. allOff()  (brief freewheel)
       [Step-skip disabled: was causing motor stall during flight]
  3. Set flag_demag_detected = 1  (pessimistic default for next cycle)
  4. Advance step by 1 in the motor direction
  5. comStep(step) — restore power on the new step   (motor auto-continues)
  6. changeCompInput() — re-enable BEMF zero-cross sensing on the new step

Between commutations:
  - If a zero cross is found  →  flag_demag_detected = 0
  - If timeout or desync      →  flag_demag_detected stays / is set to 1

After commutation timer fires (PeriodElapsedCallback()):
  [adjust_comm_timing() disabled: was causing motor stall during flight]
  7. Set waitTime = commutation_interval/2 − advance
```
