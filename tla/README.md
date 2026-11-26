# TLA+ model of the firmware state machine

`NutrientMixer.tla` models the control logic of `../nutrient_mixer.ino`:
the top-level `State` in `loop()`, the `pHadjustment()` and `TDSadjustment()`
sub-machines, their round counters, the shared `runCountDown()` timer and
which pumps are energised.

Each `loop()` iteration is two steps: `pc = "read"` applies an incoming
command (`switch (command)`), and `pc = "exec"` runs one slice of work
(`switch (state)`). Timer expiry and sensor results are nondeterministic,
so TLC explores every ordering.

| Config | Purpose |
| --- | --- |
| `MC_graph.cfg` | Intended behaviour: commands only when IDLE, `!` anytime. Used to draw the diagrams. |
| `MC_firmware.cfg` | Faithful to the code: commands are applied even while busy. Checks safety invariants. |
| `MC_boot.cfg` | Checks the pump state that `setup()` leaves behind. |

## Regenerate the diagrams

```bash
curl -LO https://github.com/tlaplus/tlaplus/releases/download/v1.8.0/tla2tools.jar
```

```bash
java -cp tla2tools.jar tlc2.TLC -config MC_graph.cfg -dump dot,actionlabels graph NutrientMixer.tla
```

```bash
python3 graph_to_mermaid.py graph.dot > diagrams.md
```

## Check the invariants

```bash
java -cp tla2tools.jar tlc2.TLC -config MC_firmware.cfg NutrientMixer.tla
```

## Diagrams


### Top-level `loop()` state

```mermaid
stateDiagram-v2
    direction LR
    [*] --> IDLE
    ABORT --> IDLE: pumps off + reboot
    ADJUST_ALL --> ADJUST_PH: TDS in range
    ADJUST_PH --> IDLE: in range or 6 rounds
    ADJUST_TDS --> IDLE: in range or 6 rounds
    CIRCULATE --> IDLE: timer expired
    EMPTY --> IDLE: timer expired
    FILL --> IDLE: timer expired
    IDLE --> ABORT: !
    IDLE --> ADJUST_ALL: a (adjust all)
    IDLE --> ADJUST_PH: p (adjust pH)
    IDLE --> ADJUST_TDS: t (adjust TDS)
    IDLE --> CIRCULATE: c / menu Circulate
    IDLE --> EMPTY: e / menu Empty
    IDLE --> FILL: f / menu Fill
    IDLE --> MEASURE: m (measure)
    IDLE --> PLOT: l (plot)
    IDLE --> PUMP_NUTRIENT: n (pump nutrients)
    IDLE --> PUMP_PH: u (pump acid)
    MEASURE --> IDLE: readings printed
    PLOT --> IDLE: timer expired
    PUMP_NUTRIENT --> CIRCULATE: target 0 ⇒ done at once
    PUMP_PH --> CIRCULATE: acid timer expired
    note right of ABORT: '!' is accepted in every state
```

### `pHadjustment()` sub-machine

```mermaid
stateDiagram-v2
    state "ADJUST_PH" as SUB {
    [*] --> PH_MEASURE
    PH_CIRCULATE --> PH_MEASURE: circulation timer expired
    PH_MEASURE --> PH_PUMP: pH above target, round < 6
    PH_PUMP --> PH_CIRCULATE: acid timer expired
    PH_MEASURE --> [*]: in range or round > 5
    }
    note right of SUB
        on exit: ADJUST_PH → IDLE
    end note
```

### `TDSadjustment()` sub-machine

```mermaid
stateDiagram-v2
    state "ADJUST_TDS / ADJUST_ALL / PUMP_NUTRIENT" as SUB {
    [*] --> TDS_MEASURE
    TDS_A_PUMP --> TDS_B_PUMP: nutrient A timer expired
    TDS_B_PUMP --> TDS_CIRCULATE: nutrient B timer expired
    TDS_CIRCULATE --> TDS_MEASURE: circulation timer expired
    TDS_MEASURE --> TDS_A_PUMP: TDS below target, round < 6
    TDS_MEASURE --> [*]: in range or round > 5
    }
    note right of SUB
        on exit: ADJUST_ALL → ADJUST_PH<br/>ADJUST_TDS → IDLE<br/>PUMP_NUTRIENT → CIRCULATE
    end note
```

## What TLC found (`MC_firmware.cfg`, `MC_boot.cfg`)

1. **A command sent mid-operation leaves pumps running.** `loop()` prints
   "Operation in progress" but still applies the command. Trace: `e` (output
   pump on, timer armed), then `t`: `TDSadjustment()` finishes at once and the
   machine is IDLE with the output pump still on. Violates `IdleMeansPumpsOff`.
2. **The shared countdown leaks between operations.** `runCountDown()` keeps
   one `static` timer for every caller. In the same trace it stays armed in
   IDLE (`IdleMeansCountdownFree`), so the next timed operation skips its
   `start()` callback and only runs `stop()`.
3. **Acid pump can keep dosing outside a dosing state.** `u` then `t`
   leaves the acid pump on in `ADJUST_TDS` (`AcidOnlyWhenDosing`).
4. **Relays are on at boot.** `setup()` writes HIGH to both relay pins, which
   `engagePump()` treats as "pumping" (`MC_boot.cfg`). GPIO 38/39 are also
   input-only on the ESP32, as the TODO at the top of the sketch suspects.
5. **`n` never pumps nutrients.** `PUMP_NUTRIENT` calls `TDSadjustment()`
   with its default target 0, which returns done from `TDS_MEASURE`, so the
   machine goes straight to `CIRCULATE`. It only doses if an interrupted TDS
   adjustment left the sub-machine mid-cycle.
