# TLA+ model of the firmware state machine

`NutrientMixer.tla` models the control logic of `../nutrient_mixer.ino`: the
operation `state` in `loop()`, the `phAdjustment()` and `tdsAdjustment()`
sub-machines and their round counters, the shared `countDown` timer, which
sensor is powered, the `measureAll()` step and which pumps are requested.

Each `loop()` iteration is two steps: `pc = "read"` applies at most one
command (serial, menu or abort switch) through `handleCommand()`, and
`pc = "exec"` runs one slice of the current operation. Timer expiry, sensor
warm-up and sensor results are nondeterministic, so TLC explores every
ordering. Targets, durations, calibration, WiFi and the display are left
out. The float switches are not modelled: they only gate the relays in
`updatePumps()`, so `pumps` is what the firmware has requested.

## Checked properties (`MC.cfg`)

| Property | Meaning |
| --- | --- |
| `IdleMeansPumpsOff` | Back in IDLE, no pump is requested. |
| `IdleMeansTimerFree` | The shared countdown is not left running in IDLE. |
| `IdleMeansSensorsOff` | Both sensors are powered down in IDLE. |
| `AcidOnlyWhenDosing`, `NutrientAOnlyWhenDosing`, `NutrientBOnlyWhenDosing` | A dosing pump only runs in its own dosing state. |
| `OneDosingPumpAtATime` | Never more than one peristaltic pump at once. |
| `NoDosingWhileCirculating` | No dosing while the tank circulates. |
| `SensorsOnlyWhenMeasuring` | A sensor is only powered by an operation that reads it. |
| `OperationsFinish` | Every operation returns to IDLE (liveness, assuming time passes and readings complete). |

All hold: 1,092 distinct states, no deadlocks.

The model was checked against reintroduced bugs to make sure the properties
can fail. Accepting commands while busy, not cancelling the timer on abort,
removing the round limit and dropping the progress assumption each produce a
counterexample. Removing `cancelCountDown()` from `startOperation()` does
not: the busy check means it only runs with the timer already stopped, so
that call is a safety net.

## Run it

```bash
curl -LO https://github.com/tlaplus/tlaplus/releases/download/v1.8.0/tla2tools.jar
```

```bash
java -cp tla2tools.jar tlc2.TLC -config MC.cfg -dump dot,actionlabels graph NutrientMixer.tla
```

```bash
python3 graph_to_mermaid.py graph.dot > diagrams.md
```

## Diagrams

Projected from the TLC state graph onto `state` and the adjustment sub-states.

### Top-level `loop()` state

```mermaid
stateDiagram-v2
    direction LR
    [*] --> IDLE
    ADJUST_ALL --> ADJUST_PH: TDS reached or 6 rounds
    ADJUST_ALL --> IDLE: TDS unreadable
    ADJUST_PH --> IDLE: reached, 6 rounds or unreadable
    ADJUST_TDS --> IDLE: reached, 6 rounds or unreadable
    CIRCULATE --> IDLE: time over
    EMPTY --> IDLE: time over
    FILL --> IDLE: time over
    IDLE --> ADJUST_ALL: a / menu Adjust all
    IDLE --> ADJUST_PH: p
    IDLE --> ADJUST_TDS: t
    IDLE --> CIRCULATE: c / menu Circulate
    IDLE --> EMPTY: e / menu Empty
    IDLE --> FILL: f / menu Fill
    IDLE --> MEASURE: m / menu Measure
    IDLE --> PLOT: l
    IDLE --> PUMP_NUTRIENT_A: n / n a / menu Pump A
    IDLE --> PUMP_NUTRIENT_B: n b / menu Pump B
    IDLE --> PUMP_PH: u / menu Pump acid
    MEASURE --> IDLE: TDS and pH read
    PLOT --> IDLE: plot time over
    PUMP_NUTRIENT_A --> CIRCULATE: dose done (n a)
    PUMP_NUTRIENT_A --> PUMP_NUTRIENT_B: dose done (n)
    PUMP_NUTRIENT_B --> CIRCULATE: dose done
    PUMP_PH --> CIRCULATE: dose done
    note left of IDLE
        While busy, only ! (and ?, w) is accepted.<br/>Abort returns to IDLE from any state.
    end note
```

### `pHadjustment()` sub-machine

```mermaid
stateDiagram-v2
    state "ADJUST_PH" as SUB {
    [*] --> PH_MEASURE
    PH_CIRCULATE --> PH_SETTLE: circulation time over
    PH_MEASURE --> PH_PUMP: pH too high, round < 6
    PH_PUMP --> PH_CIRCULATE: acid dose done
    PH_SETTLE --> PH_MEASURE: settle time over
    PH_MEASURE --> [*]: reached, 6 rounds or unreadable
    }
    note right of SUB
        on exit: ADJUST_PH → IDLE
    end note
```

### `TDSadjustment()` sub-machine

```mermaid
stateDiagram-v2
    state "ADJUST_TDS / ADJUST_ALL" as SUB {
    [*] --> TDS_MEASURE
    TDS_A_PUMP --> TDS_B_PUMP: A dose done
    TDS_B_PUMP --> TDS_CIRCULATE: B dose done
    TDS_CIRCULATE --> TDS_SETTLE: circulation time over
    TDS_MEASURE --> TDS_A_PUMP: TDS too low, round < 6
    TDS_SETTLE --> TDS_MEASURE: settle time over
    TDS_MEASURE --> [*]: reached, 6 rounds or unreadable
    }
    note right of SUB
        on exit: ADJUST_ALL → ADJUST_PH<br/>ADJUST_ALL → IDLE<br/>ADJUST_TDS → IDLE
    end note
```
