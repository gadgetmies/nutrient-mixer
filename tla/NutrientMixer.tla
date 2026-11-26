--------------------------- MODULE NutrientMixer ---------------------------
(***************************************************************************)
(* Model of the control state machine in ../nutrient_mixer.ino.            *)
(*                                                                         *)
(* One iteration of loop() is two TLA+ steps:                              *)
(*   pc = "read" : at most one command arrives (serial, menu or the abort  *)
(*                 switch) and handleCommand() applies it;                 *)
(*   pc = "exec" : the `switch (state)` runs one slice of work.            *)
(*                                                                         *)
(* Abstractions                                                            *)
(*   - Time: whether a countdown has expired, a sensor has warmed up or    *)
(*     all samples are in is nondeterministic. Any busy slice may also do  *)
(*     nothing (ExecWait).                                                 *)
(*   - Sensor values: whether a reading is unusable (NAN), in range or     *)
(*     still off target is nondeterministic.                               *)
(*   - Targets, durations, calibration, WiFi and the display are left out; *)
(*     "?" stands for every command that does not touch the operation      *)
(*     state (?, w, k, 1, malformed arguments).                            *)
(*   - Float switches only gate the relays in updatePumps(); `pumps` is    *)
(*     the set of pumps the firmware has requested.                        *)
(***************************************************************************)
EXTENDS Naturals, FiniteSets

Pumps    == {"Input", "Output", "Acid", "NutA", "NutB"}
Dosing   == {"Acid", "NutA", "NutB"}
States   == {"IDLE", "CIRCULATE", "EMPTY", "FILL", "MEASURE", "PLOT",
             "ADJUST_PH", "ADJUST_TDS", "ADJUST_ALL", "PUMP_PH",
             "PUMP_NUTRIENT_A", "PUMP_NUTRIENT_B"}
PhSubs   == {"PH_MEASURE", "PH_PUMP", "PH_CIRCULATE", "PH_SETTLE"}
TdsSubs  == {"TDS_MEASURE", "TDS_A_PUMP", "TDS_B_PUMP", "TDS_CIRCULATE",
             "TDS_SETTLE"}
Sensors  == {"NONE", "TDS", "PH"}
\* "n" = both nutrients, "na" / "nb" = one of them, "!" = abort (serial,
\* menu Abort or the abort switch), "?" = anything that leaves state alone.
Commands == {"t", "p", "a", "e", "f", "c", "n", "na", "nb", "u", "m", "l",
             "!", "?"}
BusyCommands == {"!", "?"}   \* handleCommand() also lets 'w' through
MaxRounds == 6               \* maxAdjustmentRounds

VARIABLES
    pc,          \* "read" | "exec"
    state,       \* State state
    phSub,       \* phState
    tdsSub,      \* tdsState
    phRound,     \* phAdjustmentRound
    tdsRound,    \* tdsAdjustmentRound
    both,        \* pumpBothNutrients
    cdRunning,   \* countDown.running (one timer shared by every timed step)
    sensor,      \* poweredSensor
    measStep,    \* measureAllStep
    pumps        \* pumps requested on

vars == <<pc, state, phSub, tdsSub, phRound, tdsRound, both, cdRunning,
          sensor, measStep, pumps>>

TypeOK ==
    /\ pc \in {"read", "exec"}
    /\ state \in States
    /\ phSub \in PhSubs
    /\ tdsSub \in TdsSubs
    /\ phRound \in 0..MaxRounds
    /\ tdsRound \in 0..MaxRounds
    /\ both \in BOOLEAN
    /\ cdRunning \in BOOLEAN
    /\ sensor \in Sensors
    /\ measStep \in {"TDS", "PH"}
    /\ pumps \subseteq Pumps

Init ==   \* setup() clears the shift register and calls stopAllPumps()
    /\ pc = "read"
    /\ state = "IDLE"
    /\ phSub = "PH_MEASURE"
    /\ tdsSub = "TDS_MEASURE"
    /\ phRound = 0
    /\ tdsRound = 0
    /\ both = FALSE
    /\ cdRunning = FALSE
    /\ sensor = "NONE"
    /\ measStep = "TDS"
    /\ pumps = {}

-----------------------------------------------------------------------------
(* Helpers mirroring the firmware                                          *)

ResetAdjustments ==
    /\ phSub' = "PH_MEASURE" /\ phRound' = 0
    /\ tdsSub' = "TDS_MEASURE" /\ tdsRound' = 0

\* stopMeasuring(): measureAllStep = TDS, cancelSampling(), power off
StopMeasuring == sensor' = "NONE" /\ measStep' = "TDS"

\* abortOperation(): stopAllPumps, cancelCountDown, stopMeasuring,
\* resetAdjustments, state = IDLE
AbortOperation ==
    /\ pumps' = {} /\ cdRunning' = FALSE /\ StopMeasuring
    /\ ResetAdjustments /\ state' = "IDLE"

\* startOperation(s): like abortOperation but leaves the pumps alone
StartOperation(s) ==
    /\ cdRunning' = FALSE /\ StopMeasuring
    /\ ResetAdjustments /\ state' = s

-----------------------------------------------------------------------------
(* pc = "read": handleCommand()                                            *)

Target(cmd) ==
    CASE cmd = "t" -> "ADJUST_TDS"   [] cmd = "p" -> "ADJUST_PH"
      [] cmd = "a" -> "ADJUST_ALL"   [] cmd = "e" -> "EMPTY"
      [] cmd = "f" -> "FILL"         [] cmd = "c" -> "CIRCULATE"
      [] cmd = "u" -> "PUMP_PH"      [] cmd = "m" -> "MEASURE"
      [] cmd = "l" -> "PLOT"         [] cmd = "nb" -> "PUMP_NUTRIENT_B"
      [] cmd \in {"n", "na"} -> "PUMP_NUTRIENT_A"

Receive(cmd) ==
    /\ pc = "read" /\ pc' = "exec"
    \* "Operation in progress. Cannot run command."
    /\ state = "IDLE" \/ cmd \in BusyCommands
    /\ CASE cmd = "!" -> AbortOperation /\ UNCHANGED both
         [] cmd = "?" -> UNCHANGED <<state, phSub, tdsSub, phRound, tdsRound,
                                    both, cdRunning, sensor, measStep, pumps>>
         [] OTHER     -> /\ StartOperation(Target(cmd))
                         /\ both' = IF cmd \in {"n", "na"} THEN cmd = "n"
                                                         ELSE both
                         /\ UNCHANGED pumps

NoInput ==
    /\ pc = "read" /\ pc' = "exec"
    /\ UNCHANGED <<state, phSub, tdsSub, phRound, tdsRound, both, cdRunning,
                   sensor, measStep, pumps>>

ReadStep == NoInput \/ \E cmd \in Commands : Receive(cmd)

-----------------------------------------------------------------------------
(* pc = "exec": one slice of the current operation.                        *)
(* Each operation is a set of "progress" steps (ExecGo) plus slices where   *)
(* time passes without visible effect (ExecWait, MeasureWhileRunning).      *)

Exec == pc = "exec" /\ pc' = "read"

\* runCountDown(d, start, stop): first call starts, a later call stops.
\* `on` / `off` are the pumps start() and stop() switch.
TimerStart(on) == ~cdRunning /\ cdRunning' = TRUE  /\ pumps' = pumps \cup on
TimerStop(off) ==  cdRunning /\ cdRunning' = FALSE /\ pumps' = pumps \ off

\* A timed operation in loop(): state `st` runs pumps `p`, then goes to `nx`
TimedOp(st, p, nx) ==
    /\ state = st
    /\ \/ TimerStart(p) /\ UNCHANGED <<state, sensor, measStep>>
       \/ TimerStop(p)  /\ state' = nx /\ UNCHANGED <<sensor, measStep>>
    /\ UNCHANGED <<phSub, tdsSub, phRound, tdsRound, both>>

\* circulate(): input + output pumps, measureAll() while running,
\* stopMeasuring() when the time is up
CircStart == TimerStart({"Input", "Output"}) /\ UNCHANGED <<sensor, measStep>>
CircStop  == TimerStop({"Input", "Output"}) /\ StopMeasuring

GoEmpty    == TimedOp("EMPTY", {"Output"}, "IDLE")
GoFill     == TimedOp("FILL", {"Input"}, "IDLE")
GoPumpAcid == TimedOp("PUMP_PH", {"Acid"}, "CIRCULATE")
GoPumpA    == TimedOp("PUMP_NUTRIENT_A", {"NutA"},
                      IF both THEN "PUMP_NUTRIENT_B" ELSE "CIRCULATE")
GoPumpB    == TimedOp("PUMP_NUTRIENT_B", {"NutB"}, "CIRCULATE")

GoCirculate ==
    /\ state = "CIRCULATE"
    /\ \/ CircStart /\ UNCHANGED state
       \/ CircStop /\ state' = "IDLE"
    /\ UNCHANGED <<phSub, tdsSub, phRound, tdsRound, both>>

\* measureAll() as used by MEASURE: power TDS, read it, power pH, read it
GoMeasure ==
    /\ state = "MEASURE"
    /\ \/ sensor # measStep /\ sensor' = measStep
          /\ UNCHANGED <<measStep, state>>
       \/ sensor = measStep /\ measStep = "TDS" /\ measStep' = "PH"
          /\ UNCHANGED <<sensor, state>>
       \/ sensor = measStep /\ measStep = "PH" /\ measStep' = "TDS"
          /\ sensor' = "NONE" /\ state' = "IDLE"
    /\ UNCHANGED <<phSub, tdsSub, phRound, tdsRound, both, cdRunning, pumps>>

GoPlot ==   \* the countdown's callbacks are empty; measureAll() runs alongside
    /\ state = "PLOT"
    /\ \/ ~cdRunning /\ cdRunning' = TRUE /\ UNCHANGED <<state, sensor, measStep>>
       \/ cdRunning /\ cdRunning' = FALSE /\ StopMeasuring /\ state' = "IDLE"
    /\ UNCHANGED <<phSub, tdsSub, phRound, tdsRound, both, pumps>>

\* measureAll() called while circulating or plotting; never finishes them
MeasureWhileRunning ==
    /\ Exec /\ cdRunning
    /\ \/ state \in {"CIRCULATE", "PLOT"}
       \/ state = "ADJUST_PH" /\ phSub = "PH_CIRCULATE"
       \/ state \in {"ADJUST_TDS", "ADJUST_ALL"} /\ tdsSub = "TDS_CIRCULATE"
    /\ \/ sensor # measStep /\ sensor' = measStep /\ UNCHANGED measStep
       \/ sensor = measStep /\ measStep' = (IF measStep = "TDS" THEN "PH" ELSE "TDS")
          /\ UNCHANGED sensor
    /\ UNCHANGED <<state, phSub, tdsSub, phRound, tdsRound, both, cdRunning, pumps>>

-----------------------------------------------------------------------------
(* phAdjustment(): measure -> dose acid -> circulate -> settle -> measure. *)
(* On FINISHED or FAILED loop() calls abortOperation().                    *)

GoAdjustPh ==
    /\ state = "ADJUST_PH"
    /\ UNCHANGED both
    /\ \/ /\ phSub = "PH_MEASURE" /\ sensor # "PH"   \* powerSensor(PH_SENSOR)
          /\ sensor' = "PH"
          /\ UNCHANGED <<state, phSub, phRound, tdsSub, tdsRound, cdRunning,
                         measStep, pumps>>
       \/ /\ phSub = "PH_MEASURE" /\ sensor = "PH"   \* reading available
          /\ \/ AbortOperation                     \* NAN, reached, or gave up
             \/ /\ phRound < MaxRounds               \* pH still too high
                /\ phRound' = phRound + 1 /\ phSub' = "PH_PUMP"
                /\ sensor' = "NONE"
                /\ UNCHANGED <<state, tdsSub, tdsRound, cdRunning, measStep, pumps>>
       \/ /\ phSub = "PH_PUMP"
          /\ \/ TimerStart({"Acid"}) /\ UNCHANGED phSub
             \/ TimerStop({"Acid"}) /\ phSub' = "PH_CIRCULATE"
          /\ UNCHANGED <<state, phRound, tdsSub, tdsRound, sensor, measStep>>
       \/ /\ phSub = "PH_CIRCULATE"
          /\ \/ CircStart /\ UNCHANGED phSub
             \/ CircStop /\ phSub' = "PH_SETTLE"
          /\ UNCHANGED <<state, phRound, tdsSub, tdsRound>>
       \/ /\ phSub = "PH_SETTLE"
          /\ \/ TimerStart({}) /\ UNCHANGED phSub
             \/ TimerStop({}) /\ phSub' = "PH_MEASURE"
          /\ UNCHANGED <<state, phRound, tdsSub, tdsRound, sensor, measStep>>

(* tdsAdjustment(): measure -> A -> B -> circulate -> settle -> measure.   *)
(* FINISHED in ADJUST_ALL hands over to ADJUST_PH (startOperation);         *)
(* everything else ends in abortOperation().                               *)

GoAdjustTds ==
    /\ state \in {"ADJUST_TDS", "ADJUST_ALL"}
    /\ UNCHANGED <<both>>
    /\ \/ /\ tdsSub = "TDS_MEASURE" /\ sensor # "TDS"
          /\ sensor' = "TDS"
          /\ UNCHANGED <<state, phSub, phRound, tdsSub, tdsRound, cdRunning,
                         measStep, pumps>>
       \/ /\ tdsSub = "TDS_MEASURE" /\ sensor = "TDS"
          /\ \/ AbortOperation                          \* FAILED (NAN)
             \/ /\ state = "ADJUST_TDS" /\ AbortOperation   \* FINISHED
             \/ /\ state = "ADJUST_ALL"                     \* FINISHED
                /\ StartOperation("ADJUST_PH") /\ UNCHANGED pumps
             \/ /\ tdsRound < MaxRounds                 \* TDS still too low
                /\ tdsRound' = tdsRound + 1 /\ tdsSub' = "TDS_A_PUMP"
                /\ sensor' = "NONE"
                /\ UNCHANGED <<state, phSub, phRound, cdRunning, measStep, pumps>>
       \/ /\ tdsSub = "TDS_A_PUMP"
          /\ \/ TimerStart({"NutA"}) /\ UNCHANGED tdsSub
             \/ TimerStop({"NutA"}) /\ tdsSub' = "TDS_B_PUMP"
          /\ UNCHANGED <<state, phSub, phRound, tdsRound, sensor, measStep>>
       \/ /\ tdsSub = "TDS_B_PUMP"
          /\ \/ TimerStart({"NutB"}) /\ UNCHANGED tdsSub
             \/ TimerStop({"NutB"}) /\ tdsSub' = "TDS_CIRCULATE"
          /\ UNCHANGED <<state, phSub, phRound, tdsRound, sensor, measStep>>
       \/ /\ tdsSub = "TDS_CIRCULATE"
          /\ \/ CircStart /\ UNCHANGED tdsSub
             \/ CircStop /\ tdsSub' = "TDS_SETTLE"
          /\ UNCHANGED <<state, phSub, phRound, tdsRound>>
       \/ /\ tdsSub = "TDS_SETTLE"
          /\ \/ TimerStart({}) /\ UNCHANGED tdsSub
             \/ TimerStop({}) /\ tdsSub' = "TDS_MEASURE"
          /\ UNCHANGED <<state, phSub, phRound, tdsRound, sensor, measStep>>

ExecGo ==
    /\ Exec
    /\ \/ GoEmpty \/ GoFill \/ GoPumpAcid \/ GoPumpA \/ GoPumpB
       \/ GoCirculate \/ GoMeasure \/ GoPlot \/ GoAdjustPh \/ GoAdjustTds

\* A slice in which nothing visible happens: timer not expired, sensor
\* warming up, sample interval not reached, or the IDLE case.
ExecWait ==
    /\ Exec
    /\ UNCHANGED <<state, phSub, tdsSub, phRound, tdsRound, both, cdRunning,
                   sensor, measStep, pumps>>

-----------------------------------------------------------------------------

Next == ReadStep \/ ExecGo \/ ExecWait \/ MeasureWhileRunning

\* Time passes and readings complete: if an operation can make progress
\* infinitely often, it eventually does.
Fairness == WF_vars(Next) /\ SF_vars(ExecGo)

Spec == Init /\ [][Next]_vars /\ Fairness

-----------------------------------------------------------------------------
(* Properties                                                              *)

Idle == pc = "read" /\ state = "IDLE"

IdleMeansPumpsOff      == Idle => pumps = {}
IdleMeansTimerFree     == Idle => ~cdRunning
IdleMeansSensorsOff    == Idle => sensor = "NONE"

AcidOnlyWhenDosing ==
    "Acid" \in pumps => \/ state = "PUMP_PH"
                        \/ state = "ADJUST_PH" /\ phSub = "PH_PUMP"
NutrientAOnlyWhenDosing ==
    "NutA" \in pumps => \/ state = "PUMP_NUTRIENT_A"
                        \/ state \in {"ADJUST_TDS", "ADJUST_ALL"} /\ tdsSub = "TDS_A_PUMP"
NutrientBOnlyWhenDosing ==
    "NutB" \in pumps => \/ state = "PUMP_NUTRIENT_B"
                        \/ state \in {"ADJUST_TDS", "ADJUST_ALL"} /\ tdsSub = "TDS_B_PUMP"

OneDosingPumpAtATime == Cardinality(pumps \cap Dosing) <= 1
NoDosingWhileCirculating ==
    {"Input", "Output"} \subseteq pumps => pumps \cap Dosing = {}

\* Sensors are only read inside the operations that use them
SensorsOnlyWhenMeasuring ==
    sensor # "NONE" => \/ state \in {"MEASURE", "PLOT", "CIRCULATE"}
                       \/ state = "ADJUST_PH" /\ phSub \in {"PH_MEASURE", "PH_CIRCULATE"}
                       \/ state \in {"ADJUST_TDS", "ADJUST_ALL"}
                          /\ tdsSub \in {"TDS_MEASURE", "TDS_CIRCULATE"}

\* Every operation ends
OperationsFinish == (state # "IDLE") ~> (state = "IDLE")

=============================================================================
