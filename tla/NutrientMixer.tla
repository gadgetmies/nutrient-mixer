--------------------------- MODULE NutrientMixer ---------------------------
(***************************************************************************)
(* Model of the control state machine in nutrient_mixer.ino.               *)
(*                                                                         *)
(* One iteration of Arduino loop() is split into two TLA+ steps:           *)
(*   pc = "read" : a command (serial or menu) may arrive and is applied    *)
(*                 by the first `switch (command)`;                        *)
(*   pc = "exec" : the second `switch (state)` runs one slice of work.     *)
(*                                                                         *)
(* Abstractions                                                            *)
(*   - Time: CountDown expiry is nondeterministic (remaining() = 0 or not).*)
(*   - Sensors: whether a reading is "in range" is nondeterministic.       *)
(*   - The single `static CountDown` / `static bool running` inside        *)
(*     runCountDown() is SHARED by every caller; modelled as `cdRunning`.  *)
(*   - reset() (null function pointer call) is modelled as a reboot that   *)
(*     re-initialises every static variable.                               *)
(***************************************************************************)
EXTENDS Naturals

CONSTANTS
    BootPumps,        \* pumps energised by setup() (input/output relays HIGH)
    BusyCommands      \* commands that can still arrive while state # IDLE

Pumps    == {"Input", "Output", "Acid", "NutA", "NutB"}
States   == {"IDLE", "CIRCULATE", "EMPTY", "FILL", "MEASURE", "PLOT",
             "ADJUST_PH", "ADJUST_TDS", "ADJUST_ALL", "PUMP_PH",
             "PUMP_NUTRIENT", "ABORT"}
PhSubs   == {"PH_MEASURE", "PH_PUMP", "PH_CIRCULATE"}
TdsSubs  == {"TDS_MEASURE", "TDS_A_PUMP", "TDS_B_PUMP", "TDS_CIRCULATE"}
\* Characters handled by `switch (command)`; "none" = no input this loop,
\* "?" = help / unknown command (both leave `state` untouched).
Commands == {"t", "p", "a", "e", "f", "c", "n", "u", "m", "l", "!", "?"}
MaxRound == 6   \* rounds stop once adjustmentRound > 5

VARIABLES
    pc,          \* "read" | "exec"
    state,       \* loop()'s static State
    phSub,       \* pHadjustment()'s static state
    tdsSub,      \* TDSadjustment()'s static state
    phRound,     \* pHadjustment()'s static adjustmentRound
    tdsRound,    \* TDSadjustment()'s static adjustmentRound
    cdRunning,   \* runCountDown()'s static `running`
    pumps        \* set of energised pumps

vars == <<pc, state, phSub, tdsSub, phRound, tdsRound, cdRunning, pumps>>

TypeOK ==
    /\ pc \in {"read", "exec"}
    /\ state \in States
    /\ phSub \in PhSubs
    /\ tdsSub \in TdsSubs
    /\ phRound \in 0..MaxRound
    /\ tdsRound \in 0..MaxRound
    /\ cdRunning \in BOOLEAN
    /\ pumps \subseteq Pumps

Init ==
    /\ pc = "read"
    /\ state = "IDLE"
    /\ phSub = "PH_MEASURE"
    /\ tdsSub = "TDS_MEASURE"
    /\ phRound = 0
    /\ tdsRound = 0
    /\ cdRunning = FALSE
    /\ pumps = BootPumps

-----------------------------------------------------------------------------
(* Phase 1: switch (command)                                               *)

NextState(cmd) ==
    CASE cmd = "t" -> "ADJUST_TDS"
      [] cmd = "p" -> "ADJUST_PH"
      [] cmd = "a" -> "ADJUST_ALL"
      [] cmd = "e" -> "EMPTY"
      [] cmd = "f" -> "FILL"
      [] cmd = "c" -> "CIRCULATE"
      [] cmd = "n" -> "PUMP_NUTRIENT"
      [] cmd = "u" -> "PUMP_PH"
      [] cmd = "m" -> "MEASURE"
      [] cmd = "l" -> "PLOT"
      [] cmd = "!" -> "ABORT"
      [] OTHER     -> state

\* The firmware prints "Operation in progress" when busy but still applies
\* the command; BusyCommands controls which commands may arrive then.
Receive(cmd) ==
    /\ pc = "read"
    /\ state = "IDLE" \/ cmd \in BusyCommands
    /\ state' = NextState(cmd)
    /\ pc' = "exec"
    /\ UNCHANGED <<phSub, tdsSub, phRound, tdsRound, cdRunning, pumps>>

NoInput ==
    /\ pc = "read"
    /\ pc' = "exec"
    /\ UNCHANGED <<state, phSub, tdsSub, phRound, tdsRound, cdRunning, pumps>>

CmdTDS      == Receive("t")
CmdPH       == Receive("p")
CmdAll      == Receive("a")
CmdEmpty    == Receive("e")
CmdFill     == Receive("f")
CmdCirc     == Receive("c")
CmdNutrient == Receive("n")
CmdAcid     == Receive("u")
CmdMeasure  == Receive("m")
CmdPlot     == Receive("l")
CmdAbort    == Receive("!")
CmdOther    == Receive("?")

-----------------------------------------------------------------------------
(* runCountDown(start, stop) as three outcomes.                            *)
(*   CdStart(on): !running -> running := TRUE, start() energises `on`.     *)
(*   CdWait     : running, remaining() > 0 -> nothing.                     *)
(*   CdStop(off): running, remaining() = 0 -> running := FALSE,            *)
(*                stop() de-energises `off`, returns TRUE.                 *)

CdStart(on) == ~cdRunning /\ cdRunning' = TRUE  /\ pumps' = pumps \cup on
CdWait      ==  cdRunning /\ UNCHANGED <<cdRunning, pumps>>
CdStop(off) ==  cdRunning /\ cdRunning' = FALSE /\ pumps' = pumps \ off

\* Every exec step returns to "read"; `done` returns the machine to IDLE.
Exec == pc = "exec" /\ pc' = "read"

\* Timed actions that end in IDLE (CIRCULATE, EMPTY, FILL).
Timed(st, set) ==
    /\ Exec /\ state = st
    /\ \/ CdStart(set) /\ UNCHANGED state
       \/ CdWait       /\ UNCHANGED state
       \/ CdStop(set)  /\ state' = "IDLE"
    /\ UNCHANGED <<phSub, tdsSub, phRound, tdsRound>>

ExecCirculate == Timed("CIRCULATE", {"Input", "Output"})
ExecEmpty     == Timed("EMPTY", {"Output"})
ExecFill      == Timed("FILL", {"Input"})

ExecMeasure ==
    /\ Exec /\ state = "MEASURE" /\ state' = "IDLE"
    /\ UNCHANGED <<phSub, tdsSub, phRound, tdsRound, cdRunning, pumps>>

ExecPlot ==   \* start/stop callbacks are no-ops
    /\ Exec /\ state = "PLOT"
    /\ \/ ~cdRunning /\ cdRunning' = TRUE  /\ UNCHANGED state
       \/  cdRunning /\ UNCHANGED <<cdRunning, state>>
       \/  cdRunning /\ cdRunning' = FALSE /\ state' = "IDLE"
    /\ UNCHANGED <<phSub, tdsSub, phRound, tdsRound, pumps>>

ExecPumpAcid ==   \* pumppH(2), then CIRCULATE
    /\ Exec /\ state = "PUMP_PH"
    /\ \/ CdStart({"Acid"}) /\ UNCHANGED state
       \/ CdWait            /\ UNCHANGED state
       \/ CdStop({"Acid"})  /\ state' = "CIRCULATE"
    /\ UNCHANGED <<phSub, tdsSub, phRound, tdsRound>>

ExecAbort ==   \* stop everything, then reset() reboots the MCU
    /\ Exec /\ state = "ABORT"
    /\ pumps' = {} /\ state' = "IDLE"
    /\ phSub' = "PH_MEASURE" /\ tdsSub' = "TDS_MEASURE"
    /\ phRound' = 0 /\ tdsRound' = 0 /\ cdRunning' = FALSE

ExecIdle ==
    /\ Exec /\ state = "IDLE"
    /\ UNCHANGED <<state, phSub, tdsSub, phRound, tdsRound, cdRunning, pumps>>

-----------------------------------------------------------------------------
(* pHadjustment(target): one slice. `onDone` is the loop state to go to.   *)

PhSlice(onDone) ==
    \/ /\ phSub = "PH_MEASURE"
       /\ \/ /\ phRound < MaxRound               \* pH too high: dose acid
             /\ phSub' = "PH_PUMP" /\ phRound' = phRound + 1
             /\ UNCHANGED state
          \/ /\ phRound' = 0 /\ state' = onDone  \* in range, or round > 5
             /\ UNCHANGED phSub
       /\ UNCHANGED <<cdRunning, pumps>>
    \/ /\ phSub = "PH_PUMP"
       /\ \/ CdStart({"Acid"}) /\ UNCHANGED phSub
          \/ CdWait            /\ UNCHANGED phSub
          \/ CdStop({"Acid"})  /\ phSub' = "PH_CIRCULATE"
       /\ UNCHANGED <<state, phRound>>
    \/ /\ phSub = "PH_CIRCULATE"
       /\ \/ CdStart({"Input", "Output"}) /\ UNCHANGED phSub
          \/ CdWait                       /\ UNCHANGED phSub
          \/ CdStop({"Input", "Output"})  /\ phSub' = "PH_MEASURE"
       /\ UNCHANGED <<state, phRound>>

ExecAdjustPH ==
    /\ Exec /\ state = "ADJUST_PH"
    /\ PhSlice("IDLE")
    /\ UNCHANGED <<tdsSub, tdsRound>>

-----------------------------------------------------------------------------
(* TDSadjustment(target). `zeroTarget` is TRUE for the PUMP_NUTRIENT call, *)
(* which uses the default target 0 and therefore finishes at TDS_MEASURE.  *)

TdsSlice(onDone, zeroTarget) ==
    \/ /\ tdsSub = "TDS_MEASURE"
       /\ \/ /\ ~zeroTarget /\ tdsRound < MaxRound   \* TDS too low: dose
             /\ tdsSub' = "TDS_A_PUMP" /\ tdsRound' = tdsRound + 1
             /\ UNCHANGED state
          \/ /\ tdsRound' = 0 /\ state' = onDone
             /\ UNCHANGED tdsSub
       /\ UNCHANGED <<cdRunning, pumps>>
    \/ /\ tdsSub = "TDS_A_PUMP"
       /\ \/ CdStart({"NutA"}) /\ UNCHANGED tdsSub
          \/ CdWait            /\ UNCHANGED tdsSub
          \/ CdStop({"NutA"})  /\ tdsSub' = "TDS_B_PUMP"
       /\ UNCHANGED <<state, tdsRound>>
    \/ /\ tdsSub = "TDS_B_PUMP"
       /\ \/ CdStart({"NutB"}) /\ UNCHANGED tdsSub
          \/ CdWait            /\ UNCHANGED tdsSub
          \/ CdStop({"NutB"})  /\ tdsSub' = "TDS_CIRCULATE"
       /\ UNCHANGED <<state, tdsRound>>
    \/ /\ tdsSub = "TDS_CIRCULATE"
       /\ \/ CdStart({"Input", "Output"}) /\ UNCHANGED tdsSub
          \/ CdWait                       /\ UNCHANGED tdsSub
          \/ CdStop({"Input", "Output"})  /\ tdsSub' = "TDS_MEASURE"
       /\ UNCHANGED <<state, tdsRound>>

ExecAdjustTDS ==
    /\ Exec /\ state = "ADJUST_TDS"
    /\ TdsSlice("IDLE", FALSE)
    /\ UNCHANGED <<phSub, phRound>>

ExecAdjustAll ==   \* TDS first, then hand over to ADJUST_PH
    /\ Exec /\ state = "ADJUST_ALL"
    /\ TdsSlice("ADJUST_PH", FALSE)
    /\ UNCHANGED <<phSub, phRound>>

ExecPumpNutrient ==   \* TDSadjustment() with target 0, then CIRCULATE
    /\ Exec /\ state = "PUMP_NUTRIENT"
    /\ TdsSlice("CIRCULATE", TRUE)
    /\ UNCHANGED <<phSub, phRound>>

-----------------------------------------------------------------------------

Next ==
    \/ NoInput
    \/ CmdTDS \/ CmdPH \/ CmdAll \/ CmdEmpty \/ CmdFill \/ CmdCirc
    \/ CmdNutrient \/ CmdAcid \/ CmdMeasure \/ CmdPlot \/ CmdAbort \/ CmdOther
    \/ ExecIdle \/ ExecCirculate \/ ExecEmpty \/ ExecFill \/ ExecMeasure
    \/ ExecPlot \/ ExecPumpAcid \/ ExecAbort
    \/ ExecAdjustPH \/ ExecAdjustTDS \/ ExecAdjustAll \/ ExecPumpNutrient

Spec == Init /\ [][Next]_vars

-----------------------------------------------------------------------------
(* Safety properties                                                       *)

\* When the machine is idle, nothing should be pumping.
IdleMeansPumpsOff == (pc = "read" /\ state = "IDLE") => pumps = {}

\* The shared countdown is never left armed while idle.
IdleMeansCountdownFree == (pc = "read" /\ state = "IDLE") => ~cdRunning

\* Dosing pumps only run inside a dosing sub-state.
AcidOnlyWhenDosing ==
    "Acid" \in pumps => \/ state = "PUMP_PH"
                        \/ state = "ADJUST_PH" /\ phSub = "PH_PUMP"

=============================================================================
