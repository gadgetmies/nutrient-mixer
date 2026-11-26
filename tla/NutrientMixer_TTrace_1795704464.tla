---- MODULE NutrientMixer_TTrace_1795704464 ----
EXTENDS Sequences, TLCExt, Toolbox, Naturals, TLC, NutrientMixer

_expression ==
    LET NutrientMixer_TEExpression == INSTANCE NutrientMixer_TEExpression
    IN NutrientMixer_TEExpression!expression
----

_trace ==
    LET NutrientMixer_TETrace == INSTANCE NutrientMixer_TETrace
    IN NutrientMixer_TETrace!trace
----

_inv ==
    ~(
        TLCGet("level") = Len(_TETrace)
        /\
        phSub = ("PH_MEASURE")
        /\
        pc = ("exec")
        /\
        tdsSub = ("TDS_MEASURE")
        /\
        tdsRound = (0)
        /\
        pumps = ({"Acid"})
        /\
        cdRunning = (TRUE)
        /\
        state = ("ADJUST_TDS")
        /\
        phRound = (0)
    )
----

_init ==
    /\ state = _TETrace[1].state
    /\ pumps = _TETrace[1].pumps
    /\ phRound = _TETrace[1].phRound
    /\ tdsRound = _TETrace[1].tdsRound
    /\ phSub = _TETrace[1].phSub
    /\ pc = _TETrace[1].pc
    /\ tdsSub = _TETrace[1].tdsSub
    /\ cdRunning = _TETrace[1].cdRunning
----

_next ==
    /\ \E i,j \in DOMAIN _TETrace:
        /\ \/ /\ j = i + 1
              /\ i = TLCGet("level")
        /\ state  = _TETrace[i].state
        /\ state' = _TETrace[j].state
        /\ pumps  = _TETrace[i].pumps
        /\ pumps' = _TETrace[j].pumps
        /\ phRound  = _TETrace[i].phRound
        /\ phRound' = _TETrace[j].phRound
        /\ tdsRound  = _TETrace[i].tdsRound
        /\ tdsRound' = _TETrace[j].tdsRound
        /\ phSub  = _TETrace[i].phSub
        /\ phSub' = _TETrace[j].phSub
        /\ pc  = _TETrace[i].pc
        /\ pc' = _TETrace[j].pc
        /\ tdsSub  = _TETrace[i].tdsSub
        /\ tdsSub' = _TETrace[j].tdsSub
        /\ cdRunning  = _TETrace[i].cdRunning
        /\ cdRunning' = _TETrace[j].cdRunning

\* Uncomment the ASSUME below to write the states of the error trace
\* to the given file in Json format. Note that you can pass any tuple
\* to `JsonSerialize`. For example, a sub-sequence of _TETrace.
    \* ASSUME
    \*     LET J == INSTANCE Json
    \*         IN J!JsonSerialize("NutrientMixer_TTrace_1795704464.json", _TETrace)

=============================================================================

 Note that you can extract this module `NutrientMixer_TEExpression`
  to a dedicated file to reuse `expression` (the module in the 
  dedicated `NutrientMixer_TEExpression.tla` file takes precedence 
  over the module `NutrientMixer_TEExpression` below).

---- MODULE NutrientMixer_TEExpression ----
EXTENDS Sequences, TLCExt, Toolbox, Naturals, TLC, NutrientMixer

expression == 
    [
        \* To hide variables of the `NutrientMixer` spec from the error trace,
        \* remove the variables below.  The trace will be written in the order
        \* of the fields of this record.
        state |-> state
        ,pumps |-> pumps
        ,phRound |-> phRound
        ,tdsRound |-> tdsRound
        ,phSub |-> phSub
        ,pc |-> pc
        ,tdsSub |-> tdsSub
        ,cdRunning |-> cdRunning
        
        \* Put additional constant-, state-, and action-level expressions here:
        \* ,_stateNumber |-> _TEPosition
        \* ,_stateUnchanged |-> state = state'
        
        \* Format the `state` variable as Json value.
        \* ,_stateJson |->
        \*     LET J == INSTANCE Json
        \*     IN J!ToJson(state)
        
        \* Lastly, you may build expressions over arbitrary sets of states by
        \* leveraging the _TETrace operator.  For example, this is how to
        \* count the number of times a spec variable changed up to the current
        \* state in the trace.
        \* ,_stateModCount |->
        \*     LET F[s \in DOMAIN _TETrace] ==
        \*         IF s = 1 THEN 0
        \*         ELSE IF _TETrace[s].state # _TETrace[s-1].state
        \*             THEN 1 + F[s-1] ELSE F[s-1]
        \*     IN F[_TEPosition - 1]
    ]

=============================================================================



Parsing and semantic processing can take forever if the trace below is long.
 In this case, it is advised to uncomment the module below to deserialize the
 trace from a generated binary file.

\*
\*---- MODULE NutrientMixer_TETrace ----
\*EXTENDS IOUtils, TLC, NutrientMixer
\*
\*trace == IODeserialize("NutrientMixer_TTrace_1795704464.bin", TRUE)
\*
\*=============================================================================
\*

---- MODULE NutrientMixer_TETrace ----
EXTENDS TLC, NutrientMixer

trace == 
    <<
    ([phSub |-> "PH_MEASURE",pc |-> "read",tdsSub |-> "TDS_MEASURE",tdsRound |-> 0,pumps |-> {},cdRunning |-> FALSE,state |-> "IDLE",phRound |-> 0]),
    ([phSub |-> "PH_MEASURE",pc |-> "exec",tdsSub |-> "TDS_MEASURE",tdsRound |-> 0,pumps |-> {},cdRunning |-> FALSE,state |-> "PUMP_PH",phRound |-> 0]),
    ([phSub |-> "PH_MEASURE",pc |-> "read",tdsSub |-> "TDS_MEASURE",tdsRound |-> 0,pumps |-> {"Acid"},cdRunning |-> TRUE,state |-> "PUMP_PH",phRound |-> 0]),
    ([phSub |-> "PH_MEASURE",pc |-> "exec",tdsSub |-> "TDS_MEASURE",tdsRound |-> 0,pumps |-> {"Acid"},cdRunning |-> TRUE,state |-> "ADJUST_TDS",phRound |-> 0])
    >>
----


=============================================================================

---- CONFIG NutrientMixer_TTrace_1795704464 ----
CONSTANTS
    BootPumps = { }
    BusyCommands = { "t" , "p" , "a" , "e" , "f" , "c" , "n" , "u" , "m" , "l" , "!" , "?" }

INVARIANT
    _inv

CHECK_DEADLOCK
    \* CHECK_DEADLOCK off because of PROPERTY or INVARIANT above.
    FALSE

INIT
    _init

NEXT
    _next

CONSTANT
    _TETrace <- _trace

ALIAS
    _expression
=============================================================================
\* Generated on Thu Nov 26 14:47:44 UTC 2026