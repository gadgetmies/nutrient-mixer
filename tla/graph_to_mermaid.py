#!/usr/bin/env python3
"""Project the TLC state graph (tlc -dump dot,actionlabels) onto Mermaid diagrams.

usage: graph_to_mermaid.py graph.dot > STATE_MACHINE.md

TLC's graph is over every variable (pc, sub-states, counters, pumps, ...).
This collapses it onto:
  1. the top-level `state` variable of loop(), and
  2. the pH / TDS adjustment sub-machines,
keeping only transitions that change the projected variable.
"""
import re
import sys
from collections import defaultdict

NODE = re.compile(r'^(-?\d+) \[label="(.*?)"(?:,tooltip|,style|\])')
EDGE = re.compile(r'^(-?\d+) -> (-?\d+) \[label="((?:[^"\\]|\\.)*)"')
VAR = re.compile(r'/\\\\ (\w+) = (.*?)(?:\\n|$)')

COMMANDS = {
    "t": "t", "p": "p", "a": "a / menu Adjust all",
    "e": "e / menu Empty", "f": "f / menu Fill", "c": "c / menu Circulate",
    "n": "n", "na": "n a / menu Pump A", "nb": "n b / menu Pump B",
    "u": "u / menu Pump acid", "m": "m / menu Measure", "l": "l",
    "!": "! / menu Abort / abort switch",
}


def parse(path):
    nodes, edges = {}, []
    for line in open(path):
        line = line.strip()
        m = EDGE.match(line)
        if m:
            edges.append((m.group(1), m.group(2), m.group(3).replace('\\"', '"')))
            continue
        m = NODE.match(line)
        if m:
            nodes[m.group(1)] = {k: v.replace('\\"', "").strip('"')
                                 for k, v in VAR.findall(m.group(2))}
    return nodes, edges


TOP_LABELS = {
    ("ADJUST_ALL", "ADJUST_PH"): "TDS reached or 6 rounds",
    ("ADJUST_ALL", "IDLE"): "TDS unreadable",
    ("ADJUST_TDS", "IDLE"): "reached, 6 rounds or unreadable",
    ("ADJUST_PH", "IDLE"): "reached, 6 rounds or unreadable",
    ("PUMP_NUTRIENT_A", "PUMP_NUTRIENT_B"): "dose done (n)",
    ("PUMP_NUTRIENT_A", "CIRCULATE"): "dose done (n a)",
    ("PUMP_NUTRIENT_B", "CIRCULATE"): "dose done",
    ("PUMP_PH", "CIRCULATE"): "dose done",
    ("MEASURE", "IDLE"): "TDS and pH read",
    ("PLOT", "IDLE"): "plot time over",
    ("CIRCULATE", "IDLE"): "time over",
    ("EMPTY", "IDLE"): "time over",
    ("FILL", "IDLE"): "time over",
}


def top_label(action, src, dst):
    m = re.match(r'Receive\("(.+?)"\)', action)
    if m:
        return COMMANDS.get(m.group(1), m.group(1))
    return TOP_LABELS.get((src, dst), action)


def top_level(nodes, edges):
    labels = defaultdict(set)
    for a, b, act in edges:
        sa, sb = nodes[a]["state"], nodes[b]["state"]
        if sa == sb or (act == 'Receive("!")' and sa != "IDLE"):
            continue  # abort-from-anywhere is drawn once, as a note
        labels[(sa, sb)].add(top_label(act, sa, sb))
    out = ["stateDiagram-v2", "    direction LR", "    [*] --> IDLE"]
    for (a, b), ls in sorted(labels.items()):
        out.append(f"    {a} --> {b}: {' / '.join(sorted(ls))}")
    out.append("    note left of IDLE\n        While busy, only ! (and ?, w) is accepted.<br/>Abort returns to IDLE from any state.\n    end note")
    return "\n".join(out)


SUB_LABELS = {
    ("PH_MEASURE", "PH_PUMP"): "pH too high, round < 6",
    ("PH_PUMP", "PH_CIRCULATE"): "acid dose done",
    ("PH_CIRCULATE", "PH_SETTLE"): "circulation time over",
    ("PH_SETTLE", "PH_MEASURE"): "settle time over",
    ("TDS_MEASURE", "TDS_A_PUMP"): "TDS too low, round < 6",
    ("TDS_A_PUMP", "TDS_B_PUMP"): "A dose done",
    ("TDS_B_PUMP", "TDS_CIRCULATE"): "B dose done",
    ("TDS_CIRCULATE", "TDS_SETTLE"): "circulation time over",
    ("TDS_SETTLE", "TDS_MEASURE"): "settle time over",
}


def sub_machine(nodes, edges, top_states, sub_var, title):
    trans, exits = set(), defaultdict(set)
    for a, b, act in edges:
        na, nb = nodes[a], nodes[b]
        if na["state"] not in top_states or act.startswith("Receive") or act == "NoInput":
            continue
        if nb["state"] != na["state"]:
            exits[na[sub_var]].add(f"{na['state']} → {nb['state']}")
        elif na[sub_var] != nb[sub_var]:
            trans.add((na[sub_var], nb[sub_var]))
    first = "PH_MEASURE" if sub_var == "phSub" else "TDS_MEASURE"
    out = ["stateDiagram-v2", f"    state \"{title}\" as SUB {{", f"    [*] --> {first}"]
    for a, b in sorted(trans):
        out.append(f"    {a} --> {b}: {SUB_LABELS.get((a, b), '')}")
    for s in sorted(exits):
        out.append(f"    {s} --> [*]: reached, 6 rounds or unreadable")
    out.append("    }")
    all_exits = sorted(set().union(*exits.values()))
    out.append(f"    note right of SUB\n        on exit: {'<br/>'.join(all_exits)}\n    end note")
    return "\n".join(out)


def main():
    nodes, edges = parse(sys.argv[1])
    print(f"<!-- generated from a TLC graph with {len(nodes)} states and {len(edges)} transitions -->")
    for name, body in [
        ("Top-level `loop()` state", top_level(nodes, edges)),
        ("`pHadjustment()` sub-machine",
         sub_machine(nodes, edges, {"ADJUST_PH"}, "phSub", "ADJUST_PH")),
        ("`TDSadjustment()` sub-machine",
         sub_machine(nodes, edges, {"ADJUST_TDS", "ADJUST_ALL"},
                     "tdsSub", "ADJUST_TDS / ADJUST_ALL")),
    ]:
        print(f"\n### {name}\n\n```mermaid\n{body}\n```")


if __name__ == "__main__":
    main()
