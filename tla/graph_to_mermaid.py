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
    "t": "t (adjust TDS)", "p": "p (adjust pH)", "a": "a (adjust all)",
    "e": "e / menu Empty", "f": "f / menu Fill", "c": "c / menu Circulate",
    "n": "n (pump nutrients)", "u": "u (pump acid)", "m": "m (measure)",
    "l": "l (plot)",
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


def top_label(action, src, dst, s):
    m = re.match(r'Receive\("(.)"\)', action)
    if m:
        return COMMANDS.get(m.group(1), m.group(1))
    if src == "ADJUST_ALL" and dst == "ADJUST_PH":
        return "TDS in range"
    if src == "PUMP_PH":
        return "acid timer expired"
    if src == "PUMP_NUTRIENT":
        return "target 0 ⇒ done at once"
    if src in ("ADJUST_PH", "ADJUST_TDS", "ADJUST_ALL"):
        return "in range or 6 rounds"
    if src == "MEASURE":
        return "readings printed"
    if src in ("CIRCULATE", "EMPTY", "FILL", "PLOT"):
        return "timer expired"
    if src == "ABORT":
        return "pumps off + reboot"
    return action


def top_level(nodes, edges):
    labels = defaultdict(set)
    for a, b, act in edges:
        sa, sb = nodes[a]["state"], nodes[b]["state"]
        if sa == sb or (act == 'Receive("!")' and sa != "IDLE"):
            continue  # abort-from-anywhere is drawn once, as a note
        labels[(sa, sb)].add(top_label(act, sa, sb, nodes[a]))
    out = ["stateDiagram-v2", "    direction LR", "    [*] --> IDLE"]
    for (a, b), ls in sorted(labels.items()):
        out.append(f"    {a} --> {b}: {' / '.join(sorted(ls))}")
    out.append("    note right of ABORT: '!' is accepted in every state")
    return "\n".join(out)


SUB_LABELS = {
    ("PH_MEASURE", "PH_PUMP"): "pH above target, round < 6",
    ("PH_PUMP", "PH_CIRCULATE"): "acid timer expired",
    ("PH_CIRCULATE", "PH_MEASURE"): "circulation timer expired",
    ("TDS_MEASURE", "TDS_A_PUMP"): "TDS below target, round < 6",
    ("TDS_A_PUMP", "TDS_B_PUMP"): "nutrient A timer expired",
    ("TDS_B_PUMP", "TDS_CIRCULATE"): "nutrient B timer expired",
    ("TDS_CIRCULATE", "TDS_MEASURE"): "circulation timer expired",
}


def sub_machine(nodes, edges, top_states, sub_var, title):
    trans, exits = set(), defaultdict(set)
    for a, b, act in edges:
        na, nb = nodes[a], nodes[b]
        if na["state"] not in top_states or act.startswith("Receive") or act == "NoInput":
            continue
        if nb["state"] != na["state"] and nb["state"] != "ABORT":
            exits[na[sub_var]].add(f"{na['state']} → {nb['state']}")
        elif na[sub_var] != nb[sub_var]:
            trans.add((na[sub_var], nb[sub_var]))
    first = "PH_MEASURE" if sub_var == "phSub" else "TDS_MEASURE"
    out = ["stateDiagram-v2", f"    state \"{title}\" as SUB {{", f"    [*] --> {first}"]
    for a, b in sorted(trans):
        out.append(f"    {a} --> {b}: {SUB_LABELS.get((a, b), '')}")
    for s in sorted(exits):
        out.append(f"    {s} --> [*]: in range or round > 5")
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
         sub_machine(nodes, edges, {"ADJUST_TDS", "ADJUST_ALL", "PUMP_NUTRIENT"},
                     "tdsSub", "ADJUST_TDS / ADJUST_ALL / PUMP_NUTRIENT")),
    ]:
        print(f"\n### {name}\n\n```mermaid\n{body}\n```")


if __name__ == "__main__":
    main()
