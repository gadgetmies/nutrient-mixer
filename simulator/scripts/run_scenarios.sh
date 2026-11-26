#!/bin/sh
# Runs every scenario script headless and reports which passed. Exit code 1 if any failed.
PROGRAM="$1"
DIRECTORY="$2"
failed=0
for scenario in "$DIRECTORY"/*.sim; do
  if output=$("$PROGRAM" --script "$scenario" --quiet 2>&1); then
    echo "PASS $scenario"
  else
    echo "FAIL $scenario"
    echo "$output" | grep '\[scenario\]' | sed 's/^/     /'
    failed=1
  fi
done
exit $failed
