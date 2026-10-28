# Scenarios

Scenario scripts drive the simulated device without the UI and check the outcome. `make test` runs all of
them; a single one runs with `build/nutrient-mixer-sim --script scenarios/auto_adjust.sim` (add `--quiet` to hide
the serial output). Each run starts from erased flash and the default world, as fast as the host can run it, and
exits with code 1 when a check fails.

One command per line; `#` starts a comment (except in `serial` lines). Times are seconds of device time.

| Command | |
|---|---|
| `wait 10` | Let 10 s pass |
| `serial a 1000 6.0` | Send a line to the serial port |
| `until <path> <op> <value> [timeout 600]` | Wait until the condition holds (default timeout 3600 s) |
| `expect <path> <op> <value>` | Fail unless the condition holds now |
| `until-serial <text> [timeout 30]` | Wait for a serial output line containing the text |
| `expect-serial <text>` | Fail unless such a line has been printed |
| `click encoder` / `press abort` / `release abort` | Buttons: `encoder` (knob push), `abort`, `io0`, `io35`; `click` takes an optional hold time |
| `rotate 3` / `rotate -1` | Turn the knob (positive = clockwise = menu down) |
| `set <parameter> <value>` | Change a world parameter (keys as in the UI tooltips / `src/params.cpp`), e.g. `set fault.mixerFloat 2` |
| `world reset 1` / `world refill 0` / `world volume mixer 8.5` | World actions (presets: 0 used solution, 1 tap water, 2 empty mixer; bottles: 0 pH Down, 1 A, 2 B) |
| `wifi network HomeNet off` / `wifi scan` / `wifi join <ssid> <password>` / `wifi submit <ssid> <password>` | Router and phone actions |
| `print <path> ...` | Print values |
| `screenshot file.png` | Save the T-Display contents |

Paths point into the state the UI receives, for example `firmware.state`, `firmware.ph`, `firmware.tdsRound`,
`world.mixer.tds`, `world.plant.volume`, `world.peristaltic.3.on`, `board.leds.LED1`, `board.relays.0`,
`wifi.station`. Operators: `==`, `!=`, `<`, `<=`, `>`, `>=`, `contains`. Values the firmware shows as text
(such as `firmware.ph`) compare as numbers.
