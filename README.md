# F1 Hotlap

A lightweight top-down time-trial racer in C++ with SDL2. Drive laps around the circuit, stay inside the track limits, and chase your best lap. Your best lap is saved, and a ghost car replays it while you drive.

The whole game is one source file (`src/main.cpp`) with SDL2 as its only dependency. It has no image or font assets.

## Build & run

```sh
cmake -S . -B build
cmake --build build
./build/f1_hotlap
```

Or without CMake:

```sh
g++ -std=c++17 -O2 src/main.cpp $(pkg-config --cflags --libs sdl2) -o f1_hotlap
```

Or just use the script (it builds when needed, then launches):

```sh
./run.sh
```

Self-test (runs headless: an autopilot drives 3 laps to check lap timing, sectors and the ghost):

```sh
./build/f1_hotlap --selftest   # or: ctest --test-dir build
```

## Controls

| Action | Keyboard | Gamepad |
|---|---|---|
| Throttle | W / ↑ | RT |
| Brake / reverse | S / ↓ | LT |
| Steer | A D / ← → | Left stick |
| Handbrake (drift) | Space | A |
| Restart lap (back behind the line, timer cleared) | R / Enter | B / Start |
| Settings menu | Esc | Back |
| Toggle ghost | G | |
| Pause | P | |
| Fullscreen | F11 | |

## Settings

Press **Esc** to open the settings menu; the game pauses while it's open. With the mouse, click `<` / `>` or scroll to change numbers, and click a row to toggle it. With the keyboard, use ↑/↓ to choose, ←/→ to change, and Enter to toggle. Changes are saved straight away to `settings.cfg` next to the executable.

| Setting | Range | What it does |
|---|---|---|
| Steering sensitivity | 50–150% | How far the wheels turn at full lock. Lower is calmer, higher is twitchier |
| Steering speed | 50–200% | How fast the wheel moves when you press or release a key |
| Gamepad deadzone | 0–30% | How much stick movement is ignored around the centre |
| Camera zoom | 60–150% | Lower shows more of the track |
| Camera look-ahead | 0–150% | How far the camera leads the car in the direction you're going |
| Zoom out with speed | on/off | Pulls the camera back at high speed |
| Speed units | km/h / mph | |
| Ghost car, minimap, skid marks | on/off | |
| Fullscreen, VSync | on/off | |
| Reset best lap | | Click or press Enter twice to wipe your record and ghost |

The menu also has Resume and Quit.

## Rules

- You start on an out lap. Timing begins when you cross the chequered line.
- **Track limits:** kerbs count as track. If the car goes fully past the kerb, the lap is marked **INVALID** and doesn't count toward your record. Grass slows you down, and tyre walls stop you.
- Press R at any time to abandon the lap and start a fresh one from behind the line. Driving backwards over the line invalidates the lap.
- Each lap is split into 3 sectors (yellow lines). The sector deltas compare against your best lap: green is faster, red is slower.
- Your best valid lap and its ghost are saved to `best_lap.dat` next to the executable. Reset it from the settings menu, or delete the file.
