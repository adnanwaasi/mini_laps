# F1 Hotlap

A lightweight top-down time-trial racer in C++ with SDL2. Pick a circuit, drive laps, stay inside the track limits, and chase your best lap. Your best lap on each circuit is saved, and a ghost car replays it while you drive.

The whole game is one source file (`src/main.cpp`) with SDL2 as its only dependency. It has no image, font or sound assets: the engine note, tyre squeal, kerb rumble and impacts are all synthesised as you drive.

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

Self-test (runs headless: on every circuit it checks the layout, then an autopilot drives 3 laps to check lap timing, sectors, the live delta, the ghost and saving):

```sh
./build/f1_hotlap --selftest   # or: ctest --test-dir build
```

## Circuits

Choose the circuit at the top of the settings menu (Esc). Each one keeps its own best lap and ghost.

| Circuit | Character |
|---|---|
| Parkland | The original: a mix of long sweepers, a hairpin and a fast back section |
| Coastline | Fast. Long straights and high-speed sweepers, nearly flat out on the right line |
| Harbour | Tight street circuit: right-angle corners and a chicane |

## Controls

| Action | Keyboard | Gamepad |
|---|---|---|
| Throttle | W / ↑ | RT |
| Brake / reverse | S / ↓ | LT |
| Steer | A D / ← → | Left stick |
| Handbrake (drift) | Space | A |
| Restart lap (back behind the line, timer cleared) | R / Enter | B / Start |
| Cycle camera mode | C | Y |
| Settings menu | Esc | Back |
| Toggle ghost | G | |
| Toggle racing line | L | |
| Pause | P | |
| Fullscreen | F11 | |

## Settings

Press **Esc** to open the settings menu; the game pauses while it's open. With the mouse, click `<` / `>` or scroll to change numbers, and click a row to toggle it. With the keyboard, use ↑/↓ to choose, ←/→ to change, and Enter to toggle. Changes are saved straight away to `settings.cfg` next to the executable.

| Setting | Range | What it does |
|---|---|---|
| Track | Parkland / Coastline / Harbour | Switching starts a fresh session on that circuit |
| Steering sensitivity | 50–150% | How far the wheels turn at full lock. Lower is calmer, higher is twitchier |
| Steering speed | 50–200% | How fast the wheel moves when you press or release a key |
| Gamepad deadzone | 0–30% | How much stick movement is ignored around the centre |
| Camera mode | Follow / Chase / Locked / Helicopter | See below (also cycles with C) |
| Camera zoom | 60–150% | Lower shows more of the track |
| Camera look-ahead | 0–150% | How far the camera leads the car in the direction you're going |
| Zoom out with speed | on/off | Pulls the camera back at high speed |
| Speed units | km/h / mph | |
| Sound volume | 0–100% | 0 turns the sound off |
| Racing line | on/off | Shows the fastest line ahead (also L). Green: accelerate, yellow: at the grip limit, red: brake |
| Ghost car, minimap, skid marks | on/off | |
| Fullscreen, VSync | on/off | |
| Reset best lap | | Click or press Enter twice to wipe your record and ghost |

The menu also has Resume and Quit.

### Camera modes

- **Follow:** the default. North stays up, and the camera leads the car in the direction it's moving.
- **Chase:** the view rotates so your car always points up the screen, sitting low with the road ahead visible. This feels most like driving.
- **Locked:** the car stays fixed in the centre of the screen. There's no look-ahead and no zoom with speed.
- **Helicopter:** a high, wide view that shows the corners well in advance.

Camera zoom applies in every mode. Look-ahead and zoom out with speed apply where they make sense.

## HUD

The dash at the bottom shows speed, gear and throttle/brake bars. The 15 rev lights fill green, red, then blue as the revs rise, and flash blue at the shift point. The gearbox is automatic.

## Rules

- You start on an out lap. Timing begins when you cross the chequered line.
- **Track limits:** kerbs count as track. If the car goes fully past the kerb, the lap is marked **INVALID** and doesn't count toward your record. Grass slows you down, and tyre walls stop you.
- Press R at any time to abandon the lap and start a fresh one from behind the line. Driving backwards over the line invalidates the lap.
- Each lap is split into 3 sectors (yellow lines). The sector deltas compare against your best lap: green is faster, red is slower. A sector time turns **purple** when it's the fastest you've ever driven that sector.
- Once you have a best lap, the **DELTA** box at the top shows the live gap to it at your current spot on track (green means you're ahead).
- **IDEAL** is your theoretical best: the sum of your best-ever sectors.
- Your best valid lap, its ghost and your best sectors are saved next to the executable, one file per circuit (`best_lap.dat` for Parkland, `best_lap_coastline.dat`, `best_lap_harbour.dat`). Reset the current circuit's record from the settings menu, or delete the file.
