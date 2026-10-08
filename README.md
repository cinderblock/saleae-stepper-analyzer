# Stepper Motor Coils analyzer for Saleae Logic 2

A low-level analyzer for [Saleae Logic 2](https://www.saleae.com/) that watches the four drive signals of a bipolar stepper motor and shows the **position the motor is being driven to**, live, as you capture.

Connect four digital channels to the two coil pairs (A+/A−, B+/B−). Either the coil terminals (through a divider if the motor voltage exceeds the input range) or the H-bridge logic inputs work. The analyzer reports:

- position in full steps, shaft degrees or revolutions;
- the electrical angle;
- the drive level;
- direction and step rate.

It handles every common drive style with one algorithm:

- **Wave, full and half stepping** with logic-level switching.
- **PWM microstepping and current chopping**, with the PWM frequency detected automatically.
- **De-energized gaps.** Position resumes from the nearest electrical cycle when the drive returns.

## How it works

1. **Duty per window.** Each terminal's duty cycle is averaged over windows that are an exact multiple of the PWM period, which cancels the PWM ripple.
2. **Coil drive.** Each coil's drive is the duty difference across its two terminals, giving a current vector (A, B).
3. **Electrical angle.** The vector's angle is the electrical angle, and one full step is 90 electrical degrees.
4. **Position.** Unwrapping the angle over time gives the position.

For logic-level drive, duty is simply 0 or 1, so the same math covers it.

The PWM period is detected from rising-edge intervals, reading a couple of milliseconds ahead of the decoding point. A capture that starts with the motor idle therefore still detects the PWM before the first energized window is decoded.

## Results

Each result spans a time during which the position held one value at the configured resolution. Zoomed out, a long hold reads as a single bubble, and a move is a dense run of short ones.

| Frame type | Meaning |
| --- | --- |
| `position` | Energized, holding a position |
| `move` | With *Results: Moves and holds*: a run of quick position changes, shown as one result from the held position before it to the one after, e.g. `Move 0 → -142.25 steps (-142.25 steps in 0.8 s, -177.9 steps/s)` |
| `off` | De-energized: the coils carry no drive, so there is no position information |
| `ambiguous` | The electrical angle jumped about 180° between samples (or the motor came back more than 1.5 steps away after being off), so the direction is unknown. Shown as an error. |

Data table columns (FrameV2):

| Column | Meaning |
| --- | --- |
| `position` | Position in the display units |
| `steps` | Position in full steps |
| `electrical_angle` | Degrees (not on moves) |
| `from`, `delta` | Moves only: start position and change, in full steps |
| `drive` | Percent of one coil at full drive (full stepping reads about 141%) |
| `direction` | `+` / `-` for the change that ended the span |
| `rate` | Full steps per second implied by that change (for a move, its mean speed) |

The analyzer's export option writes the same data as CSV.

A fast move is hundreds of narrow frames. When they're too narrow to label, Logic packs the labels side by side instead of placing each one at its frame, so the numbers you see are not where they happened. Use *Moves and holds*, or zoom in, to read moves. You can add the analyzer twice to get both rows.

## Settings

| Setting | Default | Notes |
| --- | --- | --- |
| A+, A−, B+, B− | — | One channel per coil terminal. Swapping A and B, or + and −, reverses or offsets the direction. |
| PWM filtering | Auto-detect | *Fixed frequency* uses the frequency below. *None* is for logic-level drive with no PWM at all. |
| PWM frequency (Hz) | 20000 | Only used with *Fixed frequency*. |
| PWM periods per sample | 2 | More is smoother but follows fast moves less closely. |
| Results | Every position change | *Moves and holds* groups quick successive changes into one move result. |
| Hold time (ms) | 20 | With *Moves and holds*: a position held at least this long is a hold and ends a move. |
| Position resolution | 1/16 step | Smallest change that starts a new result. |
| Off below drive (%) | 10 | Drive below this is treated as de-energized. |
| Display units | Full steps | Or shaft degrees / revolutions. |
| Full steps per revolution | 200 | 200 for 1.8° motors, 400 for 0.9°. |
| Invert direction | off | |
| Start at zero | on | Zero is where the motor sits once first energized. When off, positions follow the absolute electrical angle (position mod 4 = electrical phase). |
| Full-step markers | off | An arrow on A+ at each full-step crossing. |

## Installing

1. Build it (below), or download a build from the GitHub Actions artifacts or releases.
2. In Logic 2, open *Preferences → Custom Low Level Analyzers* and point it at the folder holding `StepperMotorCoilsAnalyzer.dll` (`.so` on macOS and Linux).
3. Restart Logic. *Stepper Motor Coils* appears in the analyzer list.

On Windows, after building, `./tools/install.ps1` copies the DLL to `%LOCALAPPDATA%\Saleae Logic Analyzers\StepperMotorCoils`. Logic locks the DLL it has loaded, so this keeps rebuilds possible while Logic is open. Close Logic before reinstalling over a loaded copy.

With Logic closed, `./tools/install.ps1 -Register` also adds that folder to Logic's custom analyzer paths. It leaves the rest of Logic's settings untouched and keeps a backup of them.

## Building

The build uses CMake and fetches the [Saleae Analyzer SDK](https://github.com/saleae/AnalyzerSDK) automatically.

On Windows (Visual Studio 2022 with the C++ tools, plus Ninja):

```powershell
./build.ps1 -Test   # Release build + unit tests; the analyzer is build/release/Analyzers/StepperMotorCoilsAnalyzer.dll
```

Anywhere else:

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

`.github/workflows/build.yml` (from Saleae's template) builds Windows, macOS and Linux binaries. A tag publishes them as a release.

## Development

- **`src/core/`:** the decoder, with no Analyzer SDK dependency:
  - `EdgeQueue`: buffered edges per channel;
  - `PwmEstimator`: PWM period detection;
  - `StepperPipeline`: windowing;
  - `StepperDecoder`: angle → position segments;
  - `MoveGrouper`: the *Moves and holds* grouping;
  - `StepperWaveform`: a synthetic test program, also used by Logic's demo device.
- **`tests/core_tests.cpp`:** doctest unit tests. These include decoding the synthetic program at several sample rates and PWM frequencies.
- **`tests/simulation_tests.cpp`:** runs the simulation data generator against the real Analyzer SDK library, the way Logic's demo device calls it. Logic only plays an analyzer's simulation when the analyzer was added in the UI before capturing, so the automation API can't test it.
- **`tools/replay_csv.cpp`:** replays a Logic digital CSV export (*File → Export Data → CSV*) through the decoder:

  ```sh
  stepper_replay digital.csv --rate 6250000 --columns 0,1,2,3 --min-ms 50   # add --group-ms 20 for moves and holds
  ```

- **`tools/logic-test-instance.ps1`:** starts a separate Logic 2 instance with the current build loaded, the automation API on port 10431, and device scanning off. It never touches your normal Logic session. `-LoadFile capture.sal` opens a capture in its UI.

## Limitations

- Position is *commanded* position: what the drive signals ask for. A stalled or skipping motor won't show here.
- Without an encoder, re-energizing after the drive was off assumes the rotor stayed within ±2 full steps of where it was left.
- Moves faster than one electrical half-cycle (two full steps) per averaging window are ambiguous. Lower *PWM periods per sample* if that happens.

## License

MIT. Based on Saleae's [SampleAnalyzer](https://github.com/saleae/SampleAnalyzer) template.
