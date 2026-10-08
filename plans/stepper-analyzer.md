# Stepper analyzer for Logic 2

## Goal

A Logic 2 low-level analyzer that takes the four digital drive signals of a bipolar stepper and shows the **expected motor position** live in Logic. The four signals are two coil pairs: A+/A− and B+/B−. Position is reported as signed full steps (or degrees / revolutions), alongside electrical angle, drive level and step rate.

Related research (Logic 2 API limits, other plugin ideas): `../plans/logic2-derived-signals.md`. That file lives in the parent folder, which is not a git repo.

## Environment / context

- **Repo:** `C:\Users\camer\git\Personal Projects\Saleae Logic Plugins\stepper-analyzer`. It has its own git repo on branch `master` and no remote yet.
- **Base:** `saleae/SampleAnalyzer` at `f31e2d3` (2026-04-14). CMake FetchContent pulls `saleae/AnalyzerSDK` master.
- **Toolchain:** VS 2022 Community 17.10, CMake 3.30, Ninja, clang-format (LLVM).
- **Logic:**
  - Logic 2.4.46 is installed at `C:\Program Files\Logic`.
  - The user's instance runs the automation API on port 10430. **Don't disturb it.** Use a separate test instance if one is needed.
- **Hardware:** Logic Pro 16.
- **Sample capture:** `~/Downloads/Session 0.sal` (2026-10-06, 233 MB).
  - Recording: 6.25 MS/s digital, digital + analog on ch0–3, 3.3 V+ threshold.
  - Channel names are stale leftovers from an LED project. A matching `~/Downloads/digital.csv` export exists.
  - Mapping: **A+ = ch0, A− = ch1, B+ = ch2, B− = ch3** (inferred: ch0 + ch1 duty ≈ 1, ch2 + ch3 duty ≈ 1).

## Decisions already made (don't re-ask)

- **Live in Logic.** It is an LLA via the C++ AnalyzerSDK. HLAs can't read raw channels, and the user rejected offline `.sal` processing.
- **Signals:** the analyzer must work on **either** H-bridge logic inputs or coil terminals (through a divider). The user's answer was "Either".
- **Chopping:** "Maybe". The decoder must therefore be immune to PWM and chopping. The duty-based algorithm below is.
- **STEP/DIR** will be a **separate decoder**, if done at all. It is not part of this analyzer.

## Algorithm (validated in Python against the sample capture)

1. **Duty per window:** for each window, compute the time-weighted duty of each channel.
   - The window should be an exact multiple of the PWM period. That makes ripple vanish for constant duty.
   - The PWM period is auto-detected from rising-edge intervals, or set manually.
2. **Coil drive:** A = duty(A+) − duty(A−) and B = duty(B+) − duty(B−). Each is in [−1, 1]. Drive level = hypot(A, B).
3. **Angle:** electrical angle θ = atan2(B, A). One full step = 90° electrical.
4. **Position:** position = unwrap(θ) / 90°, in full steps.
   - Unwrap only while energized (drive > threshold).
   - |Δθ| > 135° between consecutive windows is direction-ambiguous. Flag it as an error.
5. **Coverage:** the same math covers wave, full, half and microstep drive, with or without PWM. With logic-level drive, duty is simply 0 or 1.

### Sample capture result

- Window used: 4 × 58.56 µs PWM periods.
- Holds at −1.25 full steps (θ = −112.3°) at drive ≈ 0.38, until about 6.8 s.
- Then a **full-step move of −142.5 steps** at about 180 steps/s (drive ≈ 1.36), ending around 7.6 s.
- Then holds at −143.75 (θ = 22.3°) at drive ≈ 0.38 until about 37.5 s.
- The largest |Δθ| between windows was 60.7°, so there were no ambiguous jumps.
- Hold noise: σ ≈ 0.002 full steps.
- PWM: center-aligned at about 17.08 kHz (period 58.56 µs = 366 samples at 6.25 MHz). All four terminals switch.

## Design

### Frames

Frames are **intervals of constant quantized position**: a frame spans the time the position held a value. Zoomed out, a long hold reads as one bubble ("−143.75 st"), and moves become dense runs of short frames.

| Type | Meaning |
| --- | --- |
| `position` | Energized interval at a constant quantized position |
| `off` | De-energized (drive < threshold) |
| `ambiguous` | Ambiguous jump (shown as an error) |
| `move` | *Moves and holds* mode only: a run of position spans shorter than the hold time, from the held position before to the one after |

FrameV2 fields as implemented (see the README for the full table):

| Field | Content |
| --- | --- |
| `position` | Position as a string, in the display unit |
| `steps` | Full steps (double) |
| `electrical_angle` | Degrees (not on moves) |
| `drive` | Percent |
| `direction` | Direction of the change that ended the interval |
| `rate` | delta / duration in steps/s (for a move, its mean speed) |
| `from`, `delta` | Moves only |

### Settings

- Channels: A+, A−, B+, B−.
- PWM filtering: Auto-detect / Fixed frequency (Hz) / None.
- Periods per window (default 2).
- Results: Every position change / Moves and holds; hold time in ms (default 20).
- Resolution: full step down to 1/256 (default 1/16).
- Energized threshold in % (default 10).
- Units (full steps / degrees / revolutions) and steps per rev (default 200).
- Invert direction.
- Start at zero.
- Step markers on/off.

### Code structure

- `StepperDecoder`: pure C++ with no SDK dependency, so it can be unit-tested.
- `StepperWaveform`: a synthetic PWM microstep generator, shared by the Logic simulation generator and the tests.
- SDK glue: per-window high-time integration over the 4 channels using `GetSampleOfNextEdge` and `AdvanceToAbsPosition`.

### Testing

- `tests/` executable (CTest) covering:
  - synthetic waveforms with known positions;
  - a CSV replay of `digital.csv`, compared against the Python reference.
- End-to-end: run the built DLL in a **separate** Logic instance through the automation API, using `add_analyzer` and a data-table export.

## Findings / gotchas

- **The tool host sets `ELECTRON_RUN_AS_NODE=1`.** With it, `Logic.exe` runs as plain Node and exits with code 9 ("invalid argument"). Unset it before launching Logic.
- **Logic 2 honors `--user-data-dir`** and has no single-instance lock, so a second, isolated instance works. Combine it with:
  - `SALEAE_DISABLE_DEVICE_SCAN=1`, so the test instance never claims the Pro 16;
  - `--automation --automationPort 10431`.
- **Logic CLI flags** (from `dist/main.js`): `--loadFile`, `--automation`, `--automationHost`, `--automationPort`, `--mcp`, `--mcpPort`.
- **Captures loaded through the automation API are headless.** They don't appear in the UI. To see bubbles, `save_capture` with the analyzer added, then open the file with `--loadFile`.
- **Driver behavior in the sample capture:** at full-step transitions the driver chops coil A at about 26 kHz for ~0.4 ms while its current reverses. Averaging shows it as a smooth sweep through the intermediate angle.
- **Power-up blip:** a ~5 µs staggered switch at power-up (0000→0010→1010→1110→1111 into brake) was taken as the first energized position. That is why the glitch filter exists.

- Quick duty estimates that only count edge rows inside a window are wrong when a state outlives the window: long full-step holds read as "off". Always integrate high time across window edges.
- **Dense frames mislabel at mid-zoom.** When frames are too narrow to label, Logic packs their bubble labels side by side rather than at each frame's time. A 0.8 s, 142-step move at 0.9 s zoom showed "−2.75" where the motor was near −100. That is why *Moves and holds* exists.
- **Simulation glue:** each `SimulationChannelDescriptor` must be advanced to the requested sample even with no transition. This was fixed in `323e25a`, found by `tests/simulation_tests.cpp`.
- **The demo device ignores analyzers added after capture.** Automation and MCP captures on simulation devices (`F4241` = Pro 16 demo) play generic random data, because analyzers can only be added after the capture starts. So the simulation is tested against the SDK library directly.
- **The Linux SDK `libAnalyzer.so` is built against libc++.** Linking an executable to it with GCC/libstdc++ fails with undefined `std::__1::…` symbols. The simulation test is therefore Windows/macOS only. The plugin `.so` and core tests build fine with GCC (checked in WSL with CMake 3.30.5 and Ninja downloaded to `/tmp`).
- **The Logic MCP server** (`--mcp --mcpPort N`, Streamable HTTP) exposes the same operations as the automation API. Nothing extra for analyzers.
- **Performance:** the analyzer takes about 8 s for the 37 s, 4-channel, 17 kHz-PWM capture (about 4.5× real time). Loading the 233 MB `.sal` in Logic takes about 49 s.
- `Session 0.sal` uses meta.json **version 22**. Device settings live under `legacyDevice` / `legacySettings`, and `binData` entries carry a `dataId`.

## Progress log

- [x] Prior-art check: unique. sigrok `stepper_motor` is STEP/DIR only.
- [x] User answers: either signal type; maybe chopping; STEP/DIR separate; sample capture provided.
- [x] Analyzed the sample capture and validated the algorithm in Python.
- [x] Scaffolded from SampleAnalyzer. MSVC + Ninja build via `build.ps1`.
- [x] `StepperDecoder`, `PwmEstimator`, `EdgeQueue`, `StepperPipeline`, `StepperWaveform`, plus doctest unit tests: 9 cases, 5217 assertions. Synthetic program checked at 1 / 4 / 6.25 / 50 MHz.
- [x] Analyzer glue, settings, results, simulation generator (`ec98c7c`).
- [x] Replayed `digital.csv` through the C++ decoder. It matches the Python reference: 0 → −142.25 → −142.5 with start-at-zero.
- [x] Glitch filter: energized spans < 20 µs after an off span count as off. This removed a false "ambiguous" at power-up.
- [x] Zero reference taken after 1 ms of settled drive (`3cde3aa`).
- [x] Loaded in an isolated Logic 2.4.46 instance via automation (`tools/logic-test-instance.ps1`, `66248ec`). Decodes `Session 0.sal` into 740 frames; the data table and legacy export both work.
- [x] Visual check in the Logic UI. Bubbles render, including the full "Position −142.5 steps (22.3° electrical, drive 38%)" text, UTF-8 degree sign and arrow.
- [x] *Moves and holds* result mode + `MoveGrouper` (`a57869a`). The UI shows "Move 0 → −142.25 steps (−142.25 steps in 0.8 s, −177.9 steps/s)".
- [x] README, and CI runs the tests (`83ef0e2`).
- [x] Simulation generator fixed and tested against the SDK library (`323e25a`).
- [x] Final DLL re-verified in an isolated Logic on `Session 0.sal` (both modes); test instance stopped.
- [x] Published to https://github.com/cinderblock/saleae-stepper-analyzer (public, user approved 2026-10-07). CI: Linux x86_64/arm64, macOS and Windows arm64 green on the first push.
- [x] Installed into everyday Logic (user approved 2026-10-07; user closed Logic themselves after the "Save Protected Capture" prompt).
  - `tools/install.ps1 -Register` (`6e003a1`): the DLL is in `%LOCALAPPDATA%\Saleae Logic Analyzers\StepperMotorCoils`, registered alongside the existing async-rgb-led path, with every other config value unchanged (formatting re-indented).
  - Backup at `%APPDATA%\Logic\config.json.before-stepper-install`.
  - Relaunched through `explorer.exe`. Verified the analyzer runs (demo-device capture via automation port 10430) and the Pro 16 reconnected.
- [x] CI: the first run hung on Windows x86_64 (simulation test couldn't find `Analyzer.dll` under the VS generator's `bin/Release`, and Windows' "DLL not found" box blocked). Cancelled; fixed in `a931d3d` (copy the DLL post-build, 120 s test timeouts).
- [x] End-of-capture handling (found while building the quadrature analyzer; see `../quadrature-analyzer/plans/quadrature-analyzer.md` Findings): `IdleFlusher` + pipeline `Checkpoint` + `MoveGrouper::Tick` + continuation flag (`23efce3`, pushed, CI green). The stepper capture's final off span now shows (it runs to about 280 s because Logic serves data past that capture's visible 37.9 s end).
- [x] Everyday Logic install updated to `23efce3` (2026-10-07, while Logic was closed).
- [ ] Optional manual check: demo device in the Logic UI. Add the analyzer before capturing so Logic plays its simulation.

## Open questions for the user

1. ~~Publish?~~ Yes, done.
2. ~~Install into everyday Logic?~~ Yes, done.

## Things not to do

- Don't restart or reconfigure the user's running Logic instance. On Windows the DLL is locked while loaded, so use a separate instance with its own user-data directory.
- Don't fold STEP/DIR into this analyzer.
