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
| `error` | Ambiguous jump |

`position` frames carry these FrameV2 fields:

| Field | Content |
| --- | --- |
| `position` | Full steps (double) |
| `position_units` | Position in the selected unit |
| `angle` | Electrical degrees |
| `drive` | Percent |
| `direction` | Direction of the change that ended the interval |
| `rate` | Steps/s, computed as resolution / duration |

### Settings

- Channels: A+, A−, B+, B−.
- PWM period in ns (0 = auto).
- Periods per window (default 2).
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

- Quick duty estimates that only count edge rows inside a window are wrong when a state outlives the window: long full-step holds read as "off". Always integrate high time across window edges.
- `Session 0.sal` uses meta.json **version 22**. Device settings live under `legacyDevice` / `legacySettings`, and `binData` entries carry a `dataId`.

## Progress log

- [x] Prior-art check: unique. sigrok `stepper_motor` is STEP/DIR only.
- [x] User answers: either signal type; maybe chopping; STEP/DIR separate; sample capture provided.
- [x] Analyzed the sample capture and validated the algorithm in Python.
- [ ] **Current step:** scaffold the repo from SampleAnalyzer and build an empty analyzer with MSVC.
- [ ] `StepperDecoder` + `StepperWaveform` + unit tests.
- [ ] Analyzer glue, settings, results, simulation generator.
- [ ] Replay `digital.csv` through the C++ decoder and compare against the Python reference.
- [ ] Load in a separate Logic 2 instance via automation and verify on `Session 0.sal`.
- [ ] README, CI workflow, first commit(s).

## Open questions for the user

None currently.

## Things not to do

- Don't restart or reconfigure the user's running Logic instance. On Windows the DLL is locked while loaded, so use a separate instance with its own user-data directory.
- Don't fold STEP/DIR into this analyzer.
