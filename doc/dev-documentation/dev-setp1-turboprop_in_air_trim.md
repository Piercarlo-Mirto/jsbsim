# Turboprop in-air trim - Step 1, FGTrim

**Status:** Implemented (throttle-only, in-air trim modes) and verified against a real Cython-built Python module via `ctest` (not just the CLI — see [Failed attempt 4](#failed-attempt-4-the-fix-didnt-actually-work-ielu-torque-limiter-froze-throttle-at-0)). One pre-existing, unrelated defect was discovered during verification and intentionally left unfixed — see [Known limitation](#known-limitation-fgpropeller-governor-instability).


## Decisions made with the user

Objectives:

1. **Extend the existing production `FGTrim`/`FGTrimAxis` path instead of building a new standalone trim class**, as the only real gap was in the engine model and not the trim algorithm.

2. **Trim control stays throttle-only** (power lever via  `ThrottleCmd`/`ThrottlePos`), matching every other engine type.

3. **Ship the `FGTurboProp::Trim()` fix as scoped; document the newly-discovered `FGPropeller` governor instability as a separate follow-up** rather than expanding scope to fix it (made during verification — see [Failed attempt 3](#failed-attempt-3-full-dotrim-on-pc7--a-second-pre-existing-crash)).

4. **Once a real build surfaced a second, more fundamental bug in the fix itself (the IELU torque limiter freezing throttle at 0 — see [Failed attempt 4](#failed-attempt-4-the-fix-didnt-actually-work-ielu-torque-limiter-froze-throttle-at-0)), fix it now** rather than reverting or shipping it broken.

## Implementation

### `src/models/propulsion/FGTurboProp.h`

Added a private method declaration, next to the existing `Off()/Run()/SpinUp()/Start()`:

```cpp
double Trim(void);
```

### `src/models/propulsion/FGTurboProp.cpp`

Added the `tpTrim` case to `Calculate()`'s phase switch:

```cpp
switch (phase) {
  case tpOff:    HP = Off(); break;
  case tpRun:    HP = Run(); break;
  case tpSpinUp: HP = SpinUp(); break;
  case tpStart:  HP = Start(); break;
  case tpTrim:   HP = Trim(); break;
  default: HP = 0;
}
```

and implemented `Trim()` itself:

```cpp
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Computes an instantaneous power response to throttle, bypassing the N1/ITT
// spool-up lag, so that trim's root-finder sees a monotonic throttle->power
// relationship instead of a time-marching one. Does not mutate persistent
// engine state (N1, ITT, oil temperature), so repeated evaluations during the
// trim search stay side-effect-free.
//
// A constant-speed propeller is governed to (approximately) its rated RPM
// across the normal power range, so the governed RPM is used here rather
// than the live RPM member: the live value is driven by FGPropeller's
// torque/inertia integration, which starts at zero and has no time to settle
// while integration is suspended for trim, and would otherwise make the
// throttle->power relationship depend on unconverged transient state.

double FGTurboProp::Trim(void)
{
  double N1_trim = IdleN1 + ThrottlePos * N1_factor;
  double trimRPM = (thrusterType == FGThruster::ttPropeller) ?
    ((FGPropeller*)Thruster)->GetMaxEngineRPM() : RPM;

  double EngPower_HP = EnginePowerRPM_N1->GetValue(trimRPM, N1_trim);
  EngPower_HP *= EnginePowerVC->GetValue();
  if (EngPower_HP > MaxPower) EngPower_HP = MaxPower;

  return EngPower_HP;
}
```

The latter is modeled on `Run()`, but without mutating any persistent engine state (`N1`, `Eng_ITT_degC`, `OilTemp_degK`), so repeated evaluations during the root-find stay side-effect-free and reproducible.

The Ielu torque-limiter block in `Calculate()` (which runs *before* the phase switch) was **initially** left untouched, on the reasoning that it naturally applies during `tpTrim` too and a torque-limited turboprop really cannot deliver unlimited power even while being trimmed. This turned out to be wrong in practice — see [Failed attempt 4](#failed-attempt-4-the-fix-didnt-actually-work-ielu-torque-limiter-froze-throttle-at-0) — and the block now explicitly skips itself during `tpTrim`:

```cpp
  // limiter intervention wanted? Skipped during trim: the torque read back
  // from the thruster reflects its last real (non-trim) time-marching state,
  // most notably RPM==0 just after loading, which combined with Trim()'s
  // idealized instantaneous power makes FGPropeller report a torque far
  // beyond any real operating envelope (power / near-zero omega). Reacting
  // to that would clamp ThrottlePos to OldThrottle on every trim evaluation
  // -- including the first one, before OldThrottle has ever advanced -- so
  // throttle would never be able to move away from 0 across the whole trim
  // search. Trim's own power model (see Trim()) is already deliberately
  // free of this kind of time-marching feedback, so the limiter is left
  // untouched (OldThrottle included) for real-time operation to pick back
  // up from where it left off once trim concludes.
  if (Ielu_max_torque > 0.0 && phase != tpTrim) {
```

### `src/models/propulsion/FGPropeller.h`

Added a small getter, used by `FGTurboProp::Trim()` above:

```cpp
/// Retrieves the propeller's configured maximum (governed) RPM, expressed
/// on the engine shaft (i.e. scaled by the gear ratio, like GetEngineRPM())
double GetMaxEngineRPM(void) const { return MaxRPM * GearRatio; }
```

 (see [Failed attempt 2](#failed-attempt-2-using-the-live-rpm-non-deterministic-result) for why it was needed)

### `src/initialization/FGTrim.h`

Added a doc-comment note (no code change) explaining the mechanism for future maintainers:

```text
During trim, integration is suspended (TotalDeltaT == 0), which engine models
use to enter a dedicated trim phase that computes an instantaneous power/thrust
response to throttle instead of their normal time-marching spool dynamics (see
e.g. FGTurbine::Trim() and FGTurboProp::Trim()). This is what allows the
tThrottle control to be used on any engine type, turboprops included.
```

### `tests/TestTurbopropTrim.py`

The testing python script. It went through several iterations as the verification work uncovered problems with the originally planned approach (See [Final test design](#final-test-design)).

```python
# TestTurbopropTrim.py
#
# Regression tests for turboprop engine power during JSBSim's trim phase.
#
# Copyright (c) 2026 Piercarlo Mirto
#
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation; either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, see <http://www.gnu.org/licenses/>
#

from JSBSim_utils import JSBSimTestCase, RunTest


class TestTurbopropTrim(JSBSimTestCase):
    # Trim suspends integration, which makes FGFDMExec pass TotalDeltaT == 0
    # to every engine. FGTurboProp::Calculate() has always entered a dedicated
    # tpTrim phase in that case, but until now there was no handler for it in
    # its phase switch, so it fell through to "default: HP = 0" and delivered
    # zero power regardless of throttle. That made the throttle trim axis
    # untrimmable on any turboprop (udot never changes sign with throttle).
    #
    # These tests check that turboprop power now responds to throttle whenever
    # integration is suspended, using fdm.run_ic() (which suspends integration
    # for a single frame, exactly like FGTrim does for each axis evaluation)
    # rather than a full simulation/do_simple_trim run. A full trim repeatedly
    # drives the throttle axis through FGPropulsion::GetSteadyState(), which
    # exposes a separate, pre-existing instability in FGPropeller's
    # constant-speed governor under large time steps once real power actually
    # flows through it (previously unreachable, since turboprop trim always
    # failed at zero thrust beforehand) -- that is a distinct defect, tracked
    # separately, and is intentionally not exercised here.
    def test_power_responds_to_throttle_when_integration_suspended(self):
        fdm = self.create_fdm()
        fdm.load_model('pc7')
        fdm['ic/vc-kts'] = 120.0
        fdm['ic/h-sl-ft'] = 3000.0
        fdm['ic/gamma-deg'] = 0.0

        powers = []
        for throttle in (0.0, 0.5, 1.0):
            fdm['fcs/throttle-cmd-norm'] = throttle
            fdm.run_ic()
            powers.append(fdm['propulsion/engine/power-hp'])

        # Idle should deliver negligible power, and power must increase
        # monotonically with throttle rather than being pinned at zero.
        self.assertAlmostEqual(powers[0], 0.0, delta=1.0)
        self.assertLess(powers[0], powers[1])
        self.assertLess(powers[1], powers[2])


RunTest(TestTurbopropTrim)

```

## Verification: failed attempts and what they revealed

Verification took much longer than the code change itself, because it surfaced three layers of problems beyond the one the plan anticipated — the third of which turned out to be in the fix itself, not just in the surrounding code. They are recorded here in full, in the order they were found, because each one changed the design or the test strategy.

### Failed attempt 1: the `L410` test aircraft cannot fly

The plan's first idea was to reuse `L410` (already used by `tests/TestTurboProp.py`, via `scripts/L4102.xml` /`engine/engtm601.xml`) for an in-air trim regression test. A manual CLI check (`JSBSim.exe --aircraft=L410 --initfile=<cruise ic>`) consistently failed to trim, with `udot` barely changing across the whole throttle range.

Investigation showed the `L410.xml` aircraft file is explicitly marked

> *"This aircraft model is a BETA release!!! This aircraft model probably
> will not fly as expected. Use this model for development purposes
> ONLY!!!"*

and its propeller is configured with a **99 ft diameter** — an obvious data error, unrelated to trim. `L410` is only ever exercised on the ground (brakes set, gear down) by the existing test suite; it cannot validate in-air trim. This ruled out `L410` as a verification vehicle and motivated switching to `pc7` (an Aeromatic-generated PT6A-powered trainer with realistic metrics) for the rest of the investigation.

### Failed attempt 2: using the live RPM — non-deterministic result

The first version of `Trim()` read the engine's own live `RPM` member (`EnginePowerRPM_N1->GetValue(RPM, N1_trim)`), exactly mirroring how `Run()` uses it. However, a temporary debug instrument in  `FGTrim::checkLimits()` showed the throttle axis's bracketing check returning **different results for the exact same control value** across successive calls within the same trim run (e.g. `ahi` for `Throttle=1` was seen as `-5.03`, `-0.06`, `-5.54`, `-8.13`, and once `+1.27`, across consecutive evaluations).

Root cause: `FGTrimAxis::Run()`'s stability loop and `FGTrim::checkLimits()` call `fdmex->Run()` with `TotalDeltaT == 0` most of the time (hitting the new `Trim()` case), but `FGTrimAxis::setThrottlesPct()` also calls `FGPropulsion::GetSteadyState()`, which temporarily forces `TotalDeltaT = 0.5`. Since `FGTurboProp::Calculate()` has

```c++
if ((phase == tpTrim) && (in.TotalDeltaT > 0)) { ... phase = tpRun ... }
```

every `GetSteadyState()` call flips the engine into its **real**, time-marching `tpRun` phase for up to 6000 iterations of 0.5 s — spinning the *live* propeller RPM up or down via `FGPropeller`'s own torque/inertia integration. The live RPM at the start of each such excursion depends on whatever state was left over from the *previous* excursion, so `Trim()`'s result (which read that same live RPM) was not a pure function of throttle — it depended on unconverged history.

#### Solution

Analogously to how `FGTurbine::Trim()` computes `N2` fresh from `ThrottlePos` alone, with no dependency on the real spool-lag state, `Trim()` now assumes the governed RPM directly rather than whatever the live member happens to hold. This required adding `FGPropeller::GetMaxEngineRPM()` (see above) and using it in `Trim()` when the thruster is a propeller.

After this change, repeated runs produced identical, deterministic `checkLimits()` results for the same throttle value.

### Failed attempt 3: full `DoTrim()` on `pc7` — a second, pre-existing crash

With the deterministic `Trim()` in place, a manual CLI trim on `pc7` (PT6A, 120 kts / 3000 ft) was attempted end-to-end via `simulation/do_simple_trim`. It did not converge normally; instead, after several million calls into `FGPropeller::Calculate()`, the process crashed with a Windows `STATUS_ARRAY_BOUNDS_EXCEEDED` (`0xC0000090`) exception. This reproduced identically in both a Debug build (with `/RTC1` runtime checks) and a from-scratch Release build, ruling out a compiler-instrumentation false positive.

Instrumentation (temporarily added and later removed) in `FGPropeller::Calculate()` showed `RPM` oscillating chaotically but *boundedly* (observed roughly between 0 and ~2150, i.e. within the propeller's rated range) over millions of calls — so this **was not** a numeric-overflow/runaway RPM problem. The oscillation is consistent with the constant-speed governor logic in `FGPropeller::GetPowerRequired()` (the `Pitch -= dRPM * in.TotalDeltaT` bang-bang update) and the explicit-Euler RPM update in `FGPropeller::Calculate()` (`RPM = (RPS + (ExcessTorque / Ixx) / (2π) * dt) * 60`) hunting/limit-cycling under the large, coarse 0.5 s steps that `GetSteadyState()` uses — a step size tuned for jets' smooth exponential (`ExpSeek`) spool dynamics, not for a torque/inertia feedback loop with an aggressive governor. After enough iterations of this hunting, some combination of inputs (likely an extreme `J`/`Pitch`/Mach value fed into one of `FGPropeller`'s internal tables) triggers the actual array-bounds violation.

This is a **separate, pre-existing defect** in `FGPropeller`, independent of the `FGTurboProp::Trim()` fix. It was simply never reachable before, because turboprop trim always failed at the very first throttle evaluation (zero thrust) — it never got far enough to drive `GetSteadyState()` with meaningful power. Checking every turboprop aircraft in the repository (`pc7`, `DHC6`, `L410`, `pogo-jsbsim`) showed all of them configure a constant-speed propeller (`<maxrpm>` present, `MinPitch != MaxPitch`), so this risk is not specific to one aircraft — any of them can hit it once a full multi-iteration `DoTrim()` is run.

For this step, the chosen resolution was to ship the `FGTurboProp::Trim()` fix as scoped, and to design the regression test so that it **does not exercise** `GetSteadyState()`'s long iteration loop at all (see below).

### Final test design

Given the above, a full `simulation/do_simple_trim` call cannot safely be used in the committed test suite on any currently-available turboprop aircraft: it risks a multi-million-iteration run ending in a crash.

Instead, `tests/TestTurbopropTrim.py` exercises the **specific mechanism the fix repairs** directly and safely: `fdm.run_ic()` suspends integration for exactly one frame (the same mechanism `FGTrim` relies on for each axis evaluation) without ever calling `GetSteadyState()`. The test loads `pc7`, sets a cruise IC, and for throttle settings `0.0, 0.5, 1.0` calls `fdm.run_ic()` and reads `propulsion/engine/power-hp` immediately afterward, asserting:

- idle power is ~0 (within 1 HP), and
- power increases strictly monotonically with throttle.

This was first checked manually via the JSBSim CLI, in an environment that didn't yet have Cython available to build the Python module (see [Failed attempt 4](#failed-attempt-4-the-fix-didnt-actually-work-ielu-torque-limiter-froze-throttle-at-0) for why that CLI check, on its own, gave a false sense of confidence). Once Cython was installed and the test was actually run through `ctest` against the compiled module, it initially **failed** — which is how the bug in Failed attempt 4 was caught — and passes after that fix:

```
1/1 Test #33: TestTurbopropTrim ................   Passed    0.57 sec
```

with the following HP-vs-throttle curve, read via the Python module itself (`fdm.run_ic()` + `propulsion/engine/power-hp`):

| Throttle | `power-hp` |
|---------:|-----------:|
| 0.0      | 0.0956     |
| 0.2      | 44.70      |
| 0.3      | 67.00      |
| 0.5      | 111.60     |
| 0.6      | 133.90     |
| 0.8      | 269.77     |
| 1.0      | 491.77     |

— a clean, monotonic, deterministic curve, confirming the fix works as intended for exactly the mechanism `FGTrim` depends on, without touching the unstable `GetSteadyState()` path. (These happen to be the same numbers the earlier CLI check produced — that check was measuring a real throttle→power relationship, just not the one-frame `tpTrim` evaluation `FGTrim` actually depends on; see below.)

The test's docstring/comments explicitly record why the full-trim path is not exercised, so a future contributor doesn't "simplify" the test back into the unsafe form.

### Failed attempt 4: the fix didn't actually work — IELU torque limiter froze throttle at 0

This was found *after* Cython became available in this environment and the Python module/test suite could finally be built and run for real (see [Environment notes](#environment-notes)). Running `tests/TestTurbopropTrim.py` through `ctest` against the actual compiled module — rather than inferring its behavior from a CLI script, as all earlier verification had done — failed immediately:

```
AssertionError: 0.09561956769524456 not less than 0.09561956769524456
```

Power at throttle 0.0 and throttle 0.5 were bit-for-bit identical: `ThrottlePos` was being silently clamped back to 0 inside `FGTurboProp::Calculate()`, regardless of the commanded value.

**Root cause:** `Calculate()`'s Ielu (over-torque) limiter block runs *before* the phase switch, and reads back `Thruster->GetTorque()` — the propeller's torque from its *previous* `Calculate()` call. `FGPropeller::Calculate()` computes `ExcessTorque = PowerAvailable / omega`, where `omega` is the propeller's real angular rate — which is still ~0 immediately after loading, since integration is suspended throughout trim and the propeller never gets a chance to spin up. Feeding `Trim()`'s realistic instantaneous power (e.g. ~112 HP at throttle 0.5) through that near-zero `omega` produces an implied torque of tens of thousands of ft·lb — far beyond `pc7`'s configured `Ielu_max_torque` (1303.7 ft·lb). The limiter reacts exactly as designed for a real overtorque event: it clamps `ThrottlePos` down to `OldThrottle`. But because this triggers on the very first trim evaluation, before `OldThrottle` has ever advanced past its initial value of 0, and because the clamped `ThrottlePos` is itself what gets written back into `OldThrottle` at the end of the block, `OldThrottle` can never advance either — so throttle is stuck at 0 for the rest of the trim run, no matter what is commanded.

This directly contradicts what [Failed attempt 2](#failed-attempt-2-using-the-live-rpm-non-deterministic-result)'s fix and the original Implementation section assumed: that letting the Ielu block "naturally apply" during `tpTrim` was harmless or even physically appropriate. It is not — the live-RPM problem and this one share the same underlying cause (the propeller's real, integrated state is meaningless while integration is suspended), and the earlier fix only addressed it for `Trim()`'s own RPM lookup, not for the limiter block that runs just before `Trim()` is called. It also explains why the CLI-based "verification" in the previous version of this document never caught it: that check used a script with real (nonzero) time steps, which drives the engine through the normal time-marching `tpRun` phase, not the single suspended-integration `tpTrim` frame that `FGTrimAxis::setThrottlesPct()`'s `fdmex->Run()` call (and `fdm.run_ic()`) actually exercise. The CLI check was measuring a real, self-consistent throttle→power relationship — just not the one `FGTrim` depends on.

A second, environment-level factor delayed catching this: this machine has a `pip`-installed `jsbsim` package (version 1.3.1, unrelated to this checkout) that silently shadows the local dev build for a *script* run via `python script.py` (Python puts the script's own directory on `sys.path` ahead of `PYTHONPATH`/site-packages, but the *local build* only ends up on `sys.path` by `os.getcwd()` being appended at the very end — see `JSBSim_utils.py` — which still loses to the pip package in site-packages). A `python -c "..."` one-liner run from inside the build's `tests/` directory does *not* have this problem (`-c` puts `""`/cwd at the very front of `sys.path`), which is why early ad-hoc checks looked consistent even while a stray, unrelated release build was actually being exercised. Running `ctest` directly surfaces the same shadowing (ctest's working directory is the build's `tests/` dir, but `JSBSim_utils.py`'s `sys.path.append(os.getcwd())` still lands after site-packages). The fix for this investigation was to set `PYTHONPATH` to the build's `tests/` directory explicitly before invoking `ctest`/`python`, forcing the local module to resolve first. This is purely a quirk of this machine's Python environment, not a defect in JSBSim's test harness, but it is worth knowing about for anyone else testing on a machine with a pip-installed `jsbsim`.

**Fix:** skip the Ielu block entirely while `phase == tpTrim`, leaving `OldThrottle` and `Ielu_intervent` untouched so real-time operation picks back up from where it left off once trim concludes (see the updated code block in [Implementation](#srcmodelspropulsionfgturbopropcpp)). After this change, `tests/TestTurbopropTrim.py` passes, and the full `ctest` suite (58/59; the one failure, `TestActuator`, is a pre-existing, unrelated missing-`scipy` dependency on this machine) shows no regressions, including `TestTurboProp`, `TestTurbine`, and `CheckTrim`.

## Debug instrumentation used during investigation

None of the snippets in this section are part of the committed diff. Each one was added to answer a specific question, used for exactly that, and deleted again before the surrounding change was finalized. They're recorded here so a future investigation into similar symptoms (non-deterministic trim results, a propeller-model crash, a frozen control) doesn't have to reinvent them from scratch.

The snippets for Failed attempts 2 and 3 were written, used, and removed in an earlier part of this same investigation, before this document existed. The literal original lines are no longer available to quote — what follows is reconstructed from the investigation notes (variable names, call sites, and observed output) rather than copied verbatim, and is illustrative of the technique rather than an exact diff. The Failed attempt 4 snippets, by contrast, are quoted exactly as written and removed, since that happened within this same editing session.

### Failed attempt 2: tracing non-determinism in `checkLimits()`

A print was added at the point in `FGTrim::checkLimits()` where the bracketing values are checked, to log them across repeated calls at the same commanded throttle:

```cpp
// temporarily added inside FGTrim::checkLimits(), removed after use
std::cerr << "checkLimits: control=" << control_value
          << " alo=" << alo << " ahi=" << ahi << std::endl;
```

and a matching one inside `FGTurboProp::Trim()` itself, to see what RPM/N1 it was actually reading on each call:

```cpp
// temporarily added inside FGTurboProp::Trim() (the live-RPM version), removed after use
std::cerr << "Trim(): live RPM=" << RPM << " N1_trim=" << N1_trim
          << " EngPower_HP=" << EngPower_HP << std::endl;
```

Together these showed `ahi` for the same `Throttle=1` evaluating to different values across consecutive calls within one trim run (`-5.03`, `-0.06`, `-5.54`, `-8.13`, `+1.27`), while the `Trim()` print showed the live `RPM` value differing between those same calls — pointing directly at the live-RPM dependency described above, and motivating the switch to `GetMaxEngineRPM()`.

### Failed attempt 3: isolating the `FGPropeller` crash

A counter-gated print was added inside `FGPropeller::Calculate()` to avoid flooding the console over the several million calls it takes to reach the crash, logging `RPM` only periodically:

```cpp
// temporarily added inside FGPropeller::Calculate(), removed after use
static int dbgcount = 0;
if (++dbgcount % 50 == 0) {
  std::cerr << "FGPropeller::Calculate RPM=" << RPM
            << " Pitch=" << Pitch << " call=" << dbgcount << std::endl;
}
```

This showed `RPM` hunting chaotically but staying bounded (roughly 0-2150) right up to the crash, which is what ruled out a simple runaway/overflow and pointed instead at the governor's bang-bang `Pitch` update limit-cycling under `GetSteadyState()`'s large steps until some input combination hit an internal table out of bounds.

### Failed attempt 4: locating the IELU freeze

This round's instrumentation is reproduced exactly, since it was added and removed within this conversation.

The first question was simply whether `FGTurboProp::Calculate()` was being reached at all, and with what `ThrottlePos`/`phase`, since the test showed power pinned regardless of commanded throttle. Added at the top of `Calculate()`, right after `ThrottlePos` is assigned:

```cpp
ThrottlePos = in.ThrottlePos[EngineNumber];
std::cerr << "DBGCALC ThrottlePos=" << ThrottlePos << " TotalDeltaT=" << in.TotalDeltaT
          << " phase_before=" << (int)phase << " Running=" << (int)Running
          << " Cutoff=" << (int)Cutoff << " Starved=" << (int)Starved << std::endl;
```

and, right after the block of phase overrides that decides the final `phase` for this frame:

```cpp
if (Condition >= 10) {
  phase = tpOff;
  StartTime=-1;
}
fprintf(stderr, "DBGCALC2 phase_after=%d\n", (int)phase);
fflush(stderr);
```

This confirmed `ThrottlePos` arriving correctly (0, 0.5, 1 in turn) and `phase_after` consistently `4` (`tpTrim`) — so the engine model was being reached correctly and entering trim mode as expected. That ruled out the phase logic and pointed at `Trim()` itself, so a second print was added there:

```cpp
double EngPower_HP = EnginePowerRPM_N1->GetValue(trimRPM, N1_trim);
EngPower_HP *= EnginePowerVC->GetValue();
if (EngPower_HP > MaxPower) EngPower_HP = MaxPower;

fprintf(stderr, "DBGTRIM ThrottlePos=%f N1_trim=%f trimRPM=%f tableval=%f VC=%f EngPower_HP=%f\n",
        ThrottlePos, N1_trim, trimRPM, EnginePowerRPM_N1->GetValue(trimRPM, N1_trim), EnginePowerVC->GetValue(), EngPower_HP);
fflush(stderr);

return EngPower_HP;
```

This was the key finding: `ThrottlePos` read inside `Trim()` was **always `0.000000`**, even on calls where `DBGCALC` (printed earlier in the very same `Calculate()` invocation) had shown `ThrottlePos` correctly set to `0.5` or `1`. Since nothing between those two points modifies `ThrottlePos` except the Ielu block, that is what identified it as the culprit, confirming the root cause described above.

(Two earlier, broader prints — logging entry into `FGPropulsion::Run()` and `FGFDMExec::Run()` — were added and removed even before these, to rule out the possibility that the engine's `Calculate()` wasn't being reached at all. That turned out to be a red herring caused by a stray `pip`-installed `jsbsim` package shadowing the local build in one particular invocation style; see the discussion of that shadowing issue above. For completeness, those prints were:

```cpp
// temporarily added at the top of FGFDMExec::Run(), removed after use
bool FGFDMExec::Run(void)
{
  bool success=true;
  std::cerr << "DBGFDM Run() entered, Models.size()=" << Models.size() << std::endl;

  Debug(2);
```

```cpp
// temporarily added at the top of FGPropulsion::Run(), removed after use
bool FGPropulsion::Run(bool Holding)
{
  std::cerr << "DBGPROP Run() called Holding=" << Holding << " Engines.size()=" << Engines.size() << std::endl;
  if (FGModel::Run(Holding)) { std::cerr << "DBGPROP skipped by FGModel::Run" << std::endl; return true; }
  if (Holding) return false;
```

)

## Known limitation: `FGPropeller` governor instability

**Not fixed by this change, left as a documented follow-up:**

`FGPropeller`'s constant-speed governor (RPM/Pitch update in `GetPowerRequired()` and the RPM integration in `Calculate()`) is unstable under the large (0.5 s) time steps `FGPropulsion::GetSteadyState()` uses once meaningful power is flowing through it — observed as chaotic RPM hunting that, over enough iterations, can trigger an out-of-bounds table access and crash the process. This affects any aircraft with a constant-speed propeller (which is every turboprop currently in the repository) if a full `simulation/do_simple_trim` in-air trim is attempted via the standard `FGTrim` throttle axis. It is independent of, and pre-existing to, the fix described in this document — it was simply unreachable before, because turboprop trim always failed at zero thrust on the very first evaluation.

Suggested starting points for a future fix:

- Reduce `GetSteadyState()`'s step size (or make it configurable /
  engine-type-aware) so it doesn't destabilize a governed propeller's
  feedback loop.
- Make the constant-speed governor's `Pitch` update in
  `FGPropeller::GetPowerRequired()` less sensitive to large `dt` (e.g. a
  proper discrete-time-stable controller rather than a fixed per-second
  rate term multiplied by a 0.5 s step).
- Add a safety clamp/guard in `FGPropeller::Calculate()` against table
  lookups with extreme `J`/`Pitch`/Mach inputs, independent of fixing the
  root instability.

## Files changed

| File | Change |
|---|---|
| `src/models/propulsion/FGTurboProp.h` | Added `double Trim(void);` declaration |
| `src/models/propulsion/FGTurboProp.cpp` | Added `Trim()` implementation, `case tpTrim:` in `Calculate()`, and the `phase != tpTrim` guard on the Ielu limiter block (see [Failed attempt 4](#failed-attempt-4-the-fix-didnt-actually-work-ielu-torque-limiter-froze-throttle-at-0)) |
| `src/models/propulsion/FGPropeller.h` | Added `GetMaxEngineRPM()` getter |
| `src/initialization/FGTrim.h` | Added doc-comment note (no behavior change) |
| `tests/TestTurbopropTrim.py` | New regression test (see [Final test design](#final-test-design)) |
| `tests/CMakeLists.txt` | Registered `TestTurbopropTrim` in the `PYTHON_TESTS` list so `ctest` actually runs it (it had been written but never wired in) |

No changes were made to `FGTrim.cpp`, `FGTrimAxis.*`, `FGPropulsion.cpp`, or
any aircraft/engine XML configuration file — the fix is entirely contained
within the turboprop engine model plus one small propeller getter.

## Environment notes

Verification happened in two phases, on Windows with MSVC (`vcvars64.bat`) + Ninja:

**Phase 1 (no Cython):** the Python interpreter CMake had originally detected (`anaconda3`) did not have Cython installed, so the Python module and `ctest` suite could not be built. All verification in this phase was done through the `JSBSim.exe` CLI (`--initfile`, `--property`, `--script`, CSV output). This is the phase that produced [Failed attempts 1-3](#failed-attempt-1-the-l410-test-aircraft-cannot-fly) and the first (incomplete) version of this document — it gave high confidence in a fix that, per Failed attempt 4, didn't actually work for its intended purpose.

**Phase 2 (Cython installed):** once Cython was available, building the Python module surfaced one more wrinkle, beyond the IELU bug: CMake's `FindPython3` had picked `anaconda3`'s Python, but `pip install cython` (and `conda install`) both failed there with SSL certificate errors specific to that environment; Cython and the rest of the test dependencies (`pandas`, `numpy`) were instead installed for a separate `C:\Python314` interpreter already on `PATH`. A dedicated Release build directory (`build_pytest/`, CMake configured with `-DPython3_EXECUTABLE=C:/Python314/python.exe` and an explicit `-DCYTHON_EXECUTABLE=...`) was created for this, since the existing Debug build directory can't link a Python extension at all without a debug Python install (MSVC's `_DEBUG` pulls in `Py_DEBUG`/`Py_INCREF_IncRefTotal`, which only exists in `python314_d.lib`). This is also the phase that uncovered the `pip`-installed `jsbsim` 1.3.1 package shadowing issue described in Failed attempt 4, worked around via an explicit `PYTHONPATH`.

With that build in place, `ctest -R TestTurbopropTrim` (and the full suite) now run for real against the compiled module, which is how Failed attempt 4 was found and fixed. A contributor on a normally-configured machine (Cython + matching Python already set up, no stray `pip`-installed `jsbsim`) should just be able to run `ctest -R TestTurbopropTrim` directly.

All temporary debug instrumentation (`fprintf`/`std::cerr` prints added at various points to `FGTrim.cpp`, `FGTurboProp.cpp`, `FGPropeller.cpp`, `FGPropulsion.cpp`, and `FGFDMExec.cpp` across both phases of investigation) and all scratch files created to drive manual verification (temporary aircraft IC files, scripts, build directories) were removed before each version of this change was finalized; none of them are part of the committed diff.
