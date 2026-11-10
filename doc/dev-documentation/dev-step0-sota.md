# Turboprop in-air trim - Step 0, SOTA
## Context

JSBSim already has a production trim system (`FGTrim` / `FGTrimAxis` in `src/initialization/`), wired to `FGFDMExec::DoTrim()`, the tied property `simulation/do_simple_trim`, the Python bindings (`do_trim`), and IC-file `<trim>` elements. Its in-air modes (`tLongitudinal`, `tFull`, `tPullup`, `tTurn`) trim the `Udot` acceleration state against the `Throttle` control, generically, for any engine type, via `FGTrimAxis::setThrottlesPct()` + `FGPropulsion::GetSteadyState()`.

The goal of this work was to make this existing machinery actually work for a turboprop aircraft (`FGTurboProp`, in `src/models/propulsion/`). Before this change, **no turboprop aircraft in the repository had ever used `<trim>`**, and no test exercised `FGTrim` against a turboprop — the scenario had simply never been exercised.

## Investigation

### Existing architecture (read-only, no changes)

- `FGTrim::DoTrim()` (`src/initialization/FGTrim.cpp`) loops over a set of `FGTrimAxis` objects, each owning one `(State, Control)` pair (e.g. `tUdot`/`tThrottle`). For each axis it does a 1-D secant/bisection root-find (`solve()`, with bracketing via `checkLimits()`/`findInterval()`).
- `FGTrimAxis::setThrottlesPct()` (`FGTrimAxis.cpp`) is the only point where the trim axis touches the engine: it maps a normalized control value into each engine's throttle range, sets it via `FGFCS::SetThrottleCmd()` and `FGPropulsion::in.ThrottlePos[]`, then calls `fdmex->Run()` followed by `fdmex->GetPropulsion()->GetSteadyState()`.
- `FGPropulsion::GetSteadyState()` (`src/models/FGPropulsion.cpp`) is generic over any `FGEngine` subtype: while trim suspends integration, it forces `TotalDeltaT = 0.5` and repeatedly calls `engine->Calculate()` (up to 6000 times) until `engine->GetThrust()` stabilizes for 120 consecutive samples.
- `FGTrim::DoTrim()` suspends integration for the whole trim process (`FGFDMExec::SuspendIntegration()`), which makes `FGFDMExec::Run()` pass `Propulsion->in.TotalDeltaT == 0` to every engine on every frame *except* during the temporary `GetSteadyState()` excursions described above.

### Root cause of "turboprop trim doesn't work"

- `FGTurboProp::Calculate()` already has the line  `if (in.TotalDeltaT == 0) phase = tpTrim;` — so the engine model enters a dedicated `tpTrim` phase automatically whenever trim suspends integration.  This part was already "wired".
- But `Calculate()`'s `switch (phase)` only ever handled  `{tpOff, tpRun, tpSpinUp, tpStart}`. `tpTrim` fell through to  `default: HP = 0`. **Every state read the secant root-finder takes is  therefore computed with zero turboprop power**, regardless of throttle, so `Udot` never changes sign across the throttle range and   `checkLimits()`/`findInterval()` can never bracket a root — the throttle axis is reported as "not trimmable" and the whole trim fails immediately.
- `FGTurbine` (the jet engine model) already solves exactly this problem:
  it has a dedicated `Trim()` method (`FGTurbine.cpp`, invoked via   `case tpTrim: thrust = Trim(); break;`) that computes thrust   *instantaneously* from `ThrottlePos` via lookup tables, bypassing the   time-marching N2/ITT spool lag entirely. `FGTurboProp` had no equivalent.

This reframed the task from "design a new trim mechanism" to a much more surgical fix: give `FGTurboProp` the `tpTrim` handler it was always missing, mirroring `FGTurbine::Trim()`.