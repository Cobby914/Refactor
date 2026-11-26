# ScaleSense co-simulation refactor progress

## 2026-11-26 · Step 1

- Created local branch `refactor/cosimulation-v2` from clean legacy revision `98a4744`.
- Completed the [repository audit](AUDIT.md), [architecture review](ARCHITECTURE.md), and [incremental roadmap](ROADMAP.md). Selected SUMO-to-CARLA traffic mirroring for the roadside scenarios.
- Added the [v2 simulation engine](../../fusion/simulation_engine.py). It pairs one SUMO advance and traffic update with one consecutive CARLA frame. Timestamp or frame discontinuities, exceptions, and interrupts invalidate the affected session. The current CLI and GUI continue to run the legacy implementation during v2 integration.

### Checks executed

| Command or check | Result |
| --- | --- |
| `python -B -m unittest discover -s tests -v` | 8 passed. Covers ordered steps, clock mismatches, CARLA frame gaps and duplicates, concurrent entry, partial steps and interruption. |
| `python -B -m unittest discover -s tests/integration -v` | 1 passed. Two real TraCI/SUMO sessions each advanced 200 steps at 20 Hz. Their seed-42 SUMO timestamps and observed vehicle ID sequences matched. The CARLA operation returned simulated frame IDs. |
| `python -B -m unittest discover -s traffic/tests -v` | All 21 existing traffic tests passed. The legacy occlusion test uses sparse traffic, 180 s and a 0.1 s step. |
| `python -B run_fusion.py --help` and `python -B -c "import fusion_gui"` | The legacy CLI parsed its options and the GUI module imported. |

### Legacy traffic baseline

Ran `python -B traffic/validate_scenarios.py --out <temporary-report-prefix>` on Windows 10 build 19045, Intel Core i7-9700K, Python 3.12.10 and Eclipse SUMO 1.28.0. The validator used 240 simulated seconds per scenario, 0.05 s step, density 60, both directions, 30 ambient vehicles, 20 pedestrians, 10 bicycles, seed 42, and corridor-priority signals. Total invocation time was **127.52 wall-clock seconds**. These timings isolate the SUMO validation path, including its TraCI traffic queries.

| Scenario | Wall seconds | Validation |
| --- | ---: | --- |
| S01 Free Flow | 8.5 | Passed |
| S02 Moderate Demand | 10.0 | Passed |
| S03 Heavy Demand | 12.3 | Passed |
| S04 Stop and Go | 10.1 | Passed |
| S05 Mixed Vehicles | 9.2 | Passed |
| S06 Rush Hour | 6.6 | Passed |
| S07 Aggressive Lane Changing | 16.7 | Passed |
| S08 Bottleneck | 8.7 | Passed |
| S09 Overtaking | 17.4 | Passed |
| S10 Multi-Lane Overtake | 14.8 | Passed |
| S11 Occlusion | 12.5 | Failed |

The validator exited 1. S11 locked 1 of 4 named pairs and SUMO printed two teleport warnings. Running S11 again with the same inputs reproduced the outcome. The scenario-specific test passed under its separate conditions.

### Next increment

Connect a short traffic-only v2 run to real CARLA and SUMO, verify the negotiated step and command ordering, and exercise interruption and restart on the same endpoint. The CARLA RPC probe timed out on the default endpoint, so the live fused performance baseline, CARLA actor behavior, GPU sensor delivery and full campaign recovery remain pending.
