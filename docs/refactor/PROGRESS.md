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

## 2026-11-26 · Step 2

- Added a [live traffic command](../../README_RUNNING.md#v2-traffic-only-co-simulation), with CARLA world settings scoped to the session and one SUMO child owned by the coordinator.
- Verified against the local offscreen `Town10HD_Opt` on port 22000. The CARLA 0.9.16 Python package and server both reported build `1ae535667`. SUMO and TraCI were 1.28.0.
- The command advanced 200 paired steps at 0.05 s per step to SUMO time 10 s. Peak SUMO traffic was four vehicles. This is a clock and lifecycle check. Actor mirroring, signal updates, capture and fused performance measurements are later milestones.

| Command or check | Result |
| --- | --- |
| `python -B -m fusion.traffic_session --carla-port 22000 --steps 200 --step-length 0.05` | Completed with consecutive CARLA frames and verified CARLA frame deltas. |
| `python -B -m unittest discover -s tests/integration -v` with `SCALESENSE_TEST_CARLA_PORT=22000` | 5 passed. Live checks cover malformed SUMO startup, failure after CARLA settings were applied, SIGINT at the first paired frame followed by two 200-step runs on the same world, and refusal to take an already-synchronous world. The suite verifies the original CARLA settings and checks that each owned SUMO child was reaped. |
| `python -B -m unittest discover -s tests -v` and `python -B -m unittest discover -s traffic/tests -v` | 8 simulation-engine tests and all 21 existing traffic tests passed. |
| CLI with an unused CARLA endpoint and a two-second readiness window | Exited 1 with a CARLA readiness error. |
| CLI with a requested 0.033333 s step | Exited 1 before CARLA configuration. SUMO reported an effective step of 0.033 s. |

### Next increment

Use the clock's pre-CARLA phase to apply acknowledged motion and signal commands. Bring over the existing scenario and map rules, track owned actors in the registry, and verify spawn, movement, culling, and teardown on the live world.

## 2026-11-26 · Step 3

- Wired the v2 traffic command to the existing scenario controllers, routes, map geometry and blueprint mappings through its own TraCI connection.
- Added a session-owned actor registry and acknowledged CARLA batches for spawn, movement and destruction. Added a deterministic walker blueprint selection keyed by the seed and SUMO ID.
- Mapped ten physical CARLA signal heads to SUMO approaches at junctions 189, 532 and 719. The paired frame is published only after CARLA reports the expected aspects.

| Command or check | Result |
| --- | --- |
| `python -B -m fusion.traffic_session --carla-port 22000 --scenario 1 --steps 800 --no-cull --ambient-vehicles 3 --pedestrians 12 --bicycles 8` | Completed 800 paired frames through 40 simulated seconds. Peak counts were 23 SUMO vehicles and 29 CARLA mirrors. One actor spawn was deferred by a collision. |
| `python -B -m unittest discover -s tests/integration -v` with `SCALESENSE_TEST_CARLA_PORT=22000` | 8 passed. Live checks include motion for cars, bikes and walkers, a corridor signal change, junction heading and position, cull/re-entry and arrival, preservation of an unrelated actor, recovery after an injected lost spawn response, and interruption and restart. |
| `python -B -m unittest discover -s tests -v` and `python -B -m unittest discover -s traffic/tests -v` | All 10 v2 unit tests and 21 legacy traffic tests passed. |
| Two independent 400-frame S08 runs with SUMO seed 42 and Python hash seeds 19 and 97 | Both scheduled the work-zone blocker. Across 20 paired-frame samples, normalized SUMO identities, CARLA blueprint IDs, x/y positions and yaw matched to two decimal places. |
| V2 CLI with S05, 240 steps and `--stretch-signals static --no-cull` | Completed 12 simulated seconds with the imported signal programs and nine peak CARLA mirrors. |

The current signal mapping covers vehicle approach colors at the three corridor junctions. CARLA freezes its lights globally during the run. Teardown verifies the original corridor aspects and freeze mode using maintenance ticks outside the paired frames. After the final run, CARLA was asynchronous with active signal timing, and no session actors or SUMO processes remained. The isolated test server was stopped. [Signal and lifecycle details](ARCHITECTURE.md#scenario-actor-and-signal-synchronization)

### Next increment

Join delayed radar and camera callbacks to exact-frame ground truth using the paired frame ID. Test missing and late sensor messages before connecting the existing dataset writers.

## 2026-11-26 · Step 4

- Added opt-in [v2 single-run capture](../../README_RUNNING.md#v2-single-run-capture) using the existing CARLA sensor rig and dataset formats. The frame manifest adds coverage for every expected sensor, including zero-return radar scans.
- Joined callbacks to confirmed CARLA frames using a bounded buffer and moved dataset export to an eight-frame writer queue. Normal teardown allows pending camera frames to arrive before it releases the rig and simulator resources.

| Command or check | Result |
| --- | --- |
| `python -B -m unittest discover -s tests -v` | All 16 tests passed. New checks cover early and out-of-order callbacks, duplicate suppression, expired and final frames, exact camera metadata, and writer error propagation. |
| `python -B -m unittest discover -s tests/integration -v` with `SCALESENSE_TEST_CARLA_PORT=22000` | All 10 tests passed. The 4-, 8-, 12- and 14-radar setups each produced 32 complete paired frames on CARLA 0.9.16. PNGs, frame IDs, actor metadata and sensor extrinsics were verified. The interruption test retained the three confirmed frames and released all owned actors. |
| Capture-specific integration rerun | The legacy labeler also processed all four layout captures with zero missing actor frames. |
| Legacy CSV schema comparison and `python -B -m unittest discover -s traffic/tests -v` | Both CSV header definitions and the fast radar return writer matched the preceding Git revision. All 21 legacy traffic tests passed. |
| 600-frame v2 S01 capture with 4 radars at 1,200 points/s, 30 ambient vehicles, 12 pedestrians, 8 bicycles, and `--postprocess` | All 600 frames had every sensor. The labeler processed 112,544 radar returns with zero missing actor frames and 779 matches. The Doppler/RCS stage completed. The PointNet frame builder produced 600 frames and 100,187 points. |

The initial sparse 48-frame verification capture had zero matched returns. The legacy postprocessor reported its existing calibration error for that capture, and the populated 600-frame run exercised postprocessing successfully. Final cleanup restored CARLA settings and signal timing, removed the owned actors and sensors, and reaped SUMO. The isolated CARLA server was stopped.

### Next increment

Route the v2 session through the existing campaign and GUI entry points. Anchor recording windows to confirmed paired frames and preserve each campaign row's metadata and output format.

## 2026-11-26 · Step 5

- Added `--engine v2` to the [single-run and campaign CLI](../../README_RUNNING.md#v2-single-run-capture) and an **Engine** selector to the [campaign GUI](../../fusion_gui.py). The selected engine is saved with the campaign. Plans without an engine field default to legacy.
- The [v2 campaign coordinator](../../fusion/campaign_v2.py) owns one rig and capture directory. Each row starts a fresh SUMO process and advances both simulators during warm-up. `segments.json` records the inclusive range of frames saved per row, and the legacy labeler uses it to populate the segment and scenario columns.

| Command or check | Result |
| --- | --- |
| `python -B run_fusion.py --engine v2 --duration 1` with four radars, 800 points/s and labeling disabled | Recorded 20 paired frames at 20 Hz through the normal single-run entry point. |
| Two consecutive two-row campaigns on the same `Town10HD_Opt` world | Scenarios 1 and 3 each had three warm-up frames and 100 recorded frames. Frame manifest, actor log, camera CSV and segment bounds agreed. Both rows used the same camera ID and distinct SUMO children. The labeler processed one campaign with zero missing actor frames and the expected segment and scenario IDs. |
| GUI Stop and a SIGINT raised during recording through `run_fusion.main()` | Both saved an interrupted capture with the confirmed prefix, skipped the second row and reaped SUMO. The CLI returned `130`. |
| Injected CARLA tick timeout on paired step nine | The first row failed with five recorded frames after three warm-up frames. The second row was skipped. A new campaign on the same world completed both rows. |
| CLI campaign with scene cleanup enabled, four radars at 1,200 points/s, and S01/S03 rows of 30 s and 5 s | Saved 600 and 100 paired frames. The labeler handled 129,200 returns with zero missing actor frames and 793 matches. Doppler and RCS processing completed with per-segment output. |
| `python -B -m unittest discover -s tests -v`, `-s tests/integration -v` and `-s traffic/tests -v` | 16 unit tests, 14 integration tests and 21 legacy traffic tests passed. The Tk integration ran outside the sandbox so Tcl could initialize. CARLA settings, signal timing and owned actor and process cleanup were verified. |

### Next increment

Record paired legacy and v2 runs with the same scenarios, seeds and sensor configuration. Compare timing, frame coverage and the perception pipeline outputs before changing the default engine.

## 2026-11-27 · Step 6

- Completed the [matched live capture benchmark](BENCHMARK.md) on CARLA 0.9.16 and SUMO 1.28.0. Ran two trials per engine at each of 4, 8, 12 and 14 radars, plus one control per layout from `98a4744`. The [benchmark harness](../../tests/benchmark_capture.py) measures the existing campaign entry points.
- Both engines saved all 3,520 camera and actor frames and all 33,440 radar scans in the matched trials. V2 confirmed 3,520 paired steps and cleaned up each session. The legacy runner updated on 3,462 recorded frames and its rig remained after each campaign returned. The pinned controls reproduced both legacy behaviors.
- A 22 sim-second recording took 7.34–9.13 wall seconds in legacy and 14.50–21.07 wall seconds in v2. V2's writer queue was full at 67.3–92.3% of recording tick samples. The report separates recording, whole campaign and shutdown timings.
- All 20 captures passed the existing labeler, Doppler/RCS processing and PointNet frame building. The labeler reported zero missing actor frames. The 16 v2 unit tests and 21 legacy traffic tests passed. The isolated CARLA server was stopped after verifying an empty rig and restored settings.

Keep v2 opt-in while profiling the recording path and validating sustained throughput at the default ray budget.

## 2026-11-27 · Step 7

- Profiled ground-truth collection, PNG encoding, radar CSV writing and writer submission. PNG encoding was the largest writer stage at 1,200 rays/s. The v2 writer now uses two camera encoders by default, with the same bound of eight queued frames plus the active frame. The ordered writer publishes each PNG and its metadata after encoding succeeds.
- In five [matched before-and-after trials](BENCHMARK.md#parallel-png-follow-up), v2 recording wall time fell by 24.0–29.7% at 1,200 rays/s. The 14-radar, 15,000 rays/s run fell from 121.90 to 107.89 wall seconds for 60 simulated seconds. A one-encoder trial at that load took 105.49 s and reduced paired-step p95 from 153.6 to 125.4 ms compared with two workers.
- All trials kept the expected paired frames, matching radar return counts and camera/actor coverage. The four tuned 1,200 rays/s captures passed labeling, Doppler/RCS processing and PointNet frame building. All 17 unit tests and 14 live integration tests passed, including ordered commits after out-of-order encoding, writer failure cleanup, the bounded frame queue, GUI Stop, and recovery from an injected clock timeout.

V2 remains opt-in. Sustained full-rate recording on the test host still exceeds real time.

## 2026-11-27 · Step 8

- Profiled the per-return writer and batched the v2 CSV output by radar scan. The [batched writer](../../fusion/radar_csv.py) reuses scan-level fields and reads packed returns. It preserves the existing schema, rounding, detection order, and CSV quoting.
- In the [matched full-rate comparison](BENCHMARK.md#batched-radar-csv-follow-up), 60 simulated seconds took 40.04 wall seconds, down from 106.08 s. Median radar CSV time fell from 68.47 to 14.57 ms per frame. Paired-step p95 fell from 152.18 to 46.78 ms. Both runs recorded all 1,200 paired frames and 9,708,118 radar returns.
- All 19 unit tests and 14 live integration tests passed. The live suite compared the two writers on identical measurements for every radar in each of the four layouts. The existing labeling, Doppler/RCS and PointNet pipeline passed on an 80-frame excerpt containing 647,802 full-rate returns. CARLA restored its settings and released the rig. The dedicated test server was stopped.

V2 remains opt-in while repeated full-rate multi-row campaigns are validated.

## 2026-11-27 · Step 9

- Ran two consecutive [full-rate campaigns](BENCHMARK.md#repeated-full-rate-campaigns) on the same CARLA world. Each used 14 radars at 15,000 rays/s and three 60-second recording windows for Free Flow, Heavy Demand, and Stop and Go. Whole-campaign wall time was 137.40 and 137.76 s, including 24 simulated seconds of warm-up in each campaign.
- Verified all 7,200 paired frames and 100,800 radar scans across the two runs. Frame IDs agreed across the camera index, actor log and segment metadata. Each row reset SUMO time and used a fresh worker. Both campaigns restored CARLA settings and released their actors. Heavy Demand's tick interval p95 was 51.48 and 51.70 ms.
- All 19 unit tests and 15 integration tests passed. The new [full-rate Stop regression](../../tests/integration/test_live_campaign_v2.py) preserved a completed row and the recorded prefix of the second row, skipped the third, and completed a retry on the same world. The offline pipeline passed on a 60-frame excerpt spanning all three scenarios. The isolated CARLA server was stopped.

Legacy remains the default.

## 2026-11-27 · Step 10

- Promoted v2 for standard 20 Hz single runs and for new GUI campaigns. Saved plans retain their explicit engine, and files without an engine field load as legacy. Single runs with a custom rate or Traffic Manager port also select legacy when the engine is omitted. The CLI still accepts an explicit engine override. [Run guide](../../README_RUNNING.md#v2-single-run-capture)
- Fixed short S10 runs by skipping delayed SUMO flows that start after the configured horizon. V2 checks for a whole-millisecond period and confirms two consecutive CARLA ticks before starting the paired clock. It continues to check every recorded frame for continuity.

| Check | Result |
| --- | --- |
| Standard single-run CLI at 20 Hz | Saved 120 complete paired frames in 6 s of simulation with four radars at 800 rays/s. |
| New default campaign through the CLI | All 11 scenarios launched and recorded 100 complete paired frames per row after 2 s of warm-up. This covered startup and 5 s of recording per scenario. |
| Legacy CLI selection | Both an older plan without an engine field and `--engine legacy` overriding a saved v2 plan completed. Both reproduced the known Windows teardown issue and left five sensors that the smoke test removed. |
| Explicit v2 rates on CARLA 0.9.16 | Two 40 Hz runs saved 80 complete frames each, with a 40-frame 20 Hz run between them. At 25 Hz the startup check stopped before creating a rig or capture and restored CARLA settings. |
| Unit, integration, and legacy traffic suites | 24 unit tests, 17 integration tests, and 21 legacy traffic tests passed. Live checks included GUI defaults and Stop, six CARLA sessions alternating between 20 and 40 Hz, all four radar layouts, and short S10 route startup. |

The final cleanup check found CARLA asynchronous, all 15 traffic lights unfrozen, no sensor or dynamic actors, and no SUMO workers. The isolated CARLA test server was stopped.

## 2026-11-27 · Step 11

- Started the first of two planned 11-scenario passes with 14 radars at 15,000 rays/s each, 20 Hz, and a 240 s recording window after 20 s of warm-up per scenario.
- S01 Free Flow stopped at SUMO time 248.35 s. The mirror still expected CARLA actor 52, which was absent from frame 25774. The remaining scenarios were skipped. The partial capture contains 4,566 complete frames over 228.3 s. The manifest, camera index, PNG files, and actor log agree on frames 21208–25773. The radar CSV contains 37,048,983 return rows.
- A separate traffic-only replay reproduced the disappearance at 248.35 s. SUMO still tracked `ped_stretch_x_east.0` on edge `:532_w2`. CARLA reported its mirror, actor 152 in the replay, as `Invalid` and omitted it from the exact-frame snapshot.
- After the replay, CARLA was asynchronous with 15 unfrozen traffic lights and no dynamic actors or sensors. No SUMO workers remained. The isolated test server was stopped.
- Follow-up controls isolated the trigger using the mirror implementation from the failed capture at `b4f37dd`. The vulnerable walker disappeared 10 s after contact with stopped car `EB_freeflow.31`. Making only the target walker invincible, or moving only that car clear, preserved the walker through 256 s. Both controls reached the same commanded walker position at 248.35 s. [Actor lifecycle finding](AUDIT.md#endurance-finding-in-step-11)
- A temporary invincibility change also passed S01. The project replaced it with [swept actor clearance](ARCHITECTURE.md#scenario-actor-and-signal-synchronization) and restored `is_invincible=false`.
- The boundary version recorded all 4,800 complete S01 frames at 14 radars and 15,000 rays/s. The target walker yielded, resumed using the same ID, and remained in ground truth past 248.35 s. A separate scan checked all 2,822,737 actor pairs across the saved frames and found zero bounding box overlaps. The full all-scenario endurance pass remains pending.
- All 26 unit tests and 17 integration tests passed with the boundaries in place. The crossing regression checks that the mirror holds a blocked pedestrian and moves it at a bounded speed once the vehicle clears.

## 2026-11-28 · Step 11 endurance pass

- The first run recorded all 11 scenarios at 20 Hz with 14 radars at 15,000 rays/s. Each row recorded 240 s after 20 s of warm-up. The saved capture contains 52,800 complete frames, 739,200 radar scans, and 428,568,737 radar returns. The persisted CSV row count matched the writer count. Live checks covered 57,200 paired frames including warm-up and 39,315,167 actor pairs with zero observed bounding box overlaps or unexpected mirror losses.
- The capture closed with CARLA asynchronous, all traffic lights unfrozen, and no dynamic actors. Twenty-four seconds after the campaign record was finalized, the CARLA process crashed during sensor-stream session cleanup. The second pass timed out at its readiness check with zero paired frames. No CARLA or SUMO worker remained from the run.
- The crash dump matched the bundled CARLA executable and PDB. The top CARLA frames were `MultiStreamState::DisconnectSession` and `Dispatcher::DeregisterSession`. The failing call was `_wassert` with expression `session == _session.load()` at line 120 in the installed build's `MultiStreamState.h`. The upstream release source has the same invariant. [CARLA stream teardown source](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/streaming/detail/MultiStreamState.h#L88-L98)
