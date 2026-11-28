# Legacy and v2 capture benchmark

## Setup

The November 27, 2026 benchmark used one offscreen `Town10HD_Opt` CARLA server on Windows 10 build 19045 with an Intel Core i7-9700K and NVIDIA GeForce RTX 2060 (driver 591.59). The CARLA 0.9.16 server and client reported build `1ae535667`. Both engines used Eclipse SUMO 1.28.0 and Python 3.12.10.

Each trial ran a one-row S01 Free Flow campaign with traffic seed 42, density 60, both directions, 30 ambient vehicles, 12 pedestrians, eight bicycles, corridor-priority SUMO signals, and a 150 m mirror radius. Each campaign requested 8 s of warm-up and 22 s of recording at a 0.05 s step (20 Hz). The rigs had an even split of 4, 8, 12 or 14 radars at 3 m, each casting 1,200 rays per second, plus the existing 800 × 600 RGB camera. Both engines started from an idle asynchronous world on the same dedicated endpoint. Scene cleanup and tree hiding were disabled. Labeling, Doppler/RCS processing, and PointNet frame building ran after the timed capture trials.

Both engines were exercised at `853e2a4`. The first pass increased radar count and ran legacy before v2 at each count. The second reversed both orders. The harness wraps the existing entry points for measurement and removes any leftover benchmark rig only after it records the engine's teardown outcome.

### Measurements

- Recording time runs from the first in-window CARLA tick call to the end of the last one. Total time covers the campaign entry point, including setup and teardown, and starts after the benchmark imports the code it instruments. Shutdown time runs from the last in-window tick completion to the campaign return. Both exclude offline processing and benchmark cleanup.
- CARLA tick times are measured around `World.tick`. Actor mirror times cover the actor adapter's `step` in each engine. Legacy runs its mirror after observing a CARLA frame, and v2 applies and acknowledges updates before its paired tick. The raw trace also includes v2's full paired-step latency.
- Sensor callbacks are counted by sensor and frame. Callback lag measures how many returned CARLA ticks a callback is behind its own frame at arrival. Radar scan counts come from processed radar messages, including scans with zero detections. Camera and actor coverage come from saved frame IDs.
- Backlog is sampled after each tick returns. Legacy reports queued radar measurements and pending camera images, including camera encode jobs. V2 reports queued joined frames. The queues hold different units, and a sampled peak can miss a transient spike.

## Results

Each range covers two trials. Every recording window spanned 22.00 CARLA simulation seconds.

| Radars | Legacy recording (wall s) | V2 recording (wall s) | Legacy campaign (wall s) | V2 campaign (wall s) | Legacy mirror updates / 440 | V2 paired updates / 440 |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 4 | 7.58–7.85 | 14.50–21.07 | 26.68–27.47 | 18.77–25.66 | 432, 436 | 440, 440 |
| 8 | 7.34–7.51 | 15.12–15.53 | 26.23–26.53 | 19.29–19.78 | 433, 431 | 440, 440 |
| 12 | 7.77–8.31 | 15.60–15.78 | 26.53–27.18 | 19.88–20.20 | 435, 433 | 440, 440 |
| 14 | 7.71–9.13 | 16.20–16.30 | 27.40–28.60 | 20.53–20.56 | 435, 427 | 440, 440 |

| Measurement | Legacy | V2 |
| --- | ---: | ---: |
| CARLA tick median / p95 (ms) | 15.6–19.3 / 19.9–27.6 | 7.7–11.7 / 16.5–17.2 |
| Actor mirror median / p95 (ms) | 11.6–14.5 / 15.5–22.1 | 9.4–12.5 / 12.2–18.1 |
| Time between CARLA tick starts, p95 (ms) | 20.7–30.1 | 58.9–67.8 |
| Shutdown (wall s) | 2.63–2.98 | 0.45–0.70 |
| Saved camera frames / expected | 3,520 / 3,520 | 3,520 / 3,520 |
| Saved actor frames / expected | 3,520 / 3,520 | 3,520 / 3,520 |
| Written radar scans / expected | 33,440 / 33,440 | 33,440 / 33,440 |
| Recorded CARLA frames with a mirror update | 3,462 / 3,520 | 3,520 / 3,520 |
| Camera callbacks behind returned CARLA tick | 180, up to 3 ticks | 21, up to 4 ticks |
| Radar callbacks behind returned CARLA tick | 0 | 0 |
| Sampled peak backlog | 2–14 radar scans and 4–68 camera images | 8 joined frames |

Both engines saved every expected sensor sample. Every v2 frame manifest entry was complete. Each v2 trial advanced SUMO to exactly 30.00 s after 160 paired warm-up steps and 440 paired recording steps. The legacy runner performed 58 fewer updates than there were recorded CARLA frames across the eight trials. Legacy applies each observed update after CARLA publishes its triggering frame.

Both engines restored world settings and left the signals unfrozen. All eight legacy campaigns returned with their radar and camera actors still present on Windows. The harness removed only those leftover rig actors after recording the outcome. V2 returned with no remaining actors from any trial.

V2's paired-step p95 was 30.0–32.0 ms. Its eight-frame writer queue was full at 67.3–92.3% of recording tick samples. The time between one paired step returning and the next starting had a p95 of 32.3–38.7 ms. This interval includes exact-frame ground-truth collection and submission to the writer. Profile those operations and the writer's serial image encoding before optimizing the simulator adapters.

### Pinned legacy control

The same harness ran one trial per rig in a temporary source archive of `98a4744`. Sensor and actor coverage were complete for all four reference captures. Each reference campaign also left its rig behind after returning.

| Radars | Recording (wall s) | Campaign (wall s) | Mirror updates / 440 | Rig actors remaining |
| ---: | ---: | ---: | ---: | ---: |
| 4 | 7.45 | 26.55 | 433 | 5 |
| 8 | 8.05 | 27.22 | 434 | 9 |
| 12 | 8.70 | 27.68 | 432 | 13 |
| 14 | 8.19 | 27.45 | 435 | 15 |

### Dataset compatibility

All 20 captures had identical raw radar, camera, and processed radar CSV headers. Each sensor's measured position and orientation matched across engines within 0.0001 m or degrees for the same rig. Radar range, field of view and ray budget matched, along with the RGB dimensions and field of view. Each capture contained all 440 referenced PNGs. The first and last PNG of each capture passed image verification.

The existing radar labeler reported zero missing actor frames for all 20 captures. Doppler/RCS processing and the PointNet frame builder completed for every capture. The resulting arrays contained 8,800 valid PointNet frames with the expected scenario and segment IDs. The two v2 runs at each rig size returned the same number of matched radar returns. PointNet point counts could vary after Doppler/RCS processing.

| Radars | Current legacy matches | V2 matches | Pinned legacy matches | PointNet points per capture |
| ---: | ---: | ---: | ---: | ---: |
| 4 | 751, 765 | 779, 779 | 730 | 73,165–73,464 |
| 8 | 1,206, 1,256 | 1,258, 1,258 | 1,255 | 146,895–147,351 |
| 12 | 2,455, 2,411 | 2,409, 2,409 | 2,422 | 210,751–211,351 |
| 14 | 2,450, 2,567 | 2,720, 2,720 | 2,612 | 252,666–253,251 |

### Scope

These measurements cover S01, short single-row campaigns, and a 1,200 rays/s budget on one host. Legacy builds routes with a 60 s margin and lets CARLA run its own signals. V2 uses the shorter scenario horizon, mirrors corridor signal aspects, and keeps SUMO and CARLA on the same step. The seed and requested traffic settings match. Those clock and control differences affect the resulting traffic and radar returns. The pipeline comparison verifies dataset compatibility through frame building.

V2 was opt-in at the time of this benchmark. The measured queue saturation motivated the recording changes in the following sections.

### Reproduce

Start a dedicated offscreen CARLA server on port 22000 with rendering enabled and no other clients or dynamic actors. Run from the repository root with the project venv active. Choose a new output directory for each trial. Replace `legacy` with `v2` and repeat for `4`, `8`, `12` and `14` radars.

```powershell
python -B tests/benchmark_capture.py --port 22000 --engine legacy --radars 4 --output "$env:TEMP/scalesense-legacy-04-r1"
```

Each output directory gets the dataset, bench traces, and `result.json`. For pipeline verification, run the existing processing commands against each resulting `sensor_capture_*` directory.

```powershell
python -B dataset/capture/LabelRadarCapture.py --capture-dir $capture
python -B dataset/capture/PostProcessDataset.py --capture-dir $capture --seed 0
python -B pointnet/build_frames.py $capture
```

To reproduce the pinned controls, export `98a4744` with `git archive` into a separate temporary directory. Copy `tests/benchmark_capture.py` into its `tests` directory and run the legacy command from the exported source with the same venv and endpoint.

## Parallel PNG follow-up

The November 27 follow-up compared v2 from `99424aa` with the parallel PNG writer on the same host, seed, and S01 setup. Each comparison used one run per version. The default-budget trial used 14 radars at 15,000 rays/s and a 60-second recording window. Both versions used the same benchmark instrumentation.

| Radars | Rays/s per radar | Simulation seconds | Serial writer (wall s) | Two PNG encoders (wall s) | Wall time reduction |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 4 | 1,200 | 22 | 14.67 | 10.31 | 29.7% |
| 8 | 1,200 | 22 | 14.40 | 10.95 | 24.0% |
| 12 | 1,200 | 22 | 15.00 | 11.22 | 25.1% |
| 14 | 1,200 | 22 | 15.75 | 11.66 | 26.0% |
| 14 | 15,000 | 60 | 121.90 | 107.89 | 11.5% |

At 1,200 rays/s, the serial writer spent about 29 ms per PNG at the median. Ground-truth lookup took 0.18–0.19 ms and radar CSV writing took 1.61–5.62 ms per frame. Across all five comparisons, frame submission p95 fell from 28.91–33.19 ms to 0.07–0.08 ms. Full writer queue samples fell from 70.0–97.2% to 0.2–1.1%. The queue retained its capacity of eight waiting frames plus the active frame.

At 15,000 rays/s, radar CSV writing took 67.1 ms per frame at the median in the serial baseline and 71.8 ms with two PNG workers. The parallel run also showed longer individual encode times. Paired-step p95 increased from 97.3 to 153.6 ms at that load. The 1,200 rays/s paired-step p95 stayed near 31 ms.

One further default-budget trial used a single PNG worker alongside the ordered writer. It recorded 60 simulated seconds in 105.49 wall seconds, with a paired-step p95 of 125.4 ms and the same complete frame and radar counts. The result covers one trial at each worker count. The encoder default remains two.

Every comparison retained the same number of radar return rows and saved every expected frame. The default-budget captures each contained 1,200 paired frames, 16,800 radar scans, and 9,708,118 radar returns. The four tuned 1,200 rays/s captures passed labeling, Doppler/RCS postprocessing and PointNet frame building. The simulator settings and actor cleanup checks passed for all five pairs.

Reproduce the default-budget trial with the same benchmark harness. Copy the current harness into an export of `99424aa` to measure the serial version. Use a fresh path for each run.

```powershell
python -B tests/benchmark_capture.py --port 22000 --engine v2 --radars 14 --radar-pps 15000 --duration 60 --output "$env:TEMP/scalesense-v2-dense"
```

Set `DATASET_CAMERA_ENCODER_THREADS=1` in the environment to reproduce the one-worker trial.

## Batched radar CSV follow-up

The November 27 comparison used one baseline run from `8c830ba` and one batched-writer run with the same host and S01 settings. Each run used 14 radars at the default 15,000 rays/s, 8 s of warm-up, 60 s of recording, and two PNG workers.

| Measurement | Per-return writer | Batched writer |
| --- | ---: | ---: |
| Recording wall time | 106.08 s | 40.04 s |
| Whole campaign wall time | 110.30 s | 44.28 s |
| Radar row formatting, median per frame | 37.82 ms | 12.31 ms |
| CSV serialization and buffered output, median per frame | 30.54 ms | 2.05 ms |
| Total radar CSV stage, median per frame | 68.47 ms | 14.57 ms |
| Paired-step p95 | 152.18 ms | 46.78 ms |
| Time between recorded tick starts, p95 | 149.25 ms | 47.71 ms |
| Paired frames saved | 1,200 | 1,200 |
| Radar scans written | 16,800 | 16,800 |
| Radar returns written | 9,708,118 | 9,708,118 |

The baseline formatting timer ends when the per-scan writer enters its lock. The remainder includes CSV serialization and buffered file writes. The batched writer timer splits when the preformatted scan is ready for the stream's `write` call. These are wall-clock measurements and include scheduling delays and buffered output.

V2 formats repeated scan fields once, iterates the packed CARLA returns, and writes each scan to the existing CSV in one call. CARLA 0.9.16 stores velocity, azimuth, altitude and depth as four native float32 values per detection. V2 writes the dataset's existing field order and six-decimal formatting. The legacy engine continues using its existing writer. [CARLA detection layout](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/sensor/data/RadarData.h) · [CARLA serializer](https://github.com/carla-simulator/carla/blob/0.9.16/LibCarla/source/carla/sensor/s11n/RadarSerializer.h)

The CSV headers, radar scan counts, and frame order matched across the two captures. After normalizing frame IDs and excluding per-run sensor IDs and absolute timestamps, 31 of 9,708,118 return payloads differed across the independent runs. The live integration suite compared both writers on the same CARLA scan for every sensor at each of the four supported layouts. Those same-scan comparisons matched byte for byte.

An 80-frame excerpt from the optimized full-rate capture contained 647,802 returns. The labeler found 8,300 matches and reported zero missing actor frames. Doppler/RCS processing completed and PointNet built all 80 frames with their expected scenario and segment IDs.

To reproduce, use the same full-rate command as above with `DATASET_CAMERA_ENCODER_THREADS=2`. Copy the current benchmark harness into an export of `8c830ba` for the per-return baseline. Use fresh output paths.

## Repeated full-rate campaigns

The November 27 trials used the same dedicated offscreen CARLA 0.9.16 and SUMO 1.28.0 setup. The [validation plan](../../tests/fixtures/full_rate_campaign.json) defines three rows in order, S01 Free Flow, S03 Heavy Demand, and S04 Stop and Go. The rows use seeds 42, 43, and 44 respectively. Each row runs 8 s of warm-up and records 60 s at 20 Hz. All three rows share one 14-radar rig at 15,000 rays/s per radar and one RGB camera. Both trials used two PNG workers and reused the same CARLA world.

Recording wall time is the sum of the three recording windows. Whole-campaign time includes setup, warm-up, row transitions, and teardown. Tick interval percentiles use consecutive ticks inside each recording window. Dataset processing runs separately.

| Recording window | Simulated seconds | Run 1 wall seconds | Run 2 wall seconds | Tick p95, run 1 / run 2 |
| --- | ---: | ---: | ---: | ---: |
| S01 Free Flow | 60 | 38.87 | 39.47 | 47.51 / 47.95 ms |
| S03 Heavy Demand | 60 | 45.31 | 44.70 | 51.48 / 51.70 ms |
| S04 Stop and Go | 60 | 41.32 | 41.70 | 48.83 / 50.74 ms |
| All recording windows | 180 | 125.50 | 125.86 | 49.24 / 49.81 ms |
| Whole campaign | 204 paired seconds with warm-up | 137.40 | 137.76 | |

Both campaigns recorded 3,600 complete paired frames, 3,600 camera images and 50,400 radar scans. The two runs wrote 29,163,671 and 29,163,442 radar returns. The counts differed by 229 across the independent runs with the same seeds. The eight-frame writer queue reached capacity at 26 and 45 of 3,600 sampled recording ticks. All six recording windows averaged faster than real time. Heavy Demand exceeded the 50 ms tick budget at p95 in both runs, and Stop and Go exceeded it in the second run.

Every radar CSV scan matched its recorded frame, sensor ID, and return count. The camera index, actor log, frame manifest, and `segments.json` agreed on the exact recording ranges. Each row reset SUMO time, used a fresh SUMO process, and recorded a distinct set of dynamic actor IDs. Each campaign used a single rig and restored CARLA's settings and signal timing on exit.

The 60-frame excerpt included 20 frames from each scenario and 484,801 radar returns. Labeling reported zero missing actor frames and 10,784 matches. Doppler/RCS processing completed, and PointNet built all 60 frames with the expected scenario and segment IDs. The full-rate Stop regression retained 100 frames from the completed row and 25 from the interrupted second row, skipped the third row, and immediately completed another full-rate capture on the same world.

These results cover one workstation and three scenarios. Legacy was the default when these runs were recorded. [Rollout history](PROGRESS.md#2026-11-27--step-10)

To reproduce, run the two commands in sequence on the same dedicated asynchronous CARLA endpoint. Use new output directories. The harness loads the plan and writes aggregate and per-row timings in `result.json`.

```powershell
$env:DATASET_CAMERA_ENCODER_THREADS = '2'
python -B tests/benchmark_capture.py --engine v2 --port 22000 --campaign tests/fixtures/full_rate_campaign.json --output "$env:TEMP/scalesense-full-rate-1"
python -B tests/benchmark_capture.py --engine v2 --port 22000 --campaign tests/fixtures/full_rate_campaign.json --output "$env:TEMP/scalesense-full-rate-2"
```

## Full-load tick latency follow-up

Two S03 Heavy Demand trials on the same CARLA server recorded 120 s at 20 Hz after 20 s of warm-up, with 14 radars at 15,000 rays/s each. Both used density 60, 30 ambient vehicles, 20 pedestrians, 10 bicycles, and seed 42. Each saved 2,400 complete frames and 33,600 radar scans, and each restored the world with no actors left behind.

| Trial | Recording wall time | Tick interval p95 | Actor mirror p95 |
| --- | ---: | ---: | ---: |
| Baseline (`dd0141f`) | 108.90 s | 59.04 ms | 43.07 ms |
| Experimental spatial filter | 117.03 s | 66.99 ms | 50.35 ms |

The spatial filter grouped occupied bounds by x coordinate before applying the existing swept collision checks. It was dropped after the matched trial. In a separate 60 s profile of the original mirror, per-frame TraCI position, heading, and lane reads took 15.53 ms at the median and 28.93 ms at p95. Planning the swept bounds took 3.22 ms at the median and 4.23 ms at p95. The profiled tick interval was 58.48 ms at p95. These are wall-clock timings from one workstation. The next experiment is to reduce TraCI round trips while preserving the pose recorded for each paired frame.

To profile a workload, pass `--profile-mirror` to the v2 benchmark. The result JSON includes `traci_pose_queries` and `mirror_plan` under `recording_stages`. Use the same campaign and profiler settings across trials.
