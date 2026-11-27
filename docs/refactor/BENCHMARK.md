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

V2 remains opt-in. The measured queue saturation makes the recording path the priority for the next performance pass. Rerun this matrix after tuning and extend the measurements to longer campaigns and the default 15,000 rays/s budget before considering a default change.

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
