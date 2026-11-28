"""RadarSumoFusion -- headless fused run.

V2 advances SUMO, mirrors its traffic in CARLA, and records paired sensor frames.
Use --engine legacy to select the original coordinator.

Example:
    python run_fusion.py --scenario 3 --density 60 --duration 300 \
        --radars 8 --rate-hz 20 --pedestrians 20 --bicycles 10

V2 campaign (several scenarios, one capture) from a file saved by fusion_gui.py:
    python run_fusion.py --engine v2 --campaign my_campaign.json

Prereqs: CarlaUE4 running on Town10HD_Opt, SUMO_HOME set, venv active
(carla, traci, sumolib installed). Run once first:  python traffic/build_network.py
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

from fusion.config import DEFAULT_ENGINE, DEFAULT_RATE_HZ, FusionConfig, MAX_RADARS_PER_SIDE, split_radar_count
from fusion.orchestrator import run
from fusion.campaign import CampaignConfig, run_campaign
from fusion.traffic_session import cooperative_stop


def _bool_flag(parser, name, default, help_on, help_off):
    dest = name.replace("-", "_")
    grp = parser.add_mutually_exclusive_group()
    grp.add_argument(f"--{name}", dest=dest, action="store_true", help=help_on)
    grp.add_argument(f"--no-{name}", dest=dest, action="store_false", help=help_off)
    parser.set_defaults(**{dest: default})


def _migrate_campaign(source, destination, overrides):
    data = json.loads(Path(source).read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("a campaign must contain a JSON object")
    rows = data.get("runs", [])
    if not isinstance(rows, list) or any(not isinstance(row, dict) for row in rows):
        raise ValueError("campaign runs must be a list of JSON objects")
    data.update(overrides)
    data["engine"] = "v2"
    cfg = CampaignConfig.from_dict(data)
    errors = cfg.validate()
    if errors:
        raise ValueError(", ".join(errors))
    contents = json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n"
    with Path(destination).open("x", encoding="utf-8") as output:
        output.write(contents)
    print(f"Saved v2 campaign to {destination}")


def main() -> int:
    # UTF-8 stdout on Windows consoles.
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")

    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    # traffic
    p.add_argument("--scenario", type=int, default=1, help="SUMO scenario 1-11")
    p.add_argument("--density", type=int, default=50, help="traffic density 1-100")
    p.add_argument("--duration", type=int, default=300, help="capture seconds")
    p.add_argument("--direction", default="BOTH", choices=["WB", "EB", "BOTH"])
    p.add_argument("--ambient-vehicles", type=int, default=20)
    p.add_argument("--pedestrians", type=int, default=20)
    p.add_argument("--bicycles", type=int, default=10)
    p.add_argument("--ambient-seed", type=int, default=42)
    p.add_argument("--render-radius", type=float, default=150.0)
    p.add_argument("--no-cull", action="store_true", help="mirror the whole city")
    p.add_argument("--sumo-gui", action="store_true", help="show sumo-gui")
    # sensors / capture
    p.add_argument("--radars", type=int, default=None,
                   help="total radars, split evenly between the kerbs (south takes "
                        "the odd one); default 8. Overridden per side by "
                        "--radars-south / --radars-north")
    p.add_argument("--radars-south", type=int, default=None,
                   help=f"radars on the south kerb row (0-{MAX_RADARS_PER_SIDE})")
    p.add_argument("--radars-north", type=int, default=None,
                   help=f"radars on the north kerb row (0-{MAX_RADARS_PER_SIDE})")
    p.add_argument("--radar-pps", type=int, default=None,
                   help="radar rays per second (CARLA points_per_second, 500-20000; "
                        "default 15000 -> ~580 returns/scan at 20 Hz; 800 -> ~30, "
                        "1300 -> ~50). Also overrides a --campaign's value.")
    p.add_argument("--radar-height", type=float, default=3.0,
                   help="radar mount height above the road (m)")
    _bool_flag(p, "label", True,
               "run radar labeling after capture", "skip radar labeling")
    _bool_flag(p, "postprocess", True,
               "run post-processing after capture", "skip post-processing")
    p.add_argument("--capture-base-dir", help="override output root")
    # shared clock / carla
    p.add_argument("--rate-hz", type=float,
                   help="shared tick rate for a single run or campaign migration (single-run default: 20 Hz)")
    p.add_argument("--carla-host")
    p.add_argument("--carla-port", type=int)
    p.add_argument("--tm-port", type=int,
                   help="legacy single-run Traffic Manager port, default 8000 (implies legacy if --engine is omitted)")
    # orchestration
    _bool_flag(p, "scene-cleanup", True,
               "clear parked cars/trash first", "skip scene cleanup")
    _bool_flag(p, "clear-trees", True,
               "hide trees over the monitored stretch", "keep trees")
    p.add_argument("--sync-timeout", type=float, default=180.0)
    p.add_argument("--engine", choices=("legacy", "v2"),
                   help="capture engine (default: v2 for standard 20 Hz single runs or the saved campaign setting)")
    p.add_argument("--campaign", default="",
                   help="run a saved campaign JSON; --engine, --carla-host, --carla-port, "
                        "--radar-pps and --capture-base-dir can override its settings")
    p.add_argument("--migrate-to-v2", metavar="OUTPUT",
                   help="validate and save --campaign as a new v2 plan without running it")
    args = p.parse_args()

    campaign_overrides = {
        name: value for name, value in (
            ("engine", args.engine), ("carla_host", args.carla_host), ("carla_port", args.carla_port),
            ("radar_points_per_second", args.radar_pps), ("capture_base_dir", args.capture_base_dir),
        ) if value is not None
    }
    if args.migrate_to_v2 is not None:
        if not args.campaign:
            p.error("--migrate-to-v2 requires --campaign")
        if args.engine == "legacy" or args.tm_port is not None:
            p.error("--migrate-to-v2 requires v2 settings. Remove --engine legacy and --tm-port.")
        if args.rate_hz is not None:
            campaign_overrides["rate_hz"] = args.rate_hz
        try:
            _migrate_campaign(args.campaign, args.migrate_to_v2, campaign_overrides)
        except (OSError, TypeError, ValueError) as exc:
            p.error(f"cannot migrate campaign: {exc}")
        return 0

    if args.campaign:
        try:
            ccfg = CampaignConfig.load(args.campaign)
        except (OSError, TypeError, ValueError) as exc:
            p.error(f"cannot load campaign: {exc}")
        for name, value in campaign_overrides.items():
            setattr(ccfg, name, value)
        if ccfg.engine == "v2":
            with cooperative_stop() as stop_event:
                result = run_campaign(ccfg, stop_event=stop_event)
            return _exit_code(result)
        return 0 if run_campaign(ccfg).ok else 1

    south, north = split_radar_count(8 if args.radars is None else args.radars)
    if args.radars_south is not None:
        south = args.radars_south
    if args.radars_north is not None:
        north = args.radars_north

    # Preserve existing single-run commands using a custom clock or Traffic Manager.
    rate_hz = DEFAULT_RATE_HZ if args.rate_hz is None else args.rate_hz
    legacy_options = args.tm_port is not None or rate_hz != DEFAULT_RATE_HZ
    engine = args.engine or ("legacy" if legacy_options else DEFAULT_ENGINE)
    if engine == "v2" and args.tm_port is not None:
        p.error("--tm-port applies to legacy single runs. Use --engine legacy.")

    cfg = FusionConfig(
        scenario=args.scenario, density=args.density, duration=args.duration,
        direction=args.direction, ambient_vehicles=args.ambient_vehicles,
        pedestrians=args.pedestrians, bicycles=args.bicycles,
        ambient_seed=args.ambient_seed, render_radius=args.render_radius,
        no_cull=args.no_cull, sumo_gui=args.sumo_gui,
        radars_south=south, radars_north=north, radar_height_m=args.radar_height,
        radar_points_per_second=args.radar_pps or 0,
        label=args.label, postprocess=args.postprocess,
        capture_base_dir=args.capture_base_dir or "",
        rate_hz=rate_hz, carla_host=args.carla_host or "127.0.0.1",
        carla_port=2000 if args.carla_port is None else args.carla_port,
        traffic_manager_port=8000 if args.tm_port is None else args.tm_port,
        scene_cleanup=args.scene_cleanup, clear_trees=args.clear_trees,
        sync_timeout=args.sync_timeout,
    )
    if engine == "v2":
        from fusion.campaign_v2 import run_single_v2

        with cooperative_stop() as stop_event:
            result = run_single_v2(cfg, stop_event=stop_event)
        return _exit_code(result)
    result = run(cfg)
    return 0 if result.ok else 1


def _exit_code(result):
    if result.error:
        return 1
    if result.stopped:
        return 130
    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
