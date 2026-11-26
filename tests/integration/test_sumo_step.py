from itertools import count
from pathlib import Path
import unittest

from fusion.simulation_engine import SimulationEngine
from traffic import validate_scenarios as validator

traci = validator.traci


class SumoStepIntegrationTests(unittest.TestCase):
    def run_session(self, initial_carla_frame):
        fixed_delta_s = 0.05
        duration = 10
        config, temp_files = validator.build_files(
            scenario_id=1,
            density=60,
            duration=duration,
            direction="BOTH",
            ambient_vehicles=0,
            pedestrians=0,
            bicycles=0,
            seed=42,
            step_length=fixed_delta_s,
        )
        try:
            traci.start([
                validator.sumo_binary(), "-c", config, "--no-step-log",
                "--duration-log.disable", "true",
            ])
            try:
                return self.step_session(initial_carla_frame, duration, fixed_delta_s)
            finally:
                traci.close()
        finally:
            for path in temp_files:
                Path(path).unlink(missing_ok=True)

    def step_session(self, initial_carla_frame, duration, fixed_delta_s):
        frame_ids = count(initial_carla_frame + 1)
        vehicle_ids_by_step = []

        def advance_sumo():
            traci.simulationStep()
            return traci.simulation.getTime()

        def read_traffic():
            vehicle_ids_by_step.append(tuple(sorted(traci.vehicle.getIDList())))

        self.assertEqual(traci.simulation.getDeltaT(), fixed_delta_s)
        engine = SimulationEngine(
            fixed_delta_s=fixed_delta_s,
            initial_sumo_time_s=traci.simulation.getTime(),
            initial_carla_frame=initial_carla_frame,
            advance_sumo=advance_sumo,
            sync_traffic=read_traffic,
            tick_carla=lambda: next(frame_ids),
        )
        step_count = round(duration / fixed_delta_s)
        frames = [engine.step() for _ in range(step_count)]
        self.assertEqual(frames[-1].sumo_time_s, duration)
        self.assertEqual(
            [frame.carla_frame for frame in frames],
            list(range(initial_carla_frame + 1, initial_carla_frame + step_count + 1)),
        )
        self.assertEqual(len(vehicle_ids_by_step), step_count)
        self.assertTrue(any(vehicle_ids_by_step))
        return [frame.sumo_time_s for frame in frames], vehicle_ids_by_step

    def test_repeated_sumo_sessions_pair_each_step_with_a_simulated_carla_frame(self):
        first = self.run_session(initial_carla_frame=500)
        second = self.run_session(initial_carla_frame=700)

        self.assertEqual(first, second)
