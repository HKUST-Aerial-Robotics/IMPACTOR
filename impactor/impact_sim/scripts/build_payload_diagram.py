import numpy as np

from pydrake.all import (
    DiagramBuilder, Simulator
)

from quadrotor_diagram import QuadrotorDiagram


def build_payload_diagram(config):
    builder = DiagramBuilder()

    station = builder.AddSystem(QuadrotorDiagram(config))
    station.add_quadrotor()
    station.add_rope_model()
    station.add_payload_model()
    station.add_controller()

    if config["env"]["visualization"]:
        station.connect_to_drake_visualizer()

    station.finalize()

    # TODO: add high level planner to diagram
    planner = None

    diagram = builder.Build()
    simulator = Simulator(diagram)
    systems = {"station": station, "planner": planner}

    reset_simulator_from_config(config, simulator, diagram, systems)

    return simulator, diagram, systems


def reset_simulator_from_config(config, simulator, diagram, systems):
    sim_context = simulator.get_mutable_context()
    
    # reset quadrotor pos
    quad_sys = systems["station"]
    quad_context = diagram.GetMutableSubsystemContext(quad_sys.mbp, sim_context)
    q = np.ones(quad_sys.mbp.num_positions())
    q[:4] = [1.0, 0.0, 0.0, 0.0]    # attitude of quadrotor
    q[4:7] = [0.0, 0.0, 1.2]        # position of quadrotor
    q[7:] = 0.0 * q[7:]            # joint angles of rope
    q[-4:-3] = 0.0                    # joint angle of payload
    # set q in state [q, v]
    quad_sys.mbp.SetPositions(quad_context, q)

    # TODO: reset planner

    simulator.set_target_realtime_rate(config['env']['target_realtime_rate'])
    simulator.set_publish_every_time_step(config['env']['publish_every_time_step'])
    sim_context.SetTime(0.0)
    simulator.Initialize()