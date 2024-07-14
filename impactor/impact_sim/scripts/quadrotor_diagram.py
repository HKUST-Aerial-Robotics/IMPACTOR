from future.utils import iteritems

from pydrake.all import (
    Parser, PropellerInfo, Propeller, CollisionFilterDeclaration, RigidTransform
)

from drake_sim_diagram import DrakeSimDiagram
from leaf_systems import CustomQuadrotorController, RemoteController, Accelerometer
from rope_utils import  generate_rope_sdf_from_config, post_finalize_rope_settings
from transform_utils import transform_from_dict


class QuadrotorDiagram(DrakeSimDiagram):
    def __init__(self, config):
        DrakeSimDiagram.__init__(self, config["env"])

        self._config = config
        self._subsystems = dict()

    # === Property accessors ========================================
    @property
    def subsystems(self):
        return self._subsystems

    # === Add physical components ===================================
    def add_quadrotor(self):
        quad_urdf = self._config["quadrotor"]["urdf_path"]
        quad_name = self._config["quadrotor"]["name"]
        gravity_vector = self._mbp.gravity_field().gravity_vector()
        
        parser = Parser(self._mbp, self._sg)
        quad_model_id = parser.AddModelFromFile(quad_urdf, quad_name)
        quad_body_idx = self._mbp.GetBodyIndices(quad_model_id)[0]
        self._model_ids[quad_name+"_model"] = quad_model_id
        self._body_ids[quad_name+"_body"] = quad_body_idx
        
        quad_propeller_info_list = []
        for rotor_name, rotor_config in iteritems(self._config["quadrotor"]["propellers"]):
            X_Q = transform_from_dict(rotor_config["pose"])
            quad_propeller_info_list.append(
                PropellerInfo(body_index=quad_body_idx, X_BP=X_Q,
                              thrust_ratio=rotor_config["thrust_radio"], 
                              moment_ratio=rotor_config["moment_ratio"]))
        propellers = self._builder.AddSystem(Propeller(
            propeller_info=quad_propeller_info_list))

        accelerometer = self._builder.AddSystem(Accelerometer(
            body_idx=quad_body_idx, X_BS=RigidTransform(), gravity_vector=gravity_vector))
        
        self._subsystems["propellers"] = propellers
        self._subsystems["quad_accelerometer"] = accelerometer

        def finalize_func():
            builder = self._builder
            # connect mbp and propellers
            builder.Connect(self._mbp.get_body_poses_output_port(),
                            propellers.get_body_poses_input_port())
            builder.Connect(propellers.get_spatial_forces_output_port(),
                            self._mbp.get_applied_spatial_force_input_port())
            # connect mbp and accelerometer
            builder.Connect(self._mbp.get_body_poses_output_port(),
                            accelerometer.get_input_port(0))
            builder.Connect(self._mbp.get_body_spatial_velocities_output_port(),
                            accelerometer.get_input_port(1))
            builder.Connect(self._mbp.get_body_spatial_accelerations_output_port(),
                            accelerometer.get_input_port(2))
        self._finalize_functions.append(finalize_func)

    def add_rope_model(self):
        X_Q = transform_from_dict(self._config["quadrotor"]["rope_base"])
        rope_name = self._config["rope"]["name"]

        parser = Parser(self._mbp, self._sg)
        rope_sdf = generate_rope_sdf_from_config(self._config["rope"], rope_name)
        rope_model_ids = parser.AddModelsFromString(file_contents=rope_sdf, file_type="sdf")
        rope_model_id = rope_model_ids[0]
        self._model_ids[rope_name+"_model"] = rope_model_id
        self._mbp.WeldFrames(self._mbp.GetFrameByName(f"base_link"), self._mbp.GetFrameByName(f"{rope_name}_capsule_1"), X_Q)

        def finalize_func():
            post_finalize_rope_settings(self._config, self._mbp, self._sg)
        self._finalize_functions.append(finalize_func)

    def add_payload_model(self):
        X_C = transform_from_dict(self._config["quadrotor"]["payload_base"])
        payload_urdf = self._config["payload"]["urdf_path"]
        payload_name = self._config["payload"]["name"]
        rope_name = self._config["rope"]["name"]
        rope_num = self._config["rope"]["num_segments"]
        gravity_vector = self._mbp.gravity_field().gravity_vector()

        parser = Parser(self._mbp, self._sg)
        payload_model_id = parser.AddModelFromFile(payload_urdf, payload_name)
        payload_body_idx = self._mbp.GetBodyIndices(payload_model_id)[0]
        self._model_ids[payload_name+"_model"] = payload_model_id
        self._body_ids[payload_name+"_body"] = payload_body_idx
        self._mbp.WeldFrames(self._mbp.GetFrameByName(f"{rope_name}_capsule_{rope_num}"), 
                             self._mbp.GetFrameByName(f"payload_link"), X_C)
        
        accelerometer = self._builder.AddSystem(Accelerometer(
            body_idx=payload_body_idx, X_BS=RigidTransform(), gravity_vector=gravity_vector))

        self._subsystems["load_accelerometer"] = accelerometer

        def finalize_func():
            builder = self._builder

            # connect mbp and accelerometer
            builder.Connect(self._mbp.get_body_poses_output_port(),
                            accelerometer.get_input_port(0))
            builder.Connect(self._mbp.get_body_spatial_velocities_output_port(),
                            accelerometer.get_input_port(1))
            builder.Connect(self._mbp.get_body_spatial_accelerations_output_port(),
                            accelerometer.get_input_port(2))

            self._mbp.set_penetration_allowance(self._config["env"]["penetration_allowance"])
            self._mbp.set_stiction_tolerance(self._config["env"]["stiction_tolerance"])
            body_a = self._mbp.GetBodyByName(f"{rope_name}_capsule_{rope_num}")
            set_a = self._mbp.CollectRegisteredGeometries([body_a])

            body_b = self._mbp.GetBodyByName(f"payload_link")
            set_b = self._mbp.CollectRegisteredGeometries([body_b])
            self._sg.collision_filter_manager().Apply(
                CollisionFilterDeclaration().ExcludeBetween(set_a, set_b)
            )
        self._finalize_functions.append(finalize_func)

    def add_controller(self):
        quad_name = self._config["quadrotor"]["name"]
        payload_name = self._config["payload"]["name"]
        propellers = self._subsystems["propellers"]
        quad_accelerometer = self._subsystems["quad_accelerometer"]
        load_accelerometer = self._subsystems["load_accelerometer"]
        
        # PD Hover controller
        if not self._config["env"]["use_remote_control"]:
            controller = self._builder.AddSystem(CustomQuadrotorController(quad_body_idx=self._body_ids[f"{quad_name}_body"],
                                                                           load_body_idx=self._body_ids[f"{payload_name}_body"],
                                                                           accelerometers=[quad_accelerometer, load_accelerometer],
                                                                           num_propellers=propellers.num_propellers()))
        # Remote controller
        else:
            controller = self._builder.AddSystem(RemoteController(quad_body_idx=self._body_ids[f"{quad_name}_body"],
                                                                  load_body_idx=self._body_ids[f"{payload_name}_body"],
                                                                  accelerometers=[quad_accelerometer, load_accelerometer],
                                                                  num_propellers=propellers.num_propellers()))

        self._subsystems["controller"] = controller

        def finalize_func():
            builder = self._builder

            # connect propellers and controller
            builder.Connect(self._mbp.get_body_poses_output_port(),
                            controller.get_input_port(0))
            builder.Connect(self._mbp.get_body_spatial_velocities_output_port(),
                            controller.get_input_port(1))
            builder.Connect(controller.get_output_port(),
                            propellers.get_command_input_port())

        self._finalize_functions.append(finalize_func)

    # === Finalize the completed diagram ============================
    def finalize(self):
        super(QuadrotorDiagram, self).finalize()