
import os
import numpy as np
import matplotlib.cm as cm

from pydrake.all import  CollisionFilterDeclaration


def generate_rope_sdf_from_config(rope_config, rope_name):
    gen = SDFGenerator(model_name=rope_name,
                       num_segments=rope_config["num_segments"],
                       segment_length=rope_config["segment_length"],
                       link_mass=rope_config["link_mass"],
                       rope_radius=rope_config["rope_radius"],
                       joint_limit=rope_config["joint_limit"],
                       joint_damping=rope_config["joint_damping"],
                       joint_friction=rope_config["joint_friction"],
                       spring_stiffness=rope_config["spring_stiffness"],
                       rope_height=rope_config["rope_height"],
                       rope_x=rope_config["rope_x"],
                       rope_y=rope_config["rope_y"])
    return gen.get_rope_sdf_string()


def filter_collisions(rope_config, plant, scene_graph, rope_name):
    for i in range(1, rope_config["num_segments"]):
        body_a = plant.GetBodyByName(f"{rope_name}_capsule_{i}")
        set_a = plant.CollectRegisteredGeometries([body_a])

        body_b = plant.GetBodyByName(f"{rope_name}_capsule_{i + 1}")
        set_b = plant.CollectRegisteredGeometries([body_b])
        # scene_graph.ExcludeCollisionsBetween(set_a, set_b)
        scene_graph.collision_filter_manager().Apply(
            CollisionFilterDeclaration().ExcludeBetween(set_a, set_b)
        )


def initialize_rope_zero(diagram, simulator, station, rope_name):
    simulator_context = simulator.get_mutable_context()
    station_context = diagram.GetMutableSubsystemContext(station, simulator_context)

    rope_model = station.model_ids[rope_name]
    q_len = station.mbp.num_positions(rope_model)
    v_len = station.mbp.num_velocities(rope_model)
    q_rope = np.zeros(q_len)
    q_rope[0] = -math.pi / 2

    station.set_model_state(station_context, rope_name, q_rope, np.zeros(v_len))


def post_finalize_rope_settings(config, mbp, sg):
    mbp.set_penetration_allowance(config["env"]["penetration_allowance"])
    mbp.set_stiction_tolerance(config["env"]["stiction_tolerance"])
    filter_collisions(config["rope"], mbp, sg, config["rope"]["name"])


class SDFGenerator():
    def __init__(self, model_name, num_segments,
                 segment_length, link_mass, rope_radius,
                 joint_limit, joint_damping, joint_friction,
                 spring_stiffness, rope_height, rope_x, rope_y):
        # Getting parameters
        self.joint_limit = joint_limit
        self.joint_damping = joint_damping
        self.joint_friction = joint_friction
        self.num_segments = num_segments

        if type(link_mass) == float:
            self.link_masses = np.repeat(link_mass, self.num_segments)
        else:
            self.link_masses = np.array(link_mass)

        self.rope_height = rope_height
        self.rope_radius = rope_radius
        if type(segment_length) == float:
            self.segment_lengths = np.repeat(segment_length, self.num_segments)
        else:
            self.segment_lengths = np.array(segment_length)
        self.spring_stiffness = spring_stiffness
        self.rope_x = rope_x
        self.rope_y = rope_y
        self.model_name = model_name

        # Calculate moments of interia
        self.link_ixxs = ((1./12) * self.link_masses
                         * (3 * np.square(self.rope_radius)
                            + np.square(self.segment_lengths)))
        self.link_iyys = self.link_ixxs
        self.link_izzs = ((1./2) * self.link_masses
                                * self.rope_radius ** 2)

        # Getting fragments
        self.model_start_frag = (f"<sdf version='1.6'>\n\t"
                                 f"<model name='{self.model_name}'>\n")
        self.model_end_frag = "\t</model>\n</sdf>\n"

        # Get paths
        base_folder = os.path.abspath(os.path.dirname(os.path.dirname(__file__)))
        model_folder = os.path.join(base_folder, "config/model/")
        link_filename = os.path.join(model_folder, "capsule.frag.xml")
        joint_filename = os.path.join(model_folder, "capsule_joint.frag.xml")
        # Read capsule fragment
        with open(link_filename, "r") as capsule_file:
            self.capsule_frag = capsule_file.read()

        # Read link joint fragment
        with open(joint_filename, "r") as capsule_joint_file:
            self.capsule_joint_frag = capsule_joint_file.read()

    @staticmethod
    def apply_replacements(s, replace_dict):
        new_s = s
        for key in replace_dict:
            new_s = new_s.replace(key, replace_dict[key])
        return new_s

    def print_parameters(self):
        print(f"Joint limit: {self.joint_limit}")
        print(f"Joint damping: {self.joint_damping}")
        print(f"Joint friction: {self.joint_friction}")
        print(f"Link mass: {self.link_masses}")
        print(f"Num segments: {self.num_segments}")
        print(f"Rope height: {self.rope_height}")
        print(f"Rope radius: {self.rope_radius}")
        print(f"Segment length: {self.segment_lengths}")
        print(f"Spring stiffness: {self.spring_stiffness}")
        print(f"X: {self.rope_x}")
        print(f"Y: {self.rope_y}")

    def get_rope_sdf_str(self):
        rope_sdf_str = self.model_start_frag
        curr_z = self.rope_height
        # Add the rest of the rope
        for i in range(0, self.num_segments):
            frac = i / self.num_segments
            # r, g, b, a = cm.plasma(frac)
            r, g, b, a = cm.ocean(frac)
            capsule_str = SDFGenerator.apply_replacements(
                self.capsule_frag, {
                    "{height}": str(curr_z),
                    "{name}": f"{self.model_name}_capsule_{i + 1}",
                    "{length}": str(self.segment_lengths[i]),
                    "{half_length}": str(self.segment_lengths[i] / 2.0),
                    "{radius}": str(self.rope_radius),
                    "{link_mass}": str(self.link_masses[i]),
                    "{link_ixx}": str(self.link_ixxs[i]),
                    "{link_iyy}": str(self.link_iyys[i]),
                    "{link_izz}": str(self.link_izzs[i]),
                    "{x}": str(self.rope_x),
                    "{y}": str(self.rope_y),
                    "{r}": str(r),
                    "{g}": str(g),
                    "{b}": str(b)
                }
            )

            revolute_str = SDFGenerator.apply_replacements(
                self.capsule_joint_frag, {
                    "{effort_int}": "0",
                    "{height}": str(curr_z),
                    "{joint1_name}": f"{self.model_name}_capsule_{i}",
                    "{joint2_name}": f"{self.model_name}_capsule_{i + 1}",
                    "{limit}": str(self.joint_limit),
                    "{damping}": str(self.joint_damping),
                    "{friction}": str(self.joint_friction),
                    "{spring_stiffness}": str(self.spring_stiffness),
                    "{x}": str(self.rope_x),
                    "{y}": str(self.rope_y)
                }
            )
            curr_z -= self.segment_lengths[i]
            if i == 0:
                revolute_str = ""
            rope_sdf_str += capsule_str
            rope_sdf_str += revolute_str

        rope_sdf_str += self.model_end_frag
        return rope_sdf_str

    def get_rope_sdf_string(self):
        return self.get_rope_sdf_str()
