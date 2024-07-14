import numpy as np

from pydrake.all import RigidTransform, RollPitchYaw

def get_data_from_dict(data, possible_keys):
    for key in possible_keys:
        if key in data:
            return data[key], key
    raise ValueError("None of possible keys found: " + possible_keys)


def transform_from_dict(data):
    pos, _ = get_data_from_dict(data, ['pos', 'position', 'translation'])
    orientation, key = get_data_from_dict(data, ['quat', 'quaternion', 'rpy'])
    if key == "rpy":
        return RigidTransform(RollPitchYaw(orientation), pos)
    else:
        return RigidTransform(Quaternion(orientation), pos)
