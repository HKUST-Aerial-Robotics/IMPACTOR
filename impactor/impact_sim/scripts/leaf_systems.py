import os
import numpy as np
from scipy.spatial.transform import Rotation as R

import rospy
from std_msgs.msg import Float32MultiArray

from pydrake.all import (
    LeafSystem, BasicVector, AbstractValue, SpatialVelocity_, SpatialAcceleration_, RigidTransform_, InputPortIndex
)


class Accelerometer(LeafSystem):
    def __init__(self, body_idx, X_BS, gravity_vector):
        LeafSystem.__init__(self)

        self._body_idx = body_idx
        self._X_BS = X_BS
        self._gravity = gravity_vector

        self._measurement = np.zeros(3)

        # Set up input and output ports
        self.DeclareAbstractInputPort("body_poses", AbstractValue.Make([RigidTransform_[float]()]))
        self.DeclareAbstractInputPort("body_spatial_velcocities", AbstractValue.Make([SpatialVelocity_[float]()]))
        self.DeclareAbstractInputPort("body_spatial_accelerations", AbstractValue.Make([SpatialAcceleration_[float]()]))
        # self.DeclareVectorInputPort("_", BasicVector(3))
        self.DeclareVectorOutputPort("acceleration", 
            BasicVector(3), self.CalcOutput)#, 
            # {self.input_port_ticket(InputPortIndex(0)),
            # self.input_port_ticket(InputPortIndex(1)),
            # self.input_port_ticket(InputPortIndex(2)),
            # self.input_port_ticket(InputPortIndex(3))})

    @property
    def measurement(self):
        return self._measurement
    
    def CalcOutput(self, context, output):
        poses_state = self.get_input_port(0).Eval(context)
        velocities_state = self.get_input_port(1).Eval(context)
        accelerations_state = self.get_input_port(2).Eval(context)

        X_WB = poses_state[self._body_idx]
        V_WB = velocities_state[self._body_idx]
        A_WB = accelerations_state[self._body_idx]

        R_WB = X_WB.rotation()
        p_BS_W = R_WB.multiply(self._X_BS.translation()) # vector(3,1)
        R_BS = self._X_BS.rotation()
        A_WS = A_WB.Shift(p_BS_W, V_WB.rotational()) # SpatialAcceleration
        R_SW = R_BS.inverse().multiply(R_WB.inverse())
        
        self._measurement = R_SW.multiply(A_WS.translational() - self._gravity)

        output.SetFromVector(self._measurement)


class RemoteController(LeafSystem):
    def __init__(self, quad_body_idx, load_body_idx, accelerometers, num_propellers=4):
        LeafSystem.__init__(self)

        self._quad_idx = quad_body_idx
        self._load_idx = load_body_idx

        self._quad_accelerometer = accelerometers[0]
        self._load_accelerometer = accelerometers[1]

        self._state_msg = Float32MultiArray()
        self._command_msg = Float32MultiArray()
        # receive command from remote controller
        self._command_sub = rospy.Subscriber("/ctrl_command", Float32MultiArray, self.SubsCallback)
        # publish state to remote controller in 5 kHz
        self._state_pub = rospy.Publisher("/sim_robot_state", Float32MultiArray, queue_size=1)

        # Set up input and output ports
        self.DeclareAbstractInputPort("robot_pose", AbstractValue.Make([RigidTransform_[float]()]))
        self.DeclareAbstractInputPort("robot_velocity", AbstractValue.Make([SpatialVelocity_[float]()]))
        self.DeclareVectorOutputPort("propeller_force", BasicVector(num_propellers), self.CalcOutput)

    def SubsCallback(self, msg):
        self._command_msg = msg

    def PublishCallback(self, event):
        # pass
        self._state_pub.publish(self._state_msg)

    def CalcOutput(self, context, output):
        poses_state = self.get_input_port(0).Eval(context)
        velocities_state = self.get_input_port(1).Eval(context)

        quad_transform = poses_state[self._quad_idx].GetAsMatrix34()
        load_transform = poses_state[self._load_idx].GetAsMatrix34()
        quad_velocity = velocities_state[self._quad_idx].get_coeffs()
        load_velocity = velocities_state[self._load_idx].get_coeffs()
        quad_acceleration = self._quad_accelerometer.measurement
        load_acceleration = self._load_accelerometer.measurement

        # print("quad_velocity: ", quad_velocity)
        # print("load_velocity: ", load_velocity)
        # print("quad_acceleration: ", quad_acceleration)
        # print("load_acceleration: ", load_acceleration)

        r = R.from_matrix(quad_transform[:3, :3])
        sci_quat = r.as_quat()
        quad_quat = np.array([sci_quat[3], sci_quat[0], sci_quat[1], sci_quat[2]])
        r = R.from_matrix(load_transform[:3, :3])
        sci_quat = r.as_quat()
        load_quat = np.array([sci_quat[3], sci_quat[0], sci_quat[1], sci_quat[2]])
        
        quadrotor_state = np.hstack((quad_quat, quad_transform[:3, 3], quad_velocity, quad_acceleration))
        payload_state = np.hstack((load_quat, load_transform[:3, 3], load_velocity, load_acceleration))

        # print("quadrotor_state: ", quadrotor_state)
        # print("payload_state: ", payload_state)

        # update state msg
        list_state = quadrotor_state.tolist() + payload_state.tolist()
        self._state_msg.data = list_state

        # send command to next subsystem
        output.SetFromVector(self._command_msg.data)
        # self._state_pub.publish(self._state_msg)


class CustomQuadrotorController(LeafSystem):
    """
    This class defines a custom controller for a quadrotor.
    It takes as input the state [q;qd] of the quadrotor and outputs
    propeller forces to be applied on the system. 
    """
    def __init__(self, quad_body_idx, load_body_idx, accelerometers, num_propellers=4):
        LeafSystem.__init__(self)

        self._quad_idx = quad_body_idx
        self._load_idx = load_body_idx

        self._quad_accelerometer = accelerometers[0]
        self._load_accelerometer = accelerometers[1]

        # Set up input and output ports
        self.DeclareAbstractInputPort("robot_pose", AbstractValue.Make([RigidTransform_[float]()]))
        self.DeclareAbstractInputPort("robot_velocity", AbstractValue.Make([SpatialVelocity_[float]()]))
        self.DeclareVectorOutputPort("propeller_force", BasicVector(num_propellers), self.CalcOutput)

    def CalcOutput(self, context, output):
        """
        This method is called at each timestep and used to
        set the output of this system (propeller forces). 
        """
        poses_state = self.get_input_port(0).Eval(context)
        velocities_state = self.get_input_port(1).Eval(context)

        quad_transform = poses_state[self._quad_idx].GetAsMatrix34()
        load_transform = poses_state[self._load_idx].GetAsMatrix34()
        quad_velocity = velocities_state[self._quad_idx].get_coeffs()
        load_velocity = velocities_state[self._load_idx].get_coeffs()
        quad_acceleration = self._quad_accelerometer.measurement
        load_acceleration = self._load_accelerometer.measurement

        # print("quad_acceleration: ", quad_acceleration)
        # print("load_acceleration: ", load_acceleration)

        ##################################################################
        #            IMPLEMENT YOUR CONTROLLER HERE!                     #
        ##################################################################

        # For now we'll just write a simple PD controller to hover 
        # at a given height (z_des), without any horizontal stabilization.

        z_des = 1.0     # desired height
        zd_des = 0.0    # desired vertical velocity
        z = quad_transform[2, 3]        # actual height
        zd = quad_velocity[5]      # actual vertical velocity

        m = 0.775       # quadrotor mass
        g = 9.81        # acceleration due to gravity

        kp = 2          # proportional gain
        kd = 1          # derivative gain

        # PD control law for total force in the z direction,
        # including gravity compensation
        fz = m*g - kp*(z-z_des) - kd*(zd-zd_des)

        # Divide this force equally between the four propellors
        u = fz/4*np.ones(4)

        ##################################################################

        # The control input u must be a list or numpy array with 4 elements,
        # representing the force applied by each propellor.
        output.SetFromVector(u)
