import numpy as np
import rospy
from std_msgs.msg import Float32MultiArray


class PDController():
    def __init__(self, kp, kd, ki, z_des, dz_des, m=0.775, g=9.81):
        self._kp = kp
        self._kd = kd
        self._ki = ki
        self._z_des = z_des
        self._dz_des = dz_des
        self._m = m
        self._g = g

    def CalcOutput(self, state):
        # print(state)
        z = state[6]
        dz = state[12]
        fz = self._m * self._g - self._kp * (z - self._z_des) - self._kd * (dz - self._dz_des)
        u = fz / 4 * np.ones(4)
        return u.tolist()


class Client():
    def __init__(self):
        self._controller = PDController(2, 1, 0, 1, 0)

        self._state_msg = Float32MultiArray()
        self._state_msg.data = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                                1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self._command_msg = Float32MultiArray()
        # receive state from remote controller
        self._sub = rospy.Subscriber('/sim_robot_state', Float32MultiArray, self.SubsCallback)
        # publish command to remote controller
        self._pub = rospy.Publisher('/ctrl_command', Float32MultiArray, queue_size=1)
        # rospy.Timer(rospy.Duration(0.0002), self.PublishCallback)

    def SubsCallback(self, msg):
        self._state_msg.data = msg.data

    def PublishCallback(self, event):
        self._command_msg.data = self._controller.CalcOutput(self._state_msg.data)
        self._pub.publish(self._command_msg)


def main():
    client = Client()

    rospy.init_node('client', anonymous=True)
    rospy.Timer(rospy.Duration(0.0001), client.PublishCallback)
    rospy.spin()


if __name__ == '__main__':
    main()
