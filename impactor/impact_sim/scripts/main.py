import os
import yaml
import threading
import numpy as np
import time
import rospy

from build_payload_diagram import build_payload_diagram


def spin_job():
    rospy.spin()


# Load config
config_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "config/config.yaml")
config = yaml.safe_load(open(config_path, 'r'))

step_dt = config['env']['step_dt']
duration = config['env']['duration']
sim_objects = build_payload_diagram(config)
simulator, diagram, systems = sim_objects

sim_context = simulator.get_mutable_context()
quad_accelerometer = systems["station"].subsystems["quad_accelerometer"]
load_accelerometer = systems["station"].subsystems["load_accelerometer"]
quad_acc_context = diagram.GetMutableSubsystemContext(quad_accelerometer, sim_context)
load_acc_context = diagram.GetMutableSubsystemContext(load_accelerometer, sim_context)

if config["env"]["use_remote_control"]:
    rospy.init_node("drake_sim", anonymous=True)
    rospy.Timer(rospy.Duration(step_dt), systems["station"].subsystems["controller"].PublishCallback)
    spin_thread = threading.Thread(target=spin_job)
    spin_thread.start()
    
    rate = rospy.Rate(1.0/step_dt)
    while not rospy.is_shutdown() or simulator.get_context().get_time() < duration:
        cur_t = simulator.get_context().get_time()
        simulator.AdvanceTo(cur_t + step_dt)
        rate.sleep()

    spin_thread.join()
else:
    while simulator.get_context().get_time() < duration:
        quad_acc_measured = quad_accelerometer.get_output_port().Eval(quad_acc_context)
        load_acc_measured = load_accelerometer.get_output_port().Eval(load_acc_context)
        
        cur_t = simulator.get_context().get_time()
        simulator.AdvanceTo(cur_t + step_dt)