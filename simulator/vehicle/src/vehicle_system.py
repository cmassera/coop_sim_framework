#!/usr/bin/env python
import sys
import rospy
import numpy as np
from vehicle.msg import vehicle_data, control_input

# Vehicle handle class
from vehicle_dynamics import Vehicle

# Generic parameters
vehicle_id = 0  # Vehicle id
vehicle_handle = None  # Vehicle class instance
control_input_value = np.array([0.0, 0.0])  # Control input

# Dynamics parameters
dynamics_frequency = 500.0  # Integration frequency in Hz
dynamics_last_time = None  # Time buffer for numerical integration
start_time = None  # Start time for simulation
distance_event = False
speed_event = False

# Pose publisher parameters
pose_certain_publisher_1 = None
pose_certain_publisher_2 = None
pose_uncertain_publisher = None
dsrc_publisher = None
pose_certain_frequency = 100.0  # Pose publishing frequency in Hz
pose_uncertain_frequency = 20.0  # Pose publishing frequency in Hz

# Pose covariance
pose_covariance = np.matrix([[1, 0, 0,             0,     0, 0],
                             [0, 1, 0,             0,     0, 0],
                             [0, 0, 3*np.pi/180.0, 0,     0, 0],
                             [0, 0, 0,             0.1,   0, 0],
                             [0, 0, 0,             0,   0.1, 0],
                             [0, 0, 0,             0,     0, 3*np.pi/180.0]])
acceleration_covariance = np.matrix(np.zeros((6, 6)))
acceleration_covariance[3, 3] = 0.0
acceleration_covariance[4, 4] = 0.0

# DSRC publisher parameters
dsrc_publisher_frequency = 10.0  # DSRC publishing frequency in Hz


# Callback for control input reception
def control_input_callback(msg):
    control_input_value[0] = msg.acceleration
    control_input_value[1] = msg.steering


# Callback for dynamic integration
def dynamics_callback(time):
    global dynamics_last_time, start_time, distance_event, speed_event
    # Don't integrate on first iteration
    if dynamics_last_time is not None:
        vehicle_handle.step(control_input_value,
                            time.secs - dynamics_last_time.secs +
                            (time.nsecs - dynamics_last_time.nsecs) / 1.0e9)
    else:
        start_time = time
    dynamics_last_time = time

    if vehicle_id == 1 and distance_event and \
       time.secs - start_time.secs + (time.nsecs - start_time.nsecs) / 1.0e9 > 17:
        vehicle_handle.x[0, 0] -= 3
        distance_event = False

    if vehicle_id == 1 and speed_event and \
       time.secs - start_time.secs + (time.nsecs - start_time.nsecs) / 1.0e9 > 22:
        vehicle_handle.x[3, 0] -= 3
        speed_event = False


# Callback for ground truth publishing
def pose_certain_callback(time):
    msg = vehicle_data()
    x = vehicle_handle.x[:, 0]
    dx = vehicle_handle.df(control_input_value)
    msg.id = vehicle_id
    msg.timestamp = time
    msg.position.x = x[0]
    msg.position.y = x[1]
    msg.yaw = x[2]
    msg.velocity.x = x[3]
    msg.velocity.y = x[4]
    msg.yaw_rate = x[5]
    # TODO: Fix this using rotation
    msg.acceleration.x = dx[3]
    msg.acceleration.y = dx[4]
    msg.maximum_brake = vehicle_handle.param['maxb']
    pose_certain_publisher_1.publish(msg)
    pose_certain_publisher_2.publish(msg)


def pose_uncertain_callback(pub, time):
    print time
    msg = vehicle_data()
    x = np.random.multivariate_normal(np.array(vehicle_handle.x)[:, 0],
                                      pose_covariance)
    df = vehicle_handle.df(control_input_value)
    dx = np.random.multivariate_normal(df[:, 0], acceleration_covariance)
    msg.id = vehicle_id
    msg.timestamp = time
    msg.position.x = x[0]
    msg.position.y = x[1]
    msg.yaw = x[2]
    msg.velocity.x = x[3]
    msg.velocity.y = x[4]
    msg.yaw_rate = x[5]
    # TODO: Fix this using rotation
    msg.acceleration.x = dx[3]
    msg.acceleration.y = dx[4]
    msg.maximum_brake = vehicle_handle.param['maxb']
    pub.publish(msg)


# Main function
def main():
    global vehicle_handle, pose_certain_publisher_1, \
           pose_certain_publisher_2, pose_uncertain_publisher, vehicle_id
    if len(sys.argv) > 1:
        vehicle_id = int(sys.argv[1])
    else:
        print '\nUsage: python ' + sys.argv[0] + ' <vehicle_id>\n'
        sys.exit(1)

    # Initialize ROS node
    rospy.init_node('vehicle_' + str(vehicle_id))

    # Load configurations
    dsrc_channel = rospy.get_param('~config/dsrc_vehicle_channel')

    # Initialize vehicle handle
    vehicle_handle = Vehicle()

    # Spawn dynamics update callback
    rospy.Timer(rospy.Duration(1 / dynamics_frequency),
                lambda event: dynamics_callback(event.current_real))

    # Subscribe to control input
    rospy.Subscriber('~input', control_input, control_input_callback)

    # Initialize certain pose publisher
    pose_certain_publisher_1 = rospy.Publisher('/vehicles/data', vehicle_data,
                                               queue_size=1)
    pose_certain_publisher_2 = rospy.Publisher('~certain_data', vehicle_data,
                                               queue_size=1)
    rospy.Timer(rospy.Duration(1 / pose_certain_frequency),
                lambda event: pose_certain_callback(event.current_real))

    # Initialize uncertain pose publisher
    pose_uncertain_publisher = rospy.Publisher('~uncertain_data', vehicle_data,
                                               queue_size=1)
    rospy.Timer(rospy.Duration(1 / pose_uncertain_frequency),
                lambda event: pose_uncertain_callback(pose_uncertain_publisher,
                                                      event.current_real))

    # Initialize DSRC communication publisher
    dsrc_publisher = rospy.Publisher('/dsrc_' + str(dsrc_channel) +
                                     '/channel_out', vehicle_data,
                                     queue_size=1)
    rospy.Timer(rospy.Duration(1 / dsrc_publisher_frequency),
                lambda event: pose_uncertain_callback(dsrc_publisher,
                                                      event.current_real))

    # Start execution
    rospy.spin()

# Initialize vehicle node
if __name__ == '__main__':
    main()
