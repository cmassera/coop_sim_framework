#!/usr/bin/env python
import sys
import rospy
import random
from vehicle.msg import vehicle_data


transmission_delay = 0.0  # Transmission delay in seconds
failure_rate = 0.0  # Transmission failure rate in percentage
channel_number = 0  # Channel number for independent broadcast
publisher = None


# Callback to add package losses and transmission
#   delays in transmitted messages.
def vehicle_data_in_callback(msg):
    if random.random() >= failure_rate:
        rospy.Timer(rospy.Duration(transmission_delay),
                    lambda _: vehicle_data_out_callback(msg),
                    True)


# Callback to send packets after transmission
def vehicle_data_out_callback(msg):
    publisher.publish(msg)


# Main function
def main():
    global transmission_delay, failure_rate, publisher, channel_number

    if len(sys.argv) > 1:
        channel_number = int(sys.argv[1])
    else:
        print '\nUsage: python ' + sys.argv[0] + ' <channel_number>\n'
        sys.exit(1)

    # Initialize ROS node
    rospy.init_node('dsrc_' + str(channel_number), anonymous=True)

    # Load parameters if available
    transmission_delay = rospy.get_param('~config/transmission_delay',
                                         transmission_delay)
    failure_rate = rospy.get_param('~config/failure_rate', failure_rate)

    # Subscribe input channel
    rospy.Subscriber('~channel_out', vehicle_data, vehicle_data_in_callback)

    # Publish output channel
    publisher = rospy.Publisher('~channel_in', vehicle_data, queue_size=1)

    # Start execution
    rospy.spin()

# Initialize dsrc_channel
if __name__ == '__main__':
    main()
