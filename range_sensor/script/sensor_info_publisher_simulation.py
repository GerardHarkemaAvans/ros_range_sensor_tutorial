#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Node to publish to a string topic.

import rospy
from sensor_msgs.msg import Range
from sim_sensor_data import distSensorData as getSensorData


MAX_RANGE = 1.00
MIN_RANGE = 0.10

def sensorInfoPublisher():
    si_publisher = rospy.Publisher('sensor_info', Range, queue_size = 10)
    rospy.init_node('sensor_info_publisher', anonymous=False)
    rate = rospy.Rate(10)

    # Create a new SensorInformation object and fill in its contents.
    sensor_info = Range()

    # Fill in the header information.
    sensor_info.header.stamp = rospy.Time.now()
    sensor_info.header.frame_id = 'distance_sensor_frame'

    # Fill in the sensor data information.
    sensor_info.radiation_type = sensor_info.ULTRASOUND
    sensor_info.field_of_view = 0.5 # Field of view of the sensor in rad.
    sensor_info.min_range = MIN_RANGE # Minimum distance range of the sensor in m.
    sensor_info.max_range = MAX_RANGE # Maximum distance range of the sensor in m.


    while not rospy.is_shutdown():
        # Read the sensor data from a simulated sensor.
        sensor_info.range = getSensorData(sensor_info.radiation_type,
            sensor_info.min_range, sensor_info.max_range)

        # Publish the sensor information on the /sensor_info topic.
        si_publisher.publish(sensor_info)
        # Print log message if all went well.
        rospy.loginfo("All went well. Publishing topic ")
        rate.sleep()

if __name__ == '__main__':
    try:
        sensorInfoPublisher()
    except rospy.ROSInterruptException:
        pass
