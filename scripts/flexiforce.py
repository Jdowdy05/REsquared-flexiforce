#!/usr/bin/env python3

import sys
import subprocess
import pathlib
import numpy as np
import threading

import rospy
from sensor_msgs.msg import Joy


# Serial devices to try
DEVICES = ['/dev/ttyACM0']
BAUD_RATE = 115200

# ROS rate in Hertz
ROS_rate = 100

# Value range of joints
joint_range = {
    0: (-0.5, 0.5),  # shoulder_yaw
    1: (-0.1, 0.5),  # shoulder_pitch
    3: (-0.5, 0.5),  # upper_arm_roll
    4: (-0.3, 0.5),  # elbow_pitch
}
joint_min = { k: v[0] for k, v in joint_range.items() }
joint_max = { k: v[1] for k, v in joint_range.items() }

joints = np.zeros((max(joint_range.keys()) + 1,))

#JD-will be the initial poition of the sensors
#felx_positions = [[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]

# Increment maximum this much (radians) per ROS poll
joint_increment = 0.002

class Flexiforce(threading.Thread):
    def __init__(self):
        self.find_device(DEVICES)
        self.configure_device(BAUD_RATE)
        self.maximum = 1023
        self.value_ = np.zeros((4,), dtype=int)
        super().__init__(daemon=True)

    def run(self):
        self.f = open(self.device, 'rt', encoding='ascii')
        while True:
            try:
                value_str = self.f.readline().rstrip('\n')
                for i, v in enumerate(value_str.split()):
                    self.value_[i] = int(v)
            except (ValueError, UnicodeDecodeError):
                continue
        self.f.close()

    @property
    def valueA(self):
        
        return self.value_[0]
    @property
    def valueB(self):
        return (-self.value_[1])

    @property
    def valueC(self):
        return self.value_[2]
    
    @property
    def valueD(self):
        return self.value_[3]

    def find_device(self, candidates):
        # Find device
        self.device = None
        for device in candidates:
            path = pathlib.Path(device)
            if path.exists() and path.is_char_device():
                self.device = device
                break
        if self.device is None:
            fatal("Cannot find device, tried:", *DEVICES, sep='\n')
        else:
            status("Found device on", device)

    def configure_device(self, baud):
        # Configure serial
        def run_stty(*args):
            result = subprocess.run(['stty', '-F', self.device] + list(args), check=True)
        status("Configuring", self.device, "with", baud, "baud")
        try:
            run_stty('raw')
            run_stty('-echo', '-echoe', '-echok')
            run_stty(str(baud))
        except subprocess.CalledProcessError:
            fatal("Error configuring", self.device)


def status(*args):
    print(*args, file=sys.stderr)

def fatal(*args):
    print(*args, file=sys.stderr)
    sys.exit(1)


def main():
    print(sys.version_info)
    flexi = Flexiforce()

    # Set up ROS node
    rospy.init_node('flexiforce')
    rate = rospy.Rate(ROS_rate)
    pub = rospy.Publisher('/joy', Joy, queue_size=1)
    joy = Joy()

    flexi.start()
    while not rospy.is_shutdown():
        
        sensorValues = [flexi.valueA, flexi.valueB, flexi.valueC, flexi.valueD]
        #print(sensorValues)
        joy.axes = sensorValues
        print(joy.axes)
        pub.publish(joy)
        rate.sleep()

if __name__ == '__main__':
    main()
#EOF
