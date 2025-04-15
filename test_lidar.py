import airsim
import sys
import math
import time
import argparse
import pprint
import numpy as np


class LidarTest:
    def __init__(self):
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)

    def execute(self):
        print("arming drone...")
        self.client.armDisarm(True)

        state = self.client.getMultirotorState()
        s = pprint.pformat(state)

        airsim.wait_key('Press anykey to take0ff')
        self.client.takeoffAsync().join()

        state = self.client.getMultirotorState()

        airsim.wait_key('Press any key to move vehicle to (-10, 10, -10) at 5 m/s')
        self.client.moveToPositionAsync(0, 0, -2, 1).join()

        self.client.hoverAsync().join()
        i = 1
        airsim.wait_key('Press any key to get Lidar readings')
        while True:
            lidardata = self.client.getLidarData()
            if len(lidardata.point_cloud) < 3:
                print("\tNo points received from Lidar data")
            else:
                points = self.parse_lidarData(lidardata)
                print("\tReading %d: time_stamp: %d number_of_points: %d" % (i, lidardata.time_stamp, len(points)))
                print("\t\tlidar position: %s" % (pprint.pformat(lidardata.pose.position)))
                print("\t\tlidar orientation: %s" % (pprint.pformat(lidardata.pose.orientation)))
            time.sleep(5)

    def parse_lidarData(self, data):

        # reshape array of floats to array of [X,Y,Z]
        points = np.array(data.point_cloud, dtype=np.dtype('f4'))
        points = np.reshape(points, (int(points.shape[0] / 3), 3))

        return points

    def stop(self):

        airsim.wait_key('Press any key to reset to original state')

        self.client.armDisarm(False)
        self.client.reset()

        self.client.enableApiControl(False)
        print("Done!\n")


if __name__ == "__main__":
    args = sys.argv
    args.pop(0)

    arg_parser = argparse.ArgumentParser("Lidar.py makes drone fly and gets Lidar data")
    arg_parser.add_argument('-save-to-disk', type=bool, help="save Lidar data to disk", default=False)

    args = arg_parser.parse_args(args)
    lidarTest = LidarTest()
    try:
        lidarTest.execute()
    finally:
        lidarTest.stop()

