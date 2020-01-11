#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from virtual_tracks.cfg import track_paramsConfig

def callback(config, level):
    return config

if __name__ == "__main__":
    rospy.init_node("track_server", anonymous = False)

    srv = Server(track_paramsConfig, callback)
    rospy.spin()
