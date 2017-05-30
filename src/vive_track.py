#!/usr/bin/env python3
import openvr_wrapper
import time
import sys
import json
from websocket import create_connection


def parseToString(str_pose):
    data = {
        "op": "publish",
        "topic": topic,
        "msg": {
            "data": str_pose
        }
    }
    return json.dumps(data)
    #return json.dumps(data, sort_keys=True, indent=2, separators=(',', ': '))

def parseToPose(str_pose):
    str_pose = str_pose.split(',')
    data = {
        "op": "publish",
        "topic": topic,
        "msg": {
            "position": {"x": (str_pose[0] or 0), "y": (str_pose[1] or 0), "z": (str_pose[2] or 0)},
            "orientation": {"x": (str_pose[3] or 0), "y": (str_pose[4] or 0), "z": (str_pose[5] or 0), "w": (str_pose[6]or 0)},
        }
    }
    return json.dumps(data, sort_keys=True, indent=2, separators=(',', ': '))


if __name__ == '__main__':
    #initial vive
    vive = openvr_wrapper.OpenvrWrapper()
    vive.print_discovered_objects()
    if len(vive.object_names['Controller']) == 0:
        print("\rNo Controller detected, exit program...")
        sys.exit()

    ws = create_connection("ws://echo.websocket.org")
    if not ws.connected:
        print("\rWebsocket not connected, exit program...")
        sys.exit()

    interval = 1/120
    topic = "/vive/Pose"
    try:
        while True:
            start = time.time()
            pose = ""
            for each in vive.devices["controller_1"].get_pose_quaternion():
                pose += "%.6f" % each
                pose += ","
            pose = pose[:-1]
            #print("\r" + pose, end="")
            print(parseToPose(pose))
            ws.send(parseToPose(pose))
            sleep_time = interval-(time.time()-start)
            if sleep_time>0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        pass

    ws.close()