#!/usr/bin/env python3
import openvr_wrapper
import time
import sys
import json
from websocket import create_connection


def parseString(topic,data):
    data = {
        "op": "publish",
        "topic": topic,
        "msg": {
            "data": data
        }
    }

    R = json.dumps(data, sort_keys=True, indent=2, separators=(',', ': '))

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
    try:
        while True:
            start = time.time()
            pose = ""
            for each in vive.devices["controller_1"].get_pose_quaternion():
                pose += "%.6f" % each
                pose += ","
            pose = pose[:-1]
            print("\r" + pose, end="")
            ws.send(parseString("/vive/poseString",pose))
            sleep_time = interval-(time.time()-start)
            if sleep_time>0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        pass

    ws.close()