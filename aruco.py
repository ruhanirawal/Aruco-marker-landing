#!/usr/bin/env python

from dronekit import connect, VehicleMode
from pymavlink import mavutil
import numpy as np
import cv2
from cv2 import aruco
import time
import argparse

def arm_and_takeoff(vehicle, aTargetAltitude):
    print("Basic pre-arm checks")
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialize...")
        time.sleep(1)

    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def land(vehicle):
    print("Now let's land")
    vehicle.mode = VehicleMode("LAND")

def main():
    cap = cv2.VideoCapture(0)

    parser = argparse.ArgumentParser()
    parser.add_argument('--connect', default='127.0.0.1:14550')
    args = parser.parse_args()

    print('Connecting to vehicle on: %s' % args.connect)
    vehicle = connect(args.connect, baud=921600, wait_ready=True)

    try:
        arm_and_takeoff(vehicle, 8)

        while True:
            ret, frame = cap.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
            parameters = aruco.DetectorParameters_create()
            corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
            frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)

            for rejectedPolygons in rejectedImgPoints:
                for points in rejectedPolygons:
                    cv2.line(frame_markers, tuple(points[0]), tuple(points[1]), [100, 0, 100])
                    cv2.line(frame_markers, tuple(points[2]), tuple(points[1]), [100, 0, 100])
                    cv2.line(frame_markers, tuple(points[2]), tuple(points[3]), [100, 0, 100])
                    cv2.line(frame_markers, tuple(points[0]), tuple(points[3]), [100, 0, 100])

            # Check for your specific ArUco marker ID (change 23 to your desired marker ID)
            if 23 in ids:
                land(vehicle)
                break

            cv2.imshow('frame', frame_markers)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cv2.destroyAllWindows()
        cap.release()
        vehicle.close()

if __name__ == "__main__":
    main()

