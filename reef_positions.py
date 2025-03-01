#!/usr/bin/python3

import math

def meters(inches):
    return inches * 0.0254

center = (176.75, 158.3)
center_offset = 32.75 + 28/2.0 + 3.5
reef_offset = 13.0 / 2

letter = 'A'

for i in range(-3, 3):
    angle = i * 60.0
    robot_angle = (angle + 180.0) % 360.0
    if robot_angle > 180:
        robot_angle -= 360

    rad = math.radians(angle)
    point1 = (center[0] + math.cos(rad)*center_offset, center[1] + math.sin(rad)*center_offset)

    for j in (-1, 1):
        rad2 = math.radians(angle + j * 90.0)
        point2 = (meters(point1[0] + math.cos(rad2)*reef_offset),
                  meters(point1[1] + math.sin(rad2)*reef_offset))
        print(f"    public static final Pose2d REEF_{letter} = new Pose2d({point2[0]:0.3}, {point2[1]:0.3}, Rotation2d.fromDegrees({robot_angle}));")

        letter = chr(ord(letter) + 1)
