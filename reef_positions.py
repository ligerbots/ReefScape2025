#!/usr/bin/python3

import math

def meters(inches):
    return inches * 0.0254

center = (176.75, 158.3)
# offset: (half-width of Reef) + (1/2 robot perimeter) + (bumpers)
center_offset = 32.75 + 28/2.0 + 3.5
reef_offset = 13.0 / 2
center_algae_offset = 4.0

letter = 'A'

algae = []

for i in range(-3, 3):
    angle = i * 60.0
    robot_angle = (angle + 180.0) % 360.0
    if robot_angle > 180:
        robot_angle -= 360

    rad = math.radians(angle)
    point1 = (center[0] + math.cos(rad)*center_offset, center[1] + math.sin(rad)*center_offset)

    letter2 = chr(ord(letter) + 1)
    algae_pos = (meters(point1[0] + math.cos(rad)*center_algae_offset), meters(point1[1] + math.sin(rad)*center_algae_offset))
    algae.append(f"    public static final Pose2d REEF_ALGAE_{letter}{letter2} = new Pose2d({algae_pos[0]:0.4}, {algae_pos[1]:0.4}, Rotation2d.fromDegrees({robot_angle}));")

    for j in (-1, 1):
        rad2 = math.radians(angle + j * 90.0)
        point2 = (meters(point1[0] + math.cos(rad2)*reef_offset),
                  meters(point1[1] + math.sin(rad2)*reef_offset))
        print(f"    public static final Pose2d REEF_{letter} = new Pose2d({point2[0]:0.4}, {point2[1]:0.4}, Rotation2d.fromDegrees({robot_angle}));")

        letter = chr(ord(letter) + 1)

print()
for m in algae:
    print(m)

# TODO: add the CoralStation positions
