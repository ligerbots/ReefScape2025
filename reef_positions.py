#!/usr/bin/python3

import math

def meters(inches):
    return inches * 0.0254

center = (176.75, 158.3)
# offset: (half-width of Reef) + (1/2 robot perimeter) + (bumpers)
robot_offset = 28.0 / 2.0 + 3.5
center_offset = 32.75 + robot_offset
reef_offset = 13.0 / 2

# have the Algae position back 4 inches from touching the Reef,
#   so we can move the EE first
center_algae_offset = 1.0

letter = 'A'
algae = []

print(f"    // Reef pole robot positions")
print()

for i in range(-3, 3):
    angle = i * 60.0
    robot_angle = (angle + 180.0) % 360.0
    if robot_angle > 180:
        robot_angle -= 360

    rad = math.radians(angle)
    point1 = (center[0] + math.cos(rad)*center_offset, center[1] + math.sin(rad)*center_offset)

    letter2 = chr(ord(letter) + 1)
    algae_pos = (meters(point1[0] + math.cos(rad)*center_algae_offset),
                 meters(point1[1] + math.sin(rad)*center_algae_offset))
    algae.append(f"    public static final Pose2d REEF_ALGAE_{letter}{letter2} = new Pose2d({algae_pos[0]:0.3f}, {algae_pos[1]:0.3f}, Rotation2d.fromDegrees({robot_angle}));")

    for j in (-1, 1):
        rad2 = math.radians(angle + j * 90.0)
        point2 = (meters(point1[0] + math.cos(rad2)*reef_offset),
                  meters(point1[1] + math.sin(rad2)*reef_offset))
        print(f"    public static final Pose2d REEF_{letter} = new Pose2d({point2[0]:0.3f}, {point2[1]:0.3f}, Rotation2d.fromDegrees({robot_angle}));")

        letter = chr(ord(letter) + 1)

print()
print(f"    // Algae robot positions - these positions are {center_algae_offset} inches short of the wall")
print()
for m in algae:
    print(m)

# TODO: add the CoralStation positions
station_tag_angle = 54.0
station_tag_x = 33.51
station_tag_y = 25.80
slot_distance = 8.0
# push a bit further into the wall
push_into_wall = 4.0

# ACTUAL orig path point Source2Center
# x:1.166 y:7.179 angle:141.66
# BEFORE, 2 inches into push_into_wall
# public static final Pose2d SOURCE_2_SLOT5 = new Pose2d(1.083, 7.068, Rotation2d.fromDegrees(-54.0));
# public static final Pose2d SOURCE_2_SLOT6 = new Pose2d(1.247, 7.188, Rotation2d.fromDegrees(-54.0));

# AFTER, 4 inches into push_into_wall
#  public static final Pose2d SOURCE_2_SLOT5 = new Pose2d(1.053, 7.109, Rotation2d.fromDegrees(-54.0));
#     public static final Pose2d SOURCE_2_SLOT6 = new Pose2d(1.217, 7.229, Rotation2d.fromDegrees(-54.0));

field_width_meters = 8.042  # Andymark field
#field_width_meters = 8.052  # Welded

# center slot is directly under the tag
print()
print(f"    // Coral Slot robot locations - these positions push {push_into_wall} inches into the wall")
print()

robot_angle = math.radians(station_tag_angle)
source2 = []

for slotIndex in range(-4, 5):
    rad = math.radians(station_tag_angle + 90.0)

    slot = (station_tag_x - slotIndex * slot_distance * math.cos(rad),
            station_tag_y - slotIndex * slot_distance * math.sin(rad))
    robot = (meters(slot[0] + (robot_offset - push_into_wall) * math.cos(robot_angle)),
             meters(slot[1] + (robot_offset - push_into_wall) * math.sin(robot_angle)))

    print(f"    public static final Pose2d SOURCE_1_SLOT{slotIndex+5} = new Pose2d({robot[0]:0.3f}, {robot[1]:0.3f}, Rotation2d.fromDegrees({station_tag_angle}));")

    source2.append(f"    public static final Pose2d SOURCE_2_SLOT{slotIndex+5} = new Pose2d({robot[0]:0.3f}, {(field_width_meters-robot[1]):0.3f}, Rotation2d.fromDegrees({-station_tag_angle}));")

print()
for msg in source2:
    print(msg)
