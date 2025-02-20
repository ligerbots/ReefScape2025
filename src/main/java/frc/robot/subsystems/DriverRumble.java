// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;

public class DriverRumble extends SubsystemBase {
  /** Creates a new DriverRumble. */

  // Tolerance for lateral offset (meters) within which no rumble is applied.
  private final double LATERAL_OFFSET_TOLERANCE_METER = 0.05; // TODO: Tune this value (e.g., 5 cm or ~2 in). This is the tolerance in which the controller will not rumble as we are satisfactorly aligned. This idealy should be as large as possoble to give a concrete idea if we are ligned up
  // Maximum lateral offset (meters) that corresponds to full rumble intensity.
  private final double MAX_LATERAL_OFFSET_FOR_FULL_RUMBLE_METER = 0.5; // Tune as needed

  private final double METER_TO_TRIGGER;

  private final XboxController m_xbox;
  private final Supplier<Pose2d> m_positionToAlignSupplier;
  private final Supplier<Pose2d> m_robotPositionSupplier;

  public DriverRumble(Supplier<Pose2d> positionSupplier, Supplier<Pose2d> positionToAlignSupplier, XboxController xbox,
      double meterToTrigger) {
    m_positionToAlignSupplier = positionToAlignSupplier;
    m_xbox = xbox;
    m_robotPositionSupplier = positionSupplier;
    METER_TO_TRIGGER = meterToTrigger;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run.
    Pose2d targetPose = m_positionToAlignSupplier.get();
    Pose2d robotPose = m_robotPositionSupplier.get();

    // Calculate lateral offset in meters (positive means left, negative means right)
    final double lateralOffset = getLateralOffset(targetPose, robotPose);
    final double signedRumbleValue = getRumble(lateralOffset);

    System.out.println("Lateral Offset (m): " + lateralOffset);
    System.out.println("Rumble Value: " + signedRumbleValue);

    if (isWithinFeetToTrigger(targetPose, robotPose)) {
      System.out.println("Within range, activating rumble");
      if (signedRumbleValue > 0) {
        m_xbox.setRumble(RumbleType.kLeftRumble, 1);
        m_xbox.setRumble(RumbleType.kRightRumble, 0);
        System.out.println("Rumble Left");
      } else if (signedRumbleValue < 0) {
        m_xbox.setRumble(RumbleType.kRightRumble, 1);
        m_xbox.setRumble(RumbleType.kLeftRumble, 0);
        System.out.println("Rumble Right");
      } else {
        m_xbox.setRumble(RumbleType.kBothRumble, 0);
      }
    } else {
      m_xbox.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  boolean isWithinFeetToTrigger(Pose2d targetPose, Pose2d robotPose) {
    // Calculate the distance between the target and the robot.
    double dx = robotPose.getTranslation().getX() - targetPose.getTranslation().getX();
    double dy = robotPose.getTranslation().getY() - targetPose.getTranslation().getY();
    double distance = Math.sqrt(dx * dx + dy * dy);
    return distance < METER_TO_TRIGGER;
  }

  double getLateralOffset(Pose2d targetPose, Pose2d robotPose) {
    // Transform the robot's pose into the target's coordinate frame.
    // In this frame, the x-component is forward and the y-component is lateral
    // (positive is left).
    Pose2d relativePose = robotPose.relativeTo(targetPose);
    return relativePose.getY();
  }

  double getRumble(double lateralOffset) {
    // If the lateral offset is within tolerance, no rumble is needed.
    if (Math.abs(lateralOffset) < LATERAL_OFFSET_TOLERANCE_METER) {
      return 0;
    }
    // Scale the lateral offset to a value between -1 and 1.
    double strength = lateralOffset / MAX_LATERAL_OFFSET_FOR_FULL_RUMBLE_METER;
    // Clamp the strength between -1 and 1.
    if (strength > 1) {
      strength = 1;
    } else if (strength < -1) {
      strength = -1;
    }
    return strength;
  }

  public static Pose2d getClosestScoringLocation(Pose2d robotPose) {
    final List<Pose2d> REEF_POLES = List.of(FieldConstants.REEF_A, FieldConstants.REEF_B, FieldConstants.REEF_C,
        FieldConstants.REEF_D, FieldConstants.REEF_E, FieldConstants.REEF_F,
        FieldConstants.REEF_G, FieldConstants.REEF_H, FieldConstants.REEF_I,
        FieldConstants.REEF_J, FieldConstants.REEF_K, FieldConstants.REEF_L);

    Pose2d currentPose = FieldConstants.flipPose(robotPose);
    // System.out.println("Current Pose: " + currentPose);

    Pose2d nearestPole = FieldConstants.flipPose(currentPose.nearest(REEF_POLES));
    // System.out.println("Nearest Pole: " + nearestPole);
    return nearestPole;
  }
}
