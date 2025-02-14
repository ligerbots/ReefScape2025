// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class DriverRumble extends SubsystemBase {
  /** Creates a new DriverRumble. */

  private final double ANGLE_OFF_TOLERANCE_RAD = Math.toRadians(1); //TODO: Tune this value
  private final double METER_TO_TRIGGER;
  
  private final XboxController m_xbox;
  private final Supplier<Pose2d> m_positionToAlignSupplier;
  private final Supplier<Pose2d> m_robotPositionSupplier;

  public DriverRumble(Supplier<Pose2d> positionSupplier, Supplier<Pose2d> positionToAlignSupplier, XboxController xbox, double meterToTrigger) {
    m_positionToAlignSupplier = positionToAlignSupplier;
    m_xbox = xbox;
    m_robotPositionSupplier = positionSupplier;
    METER_TO_TRIGGER = meterToTrigger;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    final double signedRummbleValue = getRumble(getAngleOffFromPoint(m_positionToAlignSupplier.get(), m_robotPositionSupplier.get()));
    System.out.println("Rumble Value: " + signedRummbleValue);
    if (isWithinFeetToTrigger(m_positionToAlignSupplier.get(), m_robotPositionSupplier.get())) {
      System.out.println("Going to rumble as in range");
      if (signedRummbleValue > 0) {
        m_xbox.setRumble(RumbleType.kLeftRumble, 1);
        System.out.println("Rumble Left");
      } else if (signedRummbleValue < 0) {
        System.out.println("Rumble Right");
        m_xbox.setRumble(RumbleType.kRightRumble, 1);
      } else {
        m_xbox.setRumble(RumbleType.kBothRumble, 0);
      }
    } else {
      m_xbox.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  boolean isWithinFeetToTrigger(Pose2d positionToAlignTo, Pose2d currentPose) {
    Pose2d relitivePose = positionToAlignTo.relativeTo(currentPose);
    double distance = (Math.sqrt(Math.pow(relitivePose.getX(),2)+Math.pow(relitivePose.getY(),2)));
    // System.out.println("Distance: " + distance);
    // System.out.println("Pose to go to: " + positionToAlignTo);
    // System.out.println("Current Pose: " + currentPose);
    return distance < METER_TO_TRIGGER;
  }

  double getAngleOffFromPoint(Pose2d targetPoint, Pose2d currentPose) {
    final Pose2d relitivePose = targetPoint.relativeTo(currentPose); 
    final double angleFromPoint = Math.atan2(relitivePose.getY(), relitivePose.getX()); 

    //Normilise angle to be relitve to robot heading turning it into a 0-2PI range with the robot heading being 0
    final double normilisedAngle = (angleFromPoint - currentPose.getRotation().getRadians()) % (2*Math.PI);

    return normilisedAngle;
  }

  double getRumble(double angleOff) {
    //Check if within tolorence
    if (angleOff < ANGLE_OFF_TOLERANCE_RAD || angleOff > (Math.PI*2) - ANGLE_OFF_TOLERANCE_RAD) {
      return 0;
    }
    //Calculate rumble strength bewteen -1 & 1. The sign represents the side of the robot the target is on.
    final double rumbleStrength = angleOff < Math.PI ? angleOff / Math.PI : (angleOff - Math.PI) / -Math.PI;


    //NOTE: Rumble strength should be Positive if the target is on the left side of the robot and Negitive if on the right
    return rumbleStrength;
  }
}
