// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriverAlignmentFeedback extends Command {
  /** Creates a new DriverAlignmentFeedback. */
  private final double ANGLE_OFF_TOLERANCE_RAD = Math.toRadians(1); //TODO: Tune this value
  private final double FEET_TO_TRIGGER;
  
  private final XboxController m_xbox;
  private final Supplier<Pose2d> m_positionToAlignSupplier;
  private final Supplier<Pose2d> m_robotPositionSupplier;

  public DriverAlignmentFeedback(Supplier<Pose2d> positionSupplier, Supplier<Pose2d> positionToAlignSupplier, XboxController xbox, double feetToTrigger) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_positionToAlignSupplier = positionToAlignSupplier;
    m_xbox = xbox;
    m_robotPositionSupplier = positionSupplier;
    FEET_TO_TRIGGER = feetToTrigger;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    final double signedRummbleValue = getRumble(getAngleOffFromPoint(m_positionToAlignSupplier.get(), m_robotPositionSupplier.get()));
    if (isWithinFeetToTrigger(m_positionToAlignSupplier.get(), m_robotPositionSupplier.get())) {
      if (signedRummbleValue > 0) {
        m_xbox.setRumble(RumbleType.kLeftRumble, signedRummbleValue);
      } else if (signedRummbleValue < 0) {
        m_xbox.setRumble(RumbleType.kRightRumble, -signedRummbleValue);
      } else {
        m_xbox.setRumble(RumbleType.kBothRumble, 0);
      }
    } else {
      m_xbox.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  boolean isWithinFeetToTrigger(Pose2d positionToAlignTo, Pose2d currentPose) {
    Pose2d relitivePose = positionToAlignTo.relativeTo(currentPose);
    return (Math.sqrt(Math.pow(relitivePose.getX(),2)+Math.pow(relitivePose.getY(),2))) < FEET_TO_TRIGGER;
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
