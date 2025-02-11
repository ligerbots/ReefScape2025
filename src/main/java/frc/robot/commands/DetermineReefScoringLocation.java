// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DetermineReefScoringLocation extends Command {
  /** Creates a new DetermineReefQuadrent. */
  final Supplier<Pose2d> m_pose;
  final Pose2d REEF = new Pose2d(0,0, Rotation2d.fromDegrees(0)); //TODO: Replace with real pose

  public DetermineReefScoringLocation(Supplier<Pose2d> pose) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pose = pose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  int getQuadrent(Pose2d currentPose) {
    final Pose2d relitivePose = currentPose.relativeTo(REEF); //Relitive pose relitive to the center of the reef
    final double angleFromReef = Math.atan2(relitivePose.getY(), relitivePose.getX()); //Angle from center of reef

    if (angleFromReef > (11*Math.PI)/6 || angleFromReef < Math.PI/6) {
      return 0;
    } else if (angleFromReef > (Math.PI)/6 || angleFromReef < Math.PI/2) {
      return 1;
    } else if (angleFromReef > (Math.PI)/2 || angleFromReef < (5*Math.PI)/6) {
      return 2;
    } else if (angleFromReef > (5*Math.PI)/6 || angleFromReef < (7*Math.PI)/6) {
      return 3;
    } else if (angleFromReef > (7*Math.PI)/6 || angleFromReef < (3*Math.PI)/2) {
      return 4;
    } else if (angleFromReef > (3*Math.PI)/2 || angleFromReef < (11*Math.PI)/6) {
      return 5;
    } else {
      //TODO: If this is hit, something is wrong. Means that angle is perfectly on a boundry. Chances almost zero unless code is wrong
      return 6;
    }
  }

  boolean isLeftOfPoint(Pose2d targetPoint, Pose2d currentPose) {
    final Pose2d relitivePose = targetPoint.relativeTo(currentPose); 
    final double angleFromPoint = Math.atan2(relitivePose.getY(), relitivePose.getX()); 

    //Normilise angle to be relitve to robot heading turning it into a 0-2PI range with the robot heading being 0
    final double normilisedAngle = (angleFromPoint - currentPose.getRotation().getRadians()) % (2*Math.PI);

    
    if (normilisedAngle < Math.PI) {
      return true;
    } else {
      return false;
    }
  }
}
