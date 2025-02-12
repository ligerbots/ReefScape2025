// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;

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
  public void execute() {
    SmartDashboard.putString("PredictedScoreLocation", getLikelyScoringPosition(FieldConstants.flipPose(m_pose.get())));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  //Returns the most likely scoring position based on the current robot position & heading
  String getLikelyScoringPosition(Pose2d currentFlippedPose) {
    Translation2d quadrentMidpoint = getMidpointForCurrentQuadrent(currentFlippedPose);
    //FIXME: Current converting to pose2d is a hack
    final boolean isLeftOfMidpoint = isLeftOfPoint(new Pose2d(quadrentMidpoint.getX(), quadrentMidpoint.getY(), new Rotation2d()), currentFlippedPose);
    if (quadrentMidpoint == FieldConstants.REEF_TAG_GH) {
      if (isLeftOfMidpoint) {
        return "G";
        // return FieldConstants.REEF_G;
      } else {
        return "H";
        // return FieldConstants.REEF_H;
      }
    } else if (quadrentMidpoint == FieldConstants.REEF_TAG_IJ) {
      if (isLeftOfMidpoint) {
        return "I";
        // return FieldConstants.REEF_I;
      } else {
        return "J";
        // return FieldConstants.REEF_J;
      }
    } else if (quadrentMidpoint == FieldConstants.REEF_TAG_KL) {
      if (isLeftOfMidpoint) {
        return "K";
        // return FieldConstants.REEF_K;
      } else {
        return "L";
        // return FieldConstants.REEF_L;
      }
    } else if (quadrentMidpoint == FieldConstants.REEF_TAG_AB) {
      if (isLeftOfMidpoint) {
        return "A";
        // return FieldConstants.REEF_A;
      } else {
        return "B";
        // return FieldConstants.REEF_B;
      }
    } else if (quadrentMidpoint == FieldConstants.REEF_TAG_CD) {
      if (isLeftOfMidpoint) {
        return "C";
        // return FieldConstants.REEF_C;
      } else {
        return "D";
        // return FieldConstants.REEF_D;
      }
    } else if (quadrentMidpoint == FieldConstants.REEF_TAG_EF) {
      if (isLeftOfMidpoint) {
        return "E";
        // return FieldConstants.REEF_E;
      } else {
        return "F";
        // return FieldConstants.REEF_F;
      }
    } else {
      //Should never happen
      throw new IllegalArgumentException("Midpoint not in any pre-set quadrent");
    }
  }

  Translation2d getMidpointForCurrentQuadrent(Pose2d currentFlippedPose) {
    final Pose2d relitivePose = currentFlippedPose.relativeTo(REEF); //Relitive pose relitive to the center of the reef
    final double angleFromReef = Math.atan2(relitivePose.getY(), relitivePose.getX()); //Angle from center of reef

    if (angleFromReef > (11*Math.PI)/6 || angleFromReef < Math.PI/6) {
      return FieldConstants.REEF_TAG_GH;
    } else if (angleFromReef > (Math.PI)/6 || angleFromReef < Math.PI/2) {
      return FieldConstants.REEF_TAG_IJ;
    } else if (angleFromReef > (Math.PI)/2 || angleFromReef < (5*Math.PI)/6) {
      return FieldConstants.REEF_TAG_KL;
    } else if (angleFromReef > (5*Math.PI)/6 || angleFromReef < (7*Math.PI)/6) {
      return FieldConstants.REEF_TAG_AB;
    } else if (angleFromReef > (7*Math.PI)/6 || angleFromReef < (3*Math.PI)/2) {
      return FieldConstants.REEF_TAG_CD;
    } else if (angleFromReef > (3*Math.PI)/2 || angleFromReef < (11*Math.PI)/6) {
      return FieldConstants.REEF_TAG_EF;
    } else {
      //Should never happen
      throw new IllegalArgumentException("Angle not in any quadrent");
    }
  }

  boolean isLeftOfPoint(Pose2d targetPoint, Pose2d currentFlippedPose) {
    final Pose2d relitivePose = targetPoint.relativeTo(currentFlippedPose); 
    final double angleFromPoint = Math.atan2(relitivePose.getY(), relitivePose.getX()); 

    //Normilise angle to be relitve to robot heading turning it into a 0-2PI range with the robot heading being 0
    final double normilisedAngle = (angleFromPoint - currentFlippedPose.getRotation().getRadians()) % (2*Math.PI);


    if (normilisedAngle < Math.PI) {
      return true;
    } else {
      return false;
    }
  }
}
