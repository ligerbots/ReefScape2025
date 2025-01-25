// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveTrain;

public abstract class RobotContainer  {
    public abstract Command getAutonomousCommand();
    public abstract Pose2d getInitialPose();

    public abstract DriveTrain getDriveTrain();
}
