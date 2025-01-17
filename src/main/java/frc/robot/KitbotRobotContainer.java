// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.HelloWorldAuto;
//import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.kitbot.KitbotRoller;

public class KitbotRobotContainer {
    private static final double JOYSTICK_DEADBAND = 0.05;

    // Need to know where the robot starts on the field
    // For this simulation, just make it fixed
    // Location: x = 2 meters, y = 1/2 of field, angle is forward
    private static final Pose2d START_LOCATION = new Pose2d(7.9, 7.1, Rotation2d.fromDegrees(0));

    private final CommandXboxController m_driverController = new CommandXboxController(0);

    private final AprilTagVision m_aprilTagVision = new AprilTagVision();
    private final DriveTrain m_driveTrain = new DriveTrain("swerve/kitbot", m_aprilTagVision);
    private final KitbotRoller m_kitbotRoller = new KitbotRoller();

    public KitbotRobotContainer() {
        configureBindings();

        m_driveTrain.setDefaultCommand(getDriveCommand());
    }
    
    private void configureBindings() {
        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        m_driverController.start().onTrue(new InstantCommand(m_driveTrain::lock, m_driveTrain));
        m_driverController.back().onTrue(new InstantCommand(m_driveTrain::zeroHeading, m_driveTrain));

        m_driverController.rightTrigger().whileTrue(new StartEndCommand(m_kitbotRoller::runRollerOut, m_kitbotRoller::stop, m_kitbotRoller));
        m_driverController.leftTrigger().whileTrue(new StartEndCommand(m_kitbotRoller::runRollerBack, m_kitbotRoller::stop, m_kitbotRoller));
    }
    
    public Command getAutonomousCommand() {
        return new HelloWorldAuto(this.getDriveTrain());
        // return new PathPlannerAuto("First Auto");
    }

    public Pose2d getInitialPose() {
        return FieldConstants.flipPose(START_LOCATION);
    }

    public Command getDriveCommand() {
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        // note: "rightBumper()"" is a Trigger which is a BooleanSupplier
        return m_driveTrain.driveCommand( 
                () -> -conditionAxis(m_driverController.getLeftY()),
                () -> -conditionAxis(m_driverController.getLeftX()),
                () -> -conditionAxis(m_driverController.getRightX()),
                // if you have a Logitech controller:
                // () -> -conditionAxis(m_driverController.getRawAxis(2)),
                m_driverController.rightBumper());
    }

    private double conditionAxis(double value) {
        value = MathUtil.applyDeadband(value, JOYSTICK_DEADBAND);
        // Square the axis, retaining the sign
        return Math.abs(value) * value;
    }

    public DriveTrain getDriveTrain() {
        return m_driveTrain;
    }
}
