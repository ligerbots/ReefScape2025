// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.subsystems.kitbot.*;

public class KitbotRobotContainer {
    private static final double JOYSTICK_DEADBAND = 0.05;

    private final CommandXboxController m_driverController = new CommandXboxController(0);

    private final AprilTagVision m_aprilTagVision = new AprilTagVision();
    private final DriveTrain m_driveTrain = new DriveTrain("swerve/kitbot", m_aprilTagVision);
    private final KitbotRoller m_kitbotRoller = new KitbotRoller();

    private final SendableChooser<Command> m_chosenAuto = new SendableChooser<>();
    private final SendableChooser<Pose2d> m_startLocation = new SendableChooser<>();
    private AutoCommandInterface m_autoCommand;

    public KitbotRobotContainer() {
        configureBindings();
        configureAutos();

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
    
    private void configureAutos() {
        // List of start locations
        // m_startLocation.setDefaultOption("Processor Side", FieldConstants.ROBOT_START_1);
        // m_startLocation.addOption("Center", FieldConstants.ROBOT_START_2);
        // m_startLocation.addOption("Away Side (Not Processor)", FieldConstants.ROBOT_START_3);
        // SmartDashboard.putData("Start Location", m_startLocation);

        m_chosenAuto.setDefaultOption("Processor Side Standard", new ExperimentalAutoBase(m_driveTrain, m_kitbotRoller, true));
        m_chosenAuto.addOption("Processor Side J-Path", new PrimaryAutoBase(m_driveTrain, m_kitbotRoller, true));

        m_chosenAuto.addOption("Away Side Standard", new PrimaryAutoBase(m_driveTrain, m_kitbotRoller, false));
        m_chosenAuto.addOption("Away Side J-Path", new PrimaryAutoBase(m_driveTrain, m_kitbotRoller, false));

        SmartDashboard.putData("Auto Choice", m_chosenAuto);

    }

    public Command getAutonomousCommand() {
        return m_chosenAuto.getSelected();
        // return m_autoCommand;
    }

    public Pose2d getInitialPose() {
        // return m_autoCommand.getInitialPose();
        AutoCommandInterface j = (AutoCommandInterface) m_chosenAuto.getSelected();
        return j.getInitialPose();
        // return FieldConstants.flipPose(m_startLocation.getSelected());
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
