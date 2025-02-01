// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoCommandInterface;
import frc.robot.commands.HelloWorldAuto2;

import frc.robot.subsystems.*;
import frc.robot.subsystems.kitbot.KitbotRoller;


public class KitbotRobotContainer {
    private static final double JOYSTICK_DEADBAND = 0.05;

    private final CommandXboxController m_driverController = new CommandXboxController(0);

    private final AprilTagVision m_aprilTagVision = new AprilTagVision();
    private final DriveTrain m_driveTrain = new DriveTrain("swerve/kitbot", m_aprilTagVision);
    private final KitbotRoller m_kitbotRoller = new KitbotRoller();
    private final Leds m_leds = new Leds();
    private final CoralEndEffector m_coralEndEffector = new CoralEndEffector();
    private final AlgaeEndEffector m_algaeEndEffector = new AlgaeEndEffector();

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

        // m_driverController.rightTrigger().whileTrue(new StartEndCommand(m_kitbotRoller::runRollerOut, m_kitbotRoller::stop, m_kitbotRoller));
        // m_driverController.leftTrigger().whileTrue(new StartEndCommand(m_kitbotRoller::runRollerBack, m_kitbotRoller::stop, m_kitbotRoller));
        // m_driverController.rightTrigger().whileTrue(new StartEndCommand(m_endEffector::runOuttake, m_endEffector::stop, m_endEffector));
        // m_driverController.leftTrigger().whileTrue(new StartEndCommand(m_endEffector::runIntake, m_endEffector::stop, m_endEffector));
        
        m_driverController.a().onTrue(Commands.runOnce(() -> {m_leds.setSolidPattern(Color.kBlue);}));
        m_driverController.b().onTrue(Commands.runOnce(() -> {m_leds.setRainbowScrollingPattern();}));
        m_driverController.x().onTrue(Commands.runOnce(() -> {m_leds.setBlinkPattern(Color.kOrange);}));
        m_driverController.y().onTrue(Commands.runOnce(() -> {m_leds.setBarPattern(SmartDashboard.getNumber("robot/percentage", 0), Color.kGreen);}));
    }
    
    private void configureAutos() {
        // TODO Auto-generated method stub
        m_autoCommand = new HelloWorldAuto2(m_driveTrain, m_kitbotRoller);
    }

    public Command getAutonomousCommand() {
        return m_autoCommand;
    }

    public Pose2d getInitialPose() {
        return m_autoCommand.getInitialPose();
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
