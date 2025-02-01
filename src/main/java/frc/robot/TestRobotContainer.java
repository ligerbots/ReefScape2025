// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class TestRobotContainer extends RobotContainer {
    private static final double JOYSTICK_DEADBAND = 0.05;

    private final CommandXboxController m_driverController = new CommandXboxController(0);

    private final AprilTagVision m_aprilTagVision = new AprilTagVision();
    private final DriveTrain m_driveTrain = null; // new DriveTrain("swerve/kitbot", m_aprilTagVision);
    private final CoralEffector m_coralEffector = new CoralEffector(); 
    private final PowerDistribution m_pdp = new PowerDistribution();
    private final AlgaeEffector m_algaeEffector = new AlgaeEffector(m_pdp); 

    private AutoCommandInterface m_autoCommand;

    public TestRobotContainer() {
        configureBindings();
        configureAutos();

        // m_driveTrain.setDefaultCommand(getDriveCommand());
    }
            
    private void configureBindings() {
        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        // m_driverController.start().onTrue(new InstantCommand(m_driveTrain::lock, m_driveTrain));
        // m_driverController.back().onTrue(new InstantCommand(m_driveTrain::zeroHeading, m_driveTrain));

        m_driverController.rightTrigger().whileTrue(new StartEndCommand(m_coralEffector::runOuttake, m_coralEffector::stop, m_coralEffector));
        m_driverController.leftTrigger().whileTrue(new StartEndCommand(m_coralEffector::runIntake, m_coralEffector::stop, m_coralEffector));
        
        m_driverController.rightBumper().whileTrue(new StartEndCommand(m_algaeEffector::scoreBarge, m_algaeEffector::stop, m_algaeEffector));
        m_driverController.leftBumper().whileTrue(new StartEndCommand(m_algaeEffector::runIntake, m_algaeEffector::stop, m_algaeEffector));
    }
    
    private void configureAutos() {
        // TODO Auto-generated method stub
        m_autoCommand = null; //new HelloWorldAuto2(m_driveTrain);
    }

    public Command getAutonomousCommand() {
        return m_autoCommand;
    }

    public Pose2d getInitialPose() {
        if (m_autoCommand == null) return new Pose2d();
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
