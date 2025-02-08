// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class CompRobotContainer extends RobotContainer {
    private static final double JOYSTICK_DEADBAND = 0.05;

    private final CommandXboxController m_driverController = new CommandXboxController(0);

    private final AprilTagVision m_aprilTagVision = new AprilTagVision();
    private final DriveTrain m_driveTrain = new DriveTrain("swerve/comp", m_aprilTagVision);
    private final CoralEffector m_coralEffector = new CoralEffector();
    private final PowerDistribution m_pdh = new PowerDistribution();
    private final AlgaeEffector m_algaeEffector = new AlgaeEffector(m_pdh);
    private final Leds m_leds = new Leds();
    private final Elevator m_elevator = new Elevator();
    private final EndEffectorPivot m_pivot = new EndEffectorPivot();
    private final Climber m_climber = new Climber();

    private boolean m_coralMode = true;

    private AutoCommandInterface m_autoCommand;

    public CompRobotContainer() {
        configureBindings();
        configureAutos();

        m_driveTrain.setDefaultCommand(getDriveCommand());
    }

    private void configureBindings() {
        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }
        //these are reserved for climbing 
        m_driverController.start().onTrue(new InstantCommand(m_driveTrain::lock, m_driveTrain));
        m_driverController.back().onTrue(new InstantCommand(m_driveTrain::zeroHeading, m_driveTrain));

        
        m_driverController.leftTrigger().whileTrue(
            new ConditionalCommand(
                new StartEndCommand(m_coralEffector::runIntake, m_coralEffector::stop, m_coralEffector), 
                new StartEndCommand(m_algaeEffector::runIntake, m_algaeEffector::stop, m_algaeEffector), 
                ()->m_coralMode));

        m_driverController.rightTrigger().whileTrue(
            new ConditionalCommand(
                new StartEndCommand(m_coralEffector::runOuttake, m_coralEffector::stop, m_coralEffector), 
                new StartEndCommand(m_algaeEffector::scoreBarge, m_algaeEffector::stop, m_algaeEffector), 
                ()->m_coralMode));

        m_driverController.rightBumper().onTrue(new MoveEndEffector(Constants.Position.STOW, m_elevator, m_pivot).andThen().finallyDo(()->m_coralMode = true));

        m_driverController.a().onTrue(new MoveEndEffector(Constants.Position.L2_ALGAE, m_elevator, m_pivot).finallyDo(()->m_coralMode = false));
        m_driverController.b().onTrue(new MoveEndEffector(Constants.Position.L3_ALGAE, m_elevator, m_pivot).finallyDo(()->m_coralMode = false));
        m_driverController.x().onTrue(new MoveEndEffector(Constants.Position.BARGE, m_elevator, m_pivot).finallyDo(()->m_coralMode = false));

        POVButton dpadLeft = new POVButton(m_driverController.getHID(), 270);
        dpadLeft.onTrue(new MoveEndEffector(Constants.Position.L4, m_elevator, m_pivot).finallyDo(()->m_coralMode = true));
        
        POVButton dpadRight = new POVButton(m_driverController.getHID(), 90);
        dpadRight.onTrue(new MoveEndEffector(Constants.Position.BACK_INTAKE, m_elevator, m_pivot).finallyDo(()->m_coralMode = true));

        POVButton dpadDown = new POVButton(m_driverController.getHID(), 180);
        dpadDown.onTrue(new MoveEndEffector(Constants.Position.L3, m_elevator, m_pivot).finallyDo(()->m_coralMode = true));

        POVButton dpadUp = new POVButton(m_driverController.getHID(), 0);
        dpadUp.onTrue(new MoveEndEffector(Constants.Position.L2, m_elevator, m_pivot).andThen().finallyDo(()->m_coralMode = true));
    
        // m_driverController.a().onTrue(new InstantCommand(() -> m_elevator.setHeight(SmartDashboard.getNumber("elevator/testGoal", 0))));
        // m_driverController.b().onTrue(new InstantCommand(() -> m_pivot.setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("pivot/testAngle", 0.0)))));

        // m_driverController.x().whileTrue(new StartEndCommand(() -> m_climber.run(0.2), m_climber::hold, m_climber));
        // m_driverController.y().whileTrue(new StartEndCommand(() -> m_climber.run(-0.2), m_climber::hold, m_climber));
    }

    private void configureAutos() {
        // TODO Auto-generated method stub
        m_autoCommand = null; // new HelloWorldAuto2(m_driveTrain);
    }

    public Command getAutonomousCommand() {
        return m_autoCommand;
    }

    public Pose2d getInitialPose() {
        if (m_autoCommand == null)
            return new Pose2d(1, 1, Rotation2d.fromDegrees(0));
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
