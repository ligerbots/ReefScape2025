// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
    private final EndEffectorPivot m_pivot = new EndEffectorPivot(() -> m_elevator.getHeight());
    private final Climber m_climber = new Climber();

    private final DriverRumble m_driverRumble = new DriverRumble(
        m_driverController.getHID(), () -> m_driveTrain.getPose(), 
        () -> m_coralEffector.hasCoral(), () -> m_algaeEffector.hasAlgae());
    
    private boolean m_coralMode = true;
    
    private final SendableChooser<String> m_chosenFieldSide = new SendableChooser<>();    
    private final SendableChooser<Pose2d> m_chosenSourcePickup = new SendableChooser<>();
    private final SendableChooser<Pose2d[]> m_chosenReefPoints = new SendableChooser<>();
    
    public CompRobotContainer() {
        m_elevator.setPivotCheckSupplier(() -> m_pivot.isOutsideLowRange());
        
        configureBindings();
        configureAutos();
        
        m_driveTrain.setDefaultCommand(getDriveCommand());
    }
    
    private void configureBindings() {
        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }
        
        //these are reserved for climbing 
        // m_driverController.start().onTrue(new InstantCommand(m_driveTrain::lock, m_driveTrain));
        // m_driverController.back().onTrue(new InstantCommand(m_driveTrain::zeroHeading, m_driveTrain));
        
        
        m_driverController.leftTrigger().whileTrue(
                new ConditionalCommand(
                        new StartEndCommand(m_coralEffector::runIntake, m_coralEffector::stop, m_coralEffector),
                        new StartEndCommand(m_algaeEffector::runIntake, m_algaeEffector::stop, m_algaeEffector),
                        () -> m_coralMode)
        );
        
        m_driverController.rightTrigger().whileTrue(
                new ConditionalCommand(
                        new StartEndCommand(m_coralEffector::runOuttake, m_coralEffector::stop, m_coralEffector),
                        new StartEndCommand(m_algaeEffector::scoreBarge, m_algaeEffector::stop, m_algaeEffector),
                        () -> m_coralMode)
        );
        
        m_driverController.rightBumper().onTrue(new MoveEndEffector(Constants.Position.STOW, m_elevator, m_pivot).andThen().finallyDo(() -> m_coralMode = true));
        
        m_driverController.a().onTrue(new MoveEndEffector(Constants.Position.L2_ALGAE, m_elevator, m_pivot).finallyDo(() -> m_coralMode = false));
        m_driverController.x().onTrue(new MoveEndEffector(Constants.Position.L3_ALGAE, m_elevator, m_pivot).finallyDo(() -> m_coralMode = false));
        m_driverController.y().onTrue(new MoveEndEffector(Constants.Position.BARGE, m_elevator, m_pivot).finallyDo(() -> m_coralMode = false));
        m_driverController.b().onTrue(new MoveEndEffector(Constants.Position.PROCESSOR, m_elevator, m_pivot).finallyDo(() -> m_coralMode = false));

        // m_driverController.y().onTrue(new DeferredCommand(new ReefTractorBeam(m_driveTrain), Set.of(m_driveTrain)));
        
        POVButton dpadLeft = new POVButton(m_driverController.getHID(), 270);
        dpadLeft.onTrue(new MoveEndEffector(Constants.Position.L4, m_elevator, m_pivot).finallyDo(() -> m_coralMode = true));
        
        POVButton dpadRight = new POVButton(m_driverController.getHID(), 90);
        dpadRight.onTrue(new MoveEndEffector(Constants.Position.BACK_INTAKE, m_elevator, m_pivot).finallyDo(() -> m_coralMode = true));
        
        POVButton dpadDown = new POVButton(m_driverController.getHID(), 0);
        dpadDown.onTrue(new MoveEndEffector(Constants.Position.L3, m_elevator, m_pivot).finallyDo(() -> m_coralMode = true));
        
        POVButton dpadUp = new POVButton(m_driverController.getHID(), 180);
        dpadUp.onTrue(new MoveEndEffector(Constants.Position.L2, m_elevator, m_pivot).finallyDo(() -> m_coralMode = true));
        
        // m_driverController.a().onTrue(new InstantCommand(() -> m_elevator.setHeight(Units.inchesToMeters(SmartDashboard.getNumber("elevator/testGoal", 0)))));
        // m_driverController.b().onTrue(new InstantCommand(() -> m_pivot.setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("pivot/testAngle", 0.0)))));
        
        m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_coralMode = !m_coralMode));
        
        // m_driverController.a().whileTrue(new StartEndCommand(() -> m_pivot.run(0.1), () -> m_pivot.run(0), m_pivot));
        // m_driverController.b().whileTrue(new StartEndCommand(() -> m_pivot.run(-0.1), () -> m_pivot.run(0), m_pivot));
        
        m_driverController.start().whileTrue(new StartEndCommand(() -> m_climber.run(0.4), m_climber::hold, m_climber));
        m_driverController.back().whileTrue(new StartEndCommand(() -> m_climber.run(-0.4), m_climber::hold, m_climber));
        m_driverController.back().onTrue(new MoveEndEffector(Constants.Position.CLIMB, m_elevator, m_pivot, 0));
    }
    
    private void configureAutos() {
        // NamedCommands.registerCommand("raiseElevatorBeforeReef", 
            
        // new MoveEndEffector(Constants.Position.L4, m_elevator, m_pivot, CompBotGenericAutoBase.RAISE_ELEVATOR_WAIT_TIME));
        
        Pose2d[] reefPoints = {FieldConstants.REEF_I, FieldConstants.REEF_J, FieldConstants.REEF_J};

        m_chosenReefPoints.setDefaultOption("IJJ  (aka FEE)", reefPoints);

        Pose2d[] reefPoints2 = {FieldConstants.REEF_J, FieldConstants.REEF_K, FieldConstants.REEF_L};
        m_chosenReefPoints.addOption("JKLA  (aka EDCB)", reefPoints2);
 
        Pose2d[] reefPoints3 = { FieldConstants.REEF_H };
        m_chosenReefPoints.addOption("H only  (aka I only) Center auto", reefPoints3);

        m_chosenFieldSide.setDefaultOption("Processor Side", "Processor Side");
        m_chosenFieldSide.addOption("Barge Side", "Barge Side");

        m_chosenSourcePickup.setDefaultOption("Center", FieldConstants.SOURCE_2_CENTER);
        m_chosenSourcePickup.addOption("Inside", FieldConstants.SOURCE_2_IN);
        m_chosenSourcePickup.addOption("Outside", FieldConstants.SOURCE_2_OUT);

        SmartDashboard.putData("Field Side", m_chosenFieldSide);
        SmartDashboard.putData("Source Pickup slot", m_chosenSourcePickup);
        SmartDashboard.putData("Reef Points", m_chosenReefPoints);
    }
    
    public Command getAutonomousCommand() {
        return new CompBotGenericAutoBase(FieldConstants.ROBOT_START_3, m_chosenSourcePickup.getSelected(), m_chosenReefPoints.getSelected(), 
        m_driveTrain, m_elevator, m_coralEffector, m_pivot, m_chosenFieldSide.getSelected().equals("Processor Side"));
    }
    

    public Pose2d getInitialPose() {
        return ((AutoCommandInterface) getAutonomousCommand()).getInitialPose();
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
        
    @Override
    public CoralEffector getCoralEffector() {
        return m_coralEffector;
    }
}
