// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Objects;
import java.util.Set;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class CompRobotContainer extends RobotContainer {
    private static final double JOYSTICK_DEADBAND = 0.05;
    
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandJoystick m_farm = new CommandJoystick(1);
    
    private final AprilTagVision m_aprilTagVision = new AprilTagVision();
    private final DriveTrain m_driveTrain = new DriveTrain("swerve/comp", m_aprilTagVision);
    // private final Leds m_leds = new Leds();
    // private final PowerDistribution m_pdh = new PowerDistribution();

    private final Elevator m_elevator = new Elevator();
    private final EndEffectorPivot m_pivot = new EndEffectorPivot(() -> m_elevator.getHeight());
    private final CoralEffector m_coralEffector = new CoralEffector(()-> m_elevator.getGoal());
    private final AlgaeEffector m_algaeEffector = new AlgaeEffector(() -> m_elevator.getHeight());

    private final Climber m_climber = new Climber();

    @SuppressWarnings("unused")
    private final DriverRumble m_driverRumble = new DriverRumble(
        m_driverController.getHID(), () -> m_driveTrain.getPose(), 
        () -> m_coralEffector.hasCoral(), () -> m_algaeEffector.hasAlgae(),
        () -> m_climber.isDeployed());
    
    private boolean m_coralMode = true;
    
    private final SendableChooser<String> m_chosenFieldSide = new SendableChooser<>(); 
    private final SendableChooser<Pose2d> m_chosenStartPoint = new SendableChooser<>();    
    // private final SendableChooser<Pose2d> m_chosenSourcePickup = new SendableChooser<>();
    private final SendableChooser<String> m_chosenAutoFlavor = new SendableChooser<>(); 
    private final SendableChooser<Pose2d[]> m_chosenReefPoints = new SendableChooser<>();

    private ReefscapeAbstractAuto m_autoCommand;
    private int m_autoSelectionCode = 0;
    
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
        
        m_driverController.leftTrigger().whileTrue(
                new ConditionalCommand(
                        new StartEndCommand(m_coralEffector::runIntake, m_coralEffector::stop, m_coralEffector),
                        new StartEndCommand(m_algaeEffector::runIntake, m_algaeEffector::stop, m_algaeEffector),
                        () -> m_coralMode)
        );

        m_driverController.rightTrigger().whileTrue(
                new ConditionalCommand(
                        new StartEndCommand(m_coralEffector::runOuttake, m_coralEffector::stop, m_coralEffector),
                        new StartEndCommand(m_algaeEffector::score, m_algaeEffector::stop, m_algaeEffector),
                        () -> m_coralMode)
        );

        Trigger coralRumble = new Trigger(() -> m_coralEffector.hasCoral());
        coralRumble.onTrue(new Rumble(m_driverController.getHID()).alongWith(new PrintCommand("RUMBLE!!!")));

        Trigger algaeRumble = new Trigger(() -> m_algaeEffector.hasAlgae());
        algaeRumble.onTrue(new Rumble(m_driverController.getHID()).alongWith(new PrintCommand("RUMBLE!!!")));
        
        m_driverController.rightBumper().onTrue(new DeferredCommand(new ReefTractorBeam(m_driveTrain, false, m_coralEffector::hasCoral), Set.of(m_driveTrain)));
        m_driverController.leftBumper().onTrue(new DeferredCommand(new ReefTractorBeam(m_driveTrain, true, m_coralEffector::hasCoral), Set.of(m_driveTrain)));

        // Algae Scoring
        m_driverController.a().onTrue(new MoveEndEffector(Constants.Position.L2_ALGAE, m_elevator, m_pivot).alongWith(new InstantCommand(() -> m_coralMode = false)));
        m_driverController.x().onTrue(new MoveEndEffector(Constants.Position.L3_ALGAE, m_elevator, m_pivot).alongWith(new InstantCommand(() -> m_coralMode = false)));
        m_driverController.y().onTrue(new MoveEndEffector(Constants.Position.BARGE, m_elevator, m_pivot).alongWith(new InstantCommand(() -> m_coralMode = false)));
        m_driverController.b().onTrue(new MoveEndEffector(Constants.Position.PROCESSOR, m_elevator, m_pivot).alongWith(new InstantCommand(() -> m_coralMode = false)));

        // Coral Scoring
        m_driverController.pov(270).onTrue(new MoveEndEffector(Constants.Position.L4, m_elevator, m_pivot).alongWith(new InstantCommand(() -> m_coralMode = true)));
        m_driverController.pov(90).onTrue(new MoveEndEffector(Constants.Position.BACK_INTAKE, m_elevator, m_pivot).alongWith(new InstantCommand(() -> m_coralMode = true)));
        m_driverController.pov(0).onTrue(new MoveEndEffector(Constants.Position.L3, m_elevator, m_pivot).alongWith(new InstantCommand(() -> m_coralMode = true)));
        m_driverController.pov(180).onTrue(new MoveEndEffector(Constants.Position.L2, m_elevator, m_pivot).alongWith(new InstantCommand(() -> m_coralMode = true)));
                
        // Climber
        m_driverController.start().onTrue(new InstantCommand(m_climber::climb));
        m_driverController.back().onTrue(new InstantCommand(m_climber::deploy));
        m_driverController.back().onTrue(new MoveEndEffector(Constants.Position.CLIMB, m_elevator, m_pivot, 0));

        m_farm.button(1).whileTrue(new StartEndCommand(() -> m_climber.run(Climber.MANUAL_SPEED), m_climber::hold, m_climber));

        m_farm.button(2).whileTrue(new StartEndCommand(() -> m_climber.run(-Climber.MANUAL_SPEED), m_climber::hold, m_climber));
        m_farm.button(2).onTrue(new MoveEndEffector(Constants.Position.CLIMB, m_elevator, m_pivot, 0));

        // Miscellaneous
        m_farm.button(6).onTrue(new InstantCommand(m_driveTrain::lock, m_driveTrain));
        
        m_farm.button(21).onTrue(new MoveEndEffector(Constants.Position.L1, m_elevator, m_pivot));
        // note: farm 7 is robot-centric
        m_farm.button(8).onTrue(new InstantCommand(m_driveTrain::zeroHeading, m_driveTrain));
        m_farm.button(11).onTrue(new InstantCommand(() -> m_coralMode = !m_coralMode));
        m_farm.button(12).whileTrue(new InstantCommand(m_elevator::zeroElevator));

        // Testing commands

        m_farm.button(5).onTrue(new InstantCommand(() -> m_elevator.setHeight(Units.inchesToMeters(SmartDashboard.getNumber("elevator/testGoal", 0)))));
        m_farm.button(10).onTrue(new InstantCommand(() -> m_pivot.setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("pivot/testAngle", 0.0)))));
    }
    
    private void configureAutos() {
        // NamedCommands.registerCommand("raiseElevatorBeforeReef", 
            
        // new MoveEndEffector(Constants.Position.L4, m_elevator, m_pivot, CompBotGenericAutoBase.RAISE_ELEVATOR_WAIT_TIME));
        
        // NamedCommands.registerCommand("raiseElevatorBeforeReef", Commands.print("Running raiseElevatorBeforeReef-- holy cow!")
        //                                 .andThen(new MoveEndEffector(Constants.Position.L4, m_elevator, m_pivot, ReefscapeAbstractAuto.RAISE_ELEVATOR_WAIT_TIME)));

        Pose2d[] reefPoints = {FieldConstants.REEF_I, FieldConstants.REEF_J, FieldConstants.REEF_K};

        m_chosenReefPoints.addOption("IJK  (aka FED)", reefPoints);

        Pose2d[] reefPoints2 = {FieldConstants.REEF_J, FieldConstants.REEF_K, FieldConstants.REEF_L, FieldConstants.REEF_A};
        m_chosenReefPoints.setDefaultOption("JKLA (aka EDCB)", reefPoints2);
 
        Pose2d[] reefPoints3 = { FieldConstants.REEF_H };
        m_chosenReefPoints.addOption("H only  (aka G only) Center auto", reefPoints3);

        Pose2d[] reefPoints4 = { FieldConstants.REEF_H, FieldConstants.REEF_ALGAE_GH };
        m_chosenReefPoints.addOption("H coral score, then grab GH Algae", reefPoints4);

        m_chosenFieldSide.setDefaultOption("Processor Side", "Processor Side");
        m_chosenFieldSide.addOption("Barge Side", "Barge Side");

        m_chosenStartPoint.setDefaultOption("3rd cage-- usual spot", FieldConstants.ROBOT_START_3);
        m_chosenStartPoint.addOption("Field Center", FieldConstants.ROBOT_START_2);

        m_chosenAutoFlavor.addOption("Granite State", "Granite State");
        m_chosenAutoFlavor.addOption("Experimental", "Experimental");
        m_chosenAutoFlavor.addOption("Algae", "Algae");
        m_chosenAutoFlavor.setDefaultOption("Refactor", "Refactor");


        // m_chosenSourcePickup.setDefaultOption("Center", FieldConstants.SOURCE_2_CENTER);
        // m_chosenSourcePickup.addOption("Inside", FieldConstants.SOURCE_2_IN);
        // m_chosenSourcePickup.addOption("Outside", FieldConstants.SOURCE_2_OUT);

        SmartDashboard.putData("Field Side", m_chosenFieldSide);
        SmartDashboard.putData("Auto Start Point", m_chosenStartPoint);
        // SmartDashboard.putData("Source Pickup slot", m_chosenSourcePickup);
        SmartDashboard.putData("Reef Points", m_chosenReefPoints);
        SmartDashboard.putData("Auto flavor", m_chosenAutoFlavor);
    }
    
    public Command getAutonomousCommand() {
        int currentAutoSelectionCode = Objects.hash(m_chosenStartPoint.getSelected(), m_chosenAutoFlavor.getSelected(),
            m_chosenReefPoints.getSelected(), m_chosenFieldSide.getSelected(), DriverStation.getAlliance());
        // Only call constructor if the auto selection inputs have changed
        if (m_autoSelectionCode != currentAutoSelectionCode) {
            String autoFlavor = m_chosenAutoFlavor.getSelected();
            if(autoFlavor.equals("Granite State")) {
                m_autoCommand = new CompBotGraniteStateAuto(m_chosenStartPoint.getSelected(), FieldConstants.SOURCE_2_CENTER, m_chosenReefPoints.getSelected(), 
                    m_driveTrain, m_elevator, m_coralEffector, m_pivot, m_chosenFieldSide.getSelected().equals("Processor Side"));
            } 

            if(autoFlavor.equals("Experimental")) { 
                m_autoCommand = new CompBotExperimentalAuto(m_chosenStartPoint.getSelected(), FieldConstants.SOURCE_2_CENTER, m_chosenReefPoints.getSelected(), 
                        m_driveTrain, m_elevator, m_coralEffector, m_pivot, m_chosenFieldSide.getSelected().equals("Processor Side"));
            }

            // if(autoFlavor.equals("Algae")) { 
            //     m_autoCommand = new CompBotAlgaeAuto(m_chosenStartPoint.getSelected(), m_chosenStartPoint.getSelected(), m_chosenReefPoints.getSelected(), 
            //             m_driveTrain, m_elevator, m_coralEffector, m_pivot, m_chosenFieldSide.getSelected().equals("Processor Side"));
            // }
            if(autoFlavor.equals("Refactor")) { 
                m_autoCommand = new CompBotExperimentalAutoRefactor(m_chosenStartPoint.getSelected(), FieldConstants.SOURCE_2_CENTER, m_chosenReefPoints.getSelected(), 
                        m_driveTrain, m_elevator, m_coralEffector, m_pivot, m_chosenFieldSide.getSelected().equals("Processor Side"));
            }
            
            m_autoSelectionCode = currentAutoSelectionCode;
        } 

        // System.out.println("Auto selection code: " + currentAutoSelectionCode + " actualAutoObj: " + m_autoCommand.hashCode());
        return m_autoCommand;
    }
    
    public Pose2d getInitialPose() {
        return ((AutoCommandInterface) getAutonomousCommand()).getInitialPose();
    }
    
    public Command getDriveCommand() {
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        return m_driveTrain.driveCommand(
            () -> -conditionAxis(m_driverController.getLeftY()),
            () -> -conditionAxis(m_driverController.getLeftX()),
            () -> -conditionAxis(m_driverController.getRightX()),
            // if you have a Logitech controller:
            // () -> -conditionAxis(m_driverController.getRawAxis(2)),
            m_farm.button(7));
    }
    
    private double conditionAxis(double value) {
        value = MathUtil.applyDeadband(value, JOYSTICK_DEADBAND);
        // Square the axis, retaining the sign
        return Math.abs(value) * value;
    }
    
        
    @Override
    public void resetAllGoals() {
        m_pivot.resetGoal();
        m_elevator.resetGoal();
    }

    public DriveTrain getDriveTrain() {
        return m_driveTrain;
    }
        
    @Override
    public CoralEffector getCoralEffector() {
        return m_coralEffector;
    }
}
