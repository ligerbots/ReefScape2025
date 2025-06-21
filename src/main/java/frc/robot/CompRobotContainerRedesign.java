// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Objects;
import java.util.Set;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Position;
import frc.robot.commands.*;
import frc.robot.commands.redesign.*;
import frc.robot.subsystems.*;

public class CompRobotContainerRedesign extends RobotContainer {
    private static final double JOYSTICK_DEADBAND = 0.05;

    private Position currRobotAction = Position.BACK_INTAKE;
    
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandJoystick m_farm = new CommandJoystick(1);
    
    private final AprilTagVision m_aprilTagVision = new AprilTagVision();
    private final DriveTrain m_driveTrain = new DriveTrain("swerve/comp", m_aprilTagVision);
    // private final Leds m_leds = new Leds();
    // private final PowerDistribution m_pdh = new PowerDistribution();

    private final Elevator m_elevator = new Elevator();
    private final EndEffectorPivot m_pivot = new EndEffectorPivot(() -> m_elevator.getHeight());
    private final EndEffectorWrist m_wrist = new EndEffectorWrist(()-> m_elevator.getGoal());
    private final Claw m_claw = new Claw(()-> m_elevator.getGoal());
    private final CoralGroundIntakeRedesign m_coralGroundIntake = new CoralGroundIntakeRedesign();
    private final AlgaeGroundIntakeRedesign m_algaeGroundIntake = new AlgaeGroundIntakeRedesign();

    private final Climber m_climber = new Climber();

    // @SuppressWarnings("unused")
    // private final DriverRumble m_driverRumble = new DriverRumble(
    //     m_driverController.getHID(), () -> m_driveTrain.getPose(), 
    //     () -> m_coralEffector.hasCoral(), () -> m_algaeEffector.hasAlgae(),
    //     () -> m_climber.isDeployed(), () -> m_driveTrain.readyToClimb());
    
    private boolean m_coralMode = true;
    
    private final SendableChooser<String> m_chosenFieldSide = new SendableChooser<>(); 
    // private final SendableChooser<Pose2d> m_chosenStartPoint = new SendableChooser<>();    
    // private final SendableChooser<Pose2d> m_chosenSourcePickup = new SendableChooser<>();
    private final SendableChooser<String> m_chosenAutoFlavor = new SendableChooser<>(); 
    // private final SendableChooser<Pose2d[]> m_chosenReefPoints = new SendableChooser<>();

    private ReefscapeAbstractAutoRedesign m_autoCommand;
    private int m_autoSelectionCode = 0;
    
    private static final Pose2d[] REEF_POINTS_JKLA = {FieldConstants.REEF_J, FieldConstants.REEF_K, FieldConstants.REEF_L, FieldConstants.REEF_A};
    private static final Pose2d[] REEF_POINTS_JKAL = {FieldConstants.REEF_J, FieldConstants.REEF_K, FieldConstants.REEF_A, FieldConstants.REEF_L};
    private static final Pose2d[] REEF_POINTS_H = { FieldConstants.REEF_H };

    public CompRobotContainerRedesign() {
        m_elevator.setPivotCheckSupplier(() -> m_pivot.isOutsideLowRange());
        
        configureBindings();
        configureAutos();
        
        m_driveTrain.setDefaultCommand(getDriveCommand());
    }
    
    private void configureBindings() {
        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }
        
        m_driverController.leftTrigger().onTrue(
                        new InstantCommand(m_coralGroundIntake::deploy));
        m_driverController.leftTrigger().onFalse(new InstantCommand(m_coralGroundIntake::stow));

        m_driverController.rightTrigger().whileTrue(
                new ConditionalCommand(
                new Score(currRobotAction, m_pivot, m_wrist, m_elevator, m_claw, ()->m_elevator.getHeight()),

                new StartEndCommand(m_claw::runOuttake, m_claw::stop, m_claw),
                        () -> m_coralMode)
        );

        // Trigger coralRumble = new Trigger(() -> m_coralEffector.hasCoral());

        // coralRumble.onTrue(new InstantCommand(() -> m_driverRumble.rumble()));
        // Trigger algaeRumble = new Trigger(() -> m_algaeEffector.hasAlgae());
        // algaeRumble.onTrue(new InstantCommand(() -> m_driverRumble.rumble()));
        
        // m_driverController.rightBumper().onTrue(new MoveEndEffector(Constants.Position.STOW, m_elevator, m_pivot).andThen().finallyDo(() -> m_coralMode = true));
        
        m_driverController.rightBumper().onTrue(new DeferredCommand(new ReefTractorBeamWithDirectPath(m_driveTrain, false, m_claw::hasCoral), Set.of(m_driveTrain)));
        m_driverController.leftBumper().onTrue(new ConditionalCommand(new DeferredCommand(new ReefTractorBeamWithDirectPath(m_driveTrain, true, m_claw::hasCoral), Set.of(m_driveTrain)), new StartEndCommand(m_claw::runIntake, m_claw::stop, m_claw), ()->m_coralMode));

        // Algae Scoring
        m_driverController.a().onTrue(new MoveEndEffectorRedesign(Constants.Position.L2_ALGAE, m_elevator, m_pivot, m_wrist).alongWith(new InstantCommand(()-> currRobotAction = Position.L2_ALGAE)).alongWith(new InstantCommand(() -> m_coralMode = false)));
        m_driverController.x().onTrue(new MoveEndEffectorRedesign(Constants.Position.L3_ALGAE, m_elevator, m_pivot, m_wrist).alongWith(new InstantCommand(()-> currRobotAction = Position.L3_ALGAE)).alongWith(new InstantCommand(() -> m_coralMode = false)));
        m_driverController.y().onTrue(new MoveEndEffectorRedesign(Constants.Position.BARGE, m_elevator, m_pivot, m_wrist).alongWith(new InstantCommand(()-> currRobotAction = Position.BARGE)).alongWith(new InstantCommand(() -> m_coralMode = false)));
        m_driverController.b().onTrue(new MoveEndEffectorRedesign(Constants.Position.STOW, m_elevator, m_pivot, m_wrist ).alongWith(new InstantCommand(()-> currRobotAction = Position.STOW)));

        m_driverController.leftStick().onTrue(new GoToAlgaeTransfer(m_pivot, m_wrist, m_elevator, m_claw, m_coralGroundIntake, m_algaeGroundIntake));
        
        m_driverController.leftStick().onFalse(new MoveEndEffectorRedesign(Constants.Position.STOW, m_elevator, m_pivot, m_wrist));
        
        m_driverController.rightStick().onTrue(new PrintCommand("right stick pressed"));
        // m_driverController.b().onTrue(new MoveEndEffectorRedesign(Constants.Position.PROCESSOR, m_elevator,sim m_pivot, m_wrist ).alongWith(new InstantCommand(() -> m_coralMode = false)));

        // Coral Scoring
        m_driverController.pov(270).onTrue(new ConditionalCommand(new MoveEndEffectorRedesign(Constants.Position.L4_PREP, m_elevator, m_pivot, m_wrist).alongWith(new InstantCommand(()-> currRobotAction = Position.L4_PREP)).alongWith(new InstantCommand(() -> m_coralMode = true)), 
        new TransferWithPos(m_pivot, m_wrist, m_elevator, m_claw, ()-> m_elevator.getHeight(), m_coralGroundIntake, Position.L4_PREP).alongWith(new InstantCommand(() -> m_coralMode = true)),
         m_coralGroundIntake::HasCoral));
       
        m_driverController.pov(0).onTrue(new ConditionalCommand(new MoveEndEffectorRedesign(Constants.Position.L3_PREP, m_elevator, m_pivot, m_wrist).alongWith(new InstantCommand(()-> currRobotAction = Position.L4_PREP)).alongWith(new InstantCommand(() -> m_coralMode = true)), 
        new TransferWithPos(m_pivot, m_wrist, m_elevator, m_claw, ()-> m_elevator.getHeight(), m_coralGroundIntake, Position.L3_PREP).alongWith(new InstantCommand(() -> m_coralMode = true)),
         m_coralGroundIntake::HasCoral));
       
        m_driverController.pov(180).onTrue(new ConditionalCommand(new MoveEndEffectorRedesign(Constants.Position.L2_PREP, m_elevator, m_pivot, m_wrist).alongWith(new InstantCommand(()-> currRobotAction = Position.L4_PREP)).alongWith(new InstantCommand(() -> m_coralMode = true)), 
        new TransferWithPos(m_pivot, m_wrist, m_elevator, m_claw, ()-> m_elevator.getHeight(), m_coralGroundIntake, Position.L2_PREP).alongWith(new InstantCommand(() -> m_coralMode = true)),
         m_coralGroundIntake::HasCoral));
        

        m_driverController.pov(90).onTrue(new TransferWithPos(m_pivot, m_wrist, m_elevator, m_claw, ()-> m_elevator.getHeight(), m_coralGroundIntake, Position.STOW).alongWith(new InstantCommand(() -> m_coralMode = true)));
        
        // Climber
        m_driverController.start().onTrue(new InstantCommand(m_climber::climb));
        m_driverController.back().onTrue(new InstantCommand(m_climber::deploy));
        m_driverController.back().onTrue(new MoveEndEffectorRedesign(Constants.Position.CLIMB, m_elevator, m_pivot, m_wrist));

        //these are the 4 buttons in the square on the top of the farm controller to make sure the command gets run even if Zach misses the button. 
        m_farm.button(1).whileTrue(new StartEndCommand(() -> m_climber.run(Climber.MANUAL_SPEED), m_climber::hold, m_climber));
        m_farm.button(1).onTrue(new MoveEndEffectorRedesign(Constants.Position.CLIMB, m_elevator, m_pivot, m_wrist));

        m_farm.button(2).whileTrue(new StartEndCommand(() -> m_climber.run(Climber.MANUAL_SPEED), m_climber::hold, m_climber));
        m_farm.button(2).onTrue(new MoveEndEffectorRedesign(Constants.Position.CLIMB, m_elevator, m_pivot, m_wrist));

        m_farm.button(6).whileTrue(new StartEndCommand(() -> m_climber.run(Climber.MANUAL_SPEED), m_climber::hold, m_climber));
        m_farm.button(6).onTrue(new MoveEndEffectorRedesign(Constants.Position.CLIMB, m_elevator, m_pivot,m_wrist));

        m_farm.button(7).whileTrue(new StartEndCommand(() -> m_climber.run(Climber.MANUAL_SPEED), m_climber::hold, m_climber));
        m_farm.button(7).onTrue(new MoveEndEffectorRedesign(Constants.Position.CLIMB, m_elevator, m_pivot, m_wrist));


        // m_farm.button(2).whileTrue(new StartEndCommand(() -> m_climber.run(-Climber.MANUAL_SPEED), m_climber::hold, m_climber));
        // m_farm.button(2).onTrue(new MoveEndEffector(Constants.Position.CLIMB, m_elevator, m_pivot, 0));

        // Miscellaneous
        // m_farm.button(6).onTrue(new InstantCommand(m_driveTrain::lock, m_driveTrain));
        
        // m_farm.button(21).onTrue(new MoveEndEffector(Constants.Position.L1, m_elevator, m_pivot));
        // note: farm 7 is robot-centric
        m_farm.button(8).whileTrue(new StartEndCommand(m_claw::runOuttake, m_claw::stop, m_claw));
        m_farm.button(5).whileTrue(new StartEndCommand(m_coralGroundIntake::TransferCoral, m_coralGroundIntake::stow, m_coralGroundIntake));

        m_farm.button(4).onTrue(new MoveEndEffectorRedesign(Position.TRANSFER_WAIT, m_elevator, m_pivot, m_wrist));


        
        // m_farm.button(11).onTrue(new InstantCommand(() -> m_coralMode = !m_coralMode));
        // m_farm.button(5).whileTrue(new InstantCommand(m_elevator::zeroElevator));
        // m_farm.button(12).onTrue(new DeferredCommand(new ReefTractorBeamWithDirectPath(m_driveTrain, false, ()->false), Set.of(m_driveTrain)));

        // schedule Drive command, which will cancel other control of Drivetrain, ie active heading
        m_farm.button(16).onTrue(new InstantCommand(() -> m_driveTrain.getDefaultCommand().schedule()));
        m_farm.button(14).onTrue(new MoveEndEffector(Constants.Position.FRONT_INTAKE, m_elevator, m_pivot).alongWith(new InstantCommand(() -> m_coralMode = true)));

        // Testing commands
        m_farm.button(18).onTrue(new InstantCommand(m_coralGroundIntake::stow));
        m_farm.button(19).onTrue(new InstantCommand(m_coralGroundIntake::deploy));
        m_farm.button(20).onTrue(new InstantCommand(m_coralGroundIntake::score));

        m_farm.button(22).onTrue(new InstantCommand(() -> m_elevator.setHeight(Units.inchesToMeters(SmartDashboard.getNumber("elevator/testGoal", 0)))));
        m_farm.button(23).onTrue(new InstantCommand(() -> m_pivot.setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("pivot/testAngle", 0.0)))));
            //Flip end effector to stow
        m_farm.button(24).onTrue(new InstantCommand(() -> m_wrist.setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("wrist/testAngle", 0.0)))));
    }
    
    private void configureAutos() {
        // NamedCommands.registerCommand("raiseElevatorBeforeReef", 
            
        // m_chosenReefPoints.setDefaultOption("JKLA (aka EDCB)", REEF_POINTS_JKLA);

        // m_chosenReefPoints.addOption("H only  (aka G only) Center auto", REEF_POINTS_H);

        // Pose2d[] reefPoints4 = { FieldConstants.REEF_H, FieldConstants.REEF_ALGAE_GH };
        // m_chosenReefPoints.addOption("H coral score, then grab GH Algae", reefPoints4);

        m_chosenFieldSide.setDefaultOption("Processor Side", "Processor Side");
        m_chosenFieldSide.addOption("Barge Side", "Barge Side");

        // m_chosenStartPoint.setDefaultOption("3rd cage-- usual spot", FieldConstants.ROBOT_START_3);
        // m_chosenStartPoint.addOption("Field Center", FieldConstants.ROBOT_START_2);

        m_chosenAutoFlavor.addOption("Algae", "Algae");
        m_chosenAutoFlavor.setDefaultOption("Primary Coral - JK-L", "Primary");
        m_chosenAutoFlavor.addOption("Secondary Coral - JK-A", "Secondary");
        m_chosenAutoFlavor.addOption("TushPush then Primary Coral", "TushPush");
        m_chosenAutoFlavor.addOption("SingleL4+PickupAlgae", "AlgaeAlt");


        SmartDashboard.putData("Field Side", m_chosenFieldSide);
        SmartDashboard.putData("Auto flavor", m_chosenAutoFlavor);
    }
    
    public Command getAutonomousCommand() {
        // int currentAutoSelectionCode = Objects.hash(m_chosenStartPoint.getSelected(), m_chosenAutoFlavor.getSelected(),
        //     m_chosenReefPoints.getSelected(), m_chosenFieldSide.getSelected(), DriverStation.getAlliance());
            int currentAutoSelectionCode = Objects.hash(m_chosenAutoFlavor.getSelected(),
                m_chosenFieldSide.getSelected(), DriverStation.getAlliance());
 
            // Only call constructor if the auto selection inputs have changed
        if (m_autoSelectionCode != currentAutoSelectionCode) {
            String autoFlavor = m_chosenAutoFlavor.getSelected();

            m_autoCommand = new CompBotRedesignAuto(FieldConstants.ROBOT_START_2, FieldConstants.ROBOT_START_2, REEF_POINTS_H, 
                     m_driveTrain, m_elevator, m_claw , m_wrist, m_pivot, m_coralGroundIntake, m_chosenFieldSide.getSelected().equals("Processor Side"), false);
            
            // if(autoFlavor.equals("Algae")) { 
            //     m_autoCommand = new CompBotAlgaeAuto(FieldConstants.ROBOT_START_2, FieldConstants.ROBOT_START_2, REEF_POINTS_H, 
            //             m_driveTrain, m_elevator, m_coralEffector, m_algaeEffector, m_pivot, m_chosenFieldSide.getSelected().equals("Processor Side"));
            // }
            // if(autoFlavor.equals("Primary")) { 
            //     m_autoCommand = new CompBotExperimentalAutoRefactor(FieldConstants.ROBOT_START_3, FieldConstants.SOURCE_2_CENTER, REEF_POINTS_JKLA, 
            //             m_driveTrain, m_elevator, m_coralEffector, m_algaeEffector, m_pivot, m_chosenFieldSide.getSelected().equals("Processor Side"), false);
            // }
            // if(autoFlavor.equals("Secondary")) { 
            //     m_autoCommand = new CompBotExperimentalAutoRefactor(FieldConstants.ROBOT_START_3, FieldConstants.SOURCE_2_CENTER, REEF_POINTS_JKAL, 
            //             m_driveTrain, m_elevator, m_coralEffector, m_algaeEffector, m_pivot, m_chosenFieldSide.getSelected().equals("Processor Side"), false);
            // }

            // if(autoFlavor.equals("TushPush")) { 
            //     m_autoCommand = new CompBotExperimentalAutoRefactor(FieldConstants.ROBOT_START_3, FieldConstants.SOURCE_2_CENTER, REEF_POINTS_JKLA, 
            //             m_driveTrain, m_elevator, m_coralEffector, m_algaeEffector, m_pivot, m_chosenFieldSide.getSelected().equals("Processor Side"), true);
            // }

            // if(autoFlavor.equals("AlgaeAlt")) { 
            //     m_autoCommand = new CompBotAlgaePickupAuto(FieldConstants.ROBOT_START_2, FieldConstants.ROBOT_START_2, REEF_POINTS_H, 
            //             m_driveTrain, m_elevator, m_coralEffector, m_algaeEffector, m_pivot, m_chosenFieldSide.getSelected().equals("Processor Side"));
            // }
            
            
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
        m_climber.resetGoal();
        m_wrist.initWristEncoder();
    }

    public DriveTrain getDriveTrain() {
        return m_driveTrain;
    }
        
    // @Override
    // public CoralEffector getCoralEffector() {
    //     return m_coralEffector;
    // }
}
