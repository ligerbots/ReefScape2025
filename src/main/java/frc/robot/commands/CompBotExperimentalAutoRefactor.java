// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Map;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.PathFactory;
import frc.robot.Robot;
import frc.robot.subsystems.AlgaeEffector;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffectorPivot;

public class CompBotExperimentalAutoRefactor extends ReefscapeAbstractAuto {

    private final double CORAL_PICKUP_WAIT_TIME;
    
    PathConstraints constraints =  new PathConstraints(
    4.0, 3.0,
    Math.toRadians(540), Math.toRadians(720));
    
        // private static Map<Pose2d, Double> elevatorRaiseTime = new HashMap<>();
        // static {
        //     elevatorRaiseTime.put(FieldConstants.REEF_J, 1.0);
        //     elevatorRaiseTime.put(FieldConstants.REEF_K, 0.8);
        //     elevatorRaiseTime.put(FieldConstants.REEF_L, 0.8);
        //     elevatorRaiseTime.put(FieldConstants.REEF_A, 0.9);
        // }
        
        private static Map<Pose2d, String> approachPathNames = new HashMap<>();
        static {
            approachPathNames.put(FieldConstants.REEF_K, "Source2Center to ReefApproachK");
            approachPathNames.put(FieldConstants.REEF_L, "Source2Center to ReefApproachL");
            approachPathNames.put(FieldConstants.REEF_A, "Source2Center to ReefApproachA");
        }
    
        public CompBotExperimentalAutoRefactor(Pose2d startPoint, Pose2d sourcePoint, Pose2d[] reefPoints, DriveTrain driveTrain, 
            Elevator elevator, CoralEffector coralEffector, AlgaeEffector algaeEffector, EndEffectorPivot pivot, boolean isProcessorSide, boolean doTushPush) {
                super(startPoint, sourcePoint, reefPoints, driveTrain, elevator, coralEffector, algaeEffector, pivot, isProcessorSide);
    
                
            if (Robot.isSimulation()) {
                CORAL_PICKUP_WAIT_TIME = 1.0;
            } else {
                // in real life, we wait for the coral to hit the limit switch
                CORAL_PICKUP_WAIT_TIME = 5.0;
            }
            try {
    
                PathPlannerPath firstCoralPath = PathFactory.getPath(startPoint, reefPoints[0], isProcessorSide);
    
                if(doTushPush) {
                    PathPlannerPath tushPushPath = PathFactory.getPath("StartX to TushPush", isProcessorSide);
                    m_initPose = tushPushPath.getStartingHolonomicPose().get();
    
                    PathPlannerPath driveBackToOriginalStart = PathFactory.getPath("TushPush to Start3", isProcessorSide);
    
                    addCommands(m_driveTrain.followPath(tushPushPath), m_driveTrain.followPath(driveBackToOriginalStart));
    
                } else { 
                    m_initPose = firstCoralPath.getStartingHolonomicPose().get();
                }
                
                addCommands(m_driveTrain.followPath(firstCoralPath).alongWith(
                    new WaitCommand(1.0).andThen(
                        new MoveEndEffector(Constants.Position.L4, elevator, pivot, RAISE_ELEVATOR_WAIT_TIME))));
                addCommands(new StartEndCommand(coralEffector::runOuttake, coralEffector::stop, coralEffector).withTimeout(CORAL_SCORE_WAIT_TIME));  
    
                if (reefPoints.length > 1) {
                    addCommands(
                        pickupCorralThenScoreL4(reefPoints[0], approachPathNames.get(reefPoints[1]), reefPoints[1]),
                        pickupCorralThenScoreL4(reefPoints[1], approachPathNames.get(reefPoints[2]), reefPoints[2]),
                        pickupCorralThenScoreL4(reefPoints[2], approachPathNames.get(reefPoints[3]), reefPoints[3])
                    );
                }
                
            } catch (Exception e) {
                DriverStation.reportError("Unable to load PP path Test", true);
                m_initPose = new Pose2d();
            }
        }
    
        private Pose2d mirrorIfNeeded(Pose2d pose) {
            return m_isProcessorSide ? FieldConstants.mirrorPose(pose) : pose;
        }
        
        private Command pickupCorralThenScoreL4(Pose2d driveStartPoint, String approachPath, Pose2d targetScore) {
            double raiseElevatorBeforeReef = RAISE_ELEVATOR_AFTER_PATH_START; //elevatorRaiseTime.get(targetScore);
            targetScore = mirrorIfNeeded(targetScore);
        return Commands.sequence(
                    Commands.parallel(new MoveEndEffector(Constants.Position.BACK_INTAKE, m_elevator, m_pivot, LOWER_ELEVATOR_WAIT_TIME),
                        Commands.parallel(m_driveTrain.followPath(PathFactory.getPath(driveStartPoint, m_sourcePoint, m_isProcessorSide)),
                                          new WaitCommand(START_INTAKE_AFTER_PATH_START).andThen(
                                              new StartEndCommand(m_coralEffector::runIntake, m_coralEffector::stop, m_coralEffector)
                                                    .until(m_coralEffector::hasCoral).withTimeout(CORAL_PICKUP_WAIT_TIME))
                            )
                        ),
                    Commands.parallel(
                        Commands.sequence(m_driveTrain.followPath(PathFactory.getPath(approachPath, m_isProcessorSide)),
                                        m_driveTrain.pathFindToPose(FieldConstants.flipPose(targetScore), constraints)
                                        ),
                        new WaitCommand(raiseElevatorBeforeReef).andThen(new MoveEndEffector(Constants.Position.L4, m_elevator, m_pivot, RAISE_ELEVATOR_WAIT_TIME))
                    ),
                    new StartEndCommand(m_coralEffector::runOuttake, m_coralEffector::stop, m_coralEffector).withTimeout(CORAL_SCORE_WAIT_TIME)
                );
    }

    private Command pickupCorralThenScoreL4Coast(Pose2d driveStartPoint, String approachPath, Pose2d targetScore) {
        targetScore = mirrorIfNeeded(targetScore);
        return Commands.sequence(
                    Commands.parallel(new MoveEndEffector(Constants.Position.BACK_INTAKE, m_elevator, m_pivot, LOWER_ELEVATOR_WAIT_TIME),
                        Commands.parallel(m_driveTrain.followPath(PathFactory.getPath(driveStartPoint, m_sourcePoint, m_isProcessorSide))
                                                .andThen(new InstantCommand(() -> m_driveTrain.setBrakeMode(false))),
                                          new WaitCommand(START_INTAKE_AFTER_PATH_START).andThen(
                                              new StartEndCommand(m_coralEffector::runIntake, m_coralEffector::stop, m_coralEffector)
                                                    .until(m_coralEffector::hasCoral).withTimeout(CORAL_PICKUP_WAIT_TIME))
                            )
                        ),
                    Commands.parallel(
                        Commands.sequence(m_driveTrain.followPath(PathFactory.getPath(approachPath, m_isProcessorSide)),
                                        m_driveTrain.pathFindToPose(FieldConstants.flipPose(targetScore), constraints)
                                        ),
                        new WaitCommand(RAISE_ELEVATOR_AFTER_PATH_START).andThen(new MoveEndEffector(Constants.Position.L4, m_elevator, m_pivot, RAISE_ELEVATOR_WAIT_TIME))
                    ),
                    new StartEndCommand(m_coralEffector::runOuttake, m_coralEffector::stop, m_coralEffector).withTimeout(CORAL_SCORE_WAIT_TIME)
                );
    }
}
