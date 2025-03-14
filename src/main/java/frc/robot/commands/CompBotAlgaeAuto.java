// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.PathFactory;
import frc.robot.Robot;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffectorPivot;

public class CompBotAlgaeAuto extends ReefscapeAbstractAuto {
    private static final double CORAL_SCORE_WAIT_TIME = 0.1;
    public static final double RAISE_ELEVATOR_WAIT_TIME = 2.0;
    private static final double LOWER_ELEVATOR_WAIT_TIME = 0.5;  // maybe can be lower

    private DriveTrain m_driveTrain;
    
    PathConstraints constraints =  new PathConstraints(
    4.0, 2.0,
    Math.toRadians(540), Math.toRadians(720));

    /** Creates a new NoteAuto. */
    public CompBotAlgaeAuto(Pose2d startPoint, Pose2d sourcePoint, Pose2d[] reefPoints, DriveTrain driveTrain, 
    Elevator elevator, CoralEffector coralEffector, EndEffectorPivot pivot, boolean isProcessorSide) {
        super(startPoint, sourcePoint, reefPoints, driveTrain, elevator, coralEffector, pivot, isProcessorSide);
        m_driveTrain = driveTrain;
        

        try {
            PathPlannerPath startPath = PathFactory.getPath(startPoint, reefPoints[0], isProcessorSide);
            
            m_initPose = startPath.getStartingDifferentialPose();
            
            addCommands(m_driveTrain.followPath(startPath).alongWith(
                new MoveEndEffector(Constants.Position.L4, elevator, pivot, RAISE_ELEVATOR_WAIT_TIME)));
            addCommands(new StartEndCommand(coralEffector::runOuttake, coralEffector::stop, coralEffector).withTimeout(CORAL_SCORE_WAIT_TIME));                
            if (reefPoints.length > 1) {

                addCommands(
                    new MoveEndEffector(Constants.Position.L3_ALGAE, elevator, pivot, LOWER_ELEVATOR_WAIT_TIME).alongWith(
                    m_driveTrain.followPath(PathFactory.getPath(reefPoints[0], sourcePoint, isProcessorSide))));

                    // ,
                    // new WaitCommand(1.5).andThen(
                    //     new StartEndCommand(coralEffector::runIntake, coralEffector::stop, coralEffector).until(coralEffector::hasCoral).withTimeout(CORAL_PICKUP_WAIT_TIME))
                    //     ));
                    
                // addCommands(m_driveTrain.followPath(PathFactory.getPath(sourcePoint, reefPoints[1], isProcessorSide))
                /* 
                addCommands(m_driveTrain.followPath(PathFactory.getPath("Source2Center to ReefApproachK", isProcessorSide)));
                Pose2d reefPoint1 = isProcessorSide ? FieldConstants.mirrorPose(reefPoints[1]) : reefPoints[1];
                Pose2d reefPoint2 = isProcessorSide ? FieldConstants.mirrorPose(reefPoints[2]) : reefPoints[2];

                addCommands(m_driveTrain.pathFindToPose(FieldConstants.flipPose(reefPoint1), constraints)
                        .alongWith(new MoveEndEffector(Constants.Position.L4, elevator, pivot, RAISE_ELEVATOR_WAIT_TIME)));
                
                addCommands(new StartEndCommand(coralEffector::runOuttake, coralEffector::stop, coralEffector).withTimeout(CORAL_SCORE_WAIT_TIME));                
                
                addCommands(new MoveEndEffector(Constants.Position.BACK_INTAKE, elevator, pivot, LOWER_ELEVATOR_WAIT_TIME)
                .alongWith(     
                    m_driveTrain.followPath(PathFactory.getPath(reefPoints[1], sourcePoint, isProcessorSide)),
                    new WaitCommand(1.5).andThen(
                        new StartEndCommand(coralEffector::runIntake, coralEffector::stop, coralEffector).until(coralEffector::hasCoral).withTimeout(CORAL_PICKUP_WAIT_TIME)
                    )));
                    
                addCommands(m_driveTrain.followPath(PathFactory.getPath("Source2Center to ReefApproachL", isProcessorSide)));                    
                // addCommands(m_driveTrain.followPath(PathFactory.getPath(sourcePoint, reefPoints[2], isProcessorSide))
                addCommands(m_driveTrain.pathFindToPose(FieldConstants.flipPose(reefPoint2), constraints)
                        .alongWith(new MoveEndEffector(Constants.Position.L4, elevator, pivot, RAISE_ELEVATOR_WAIT_TIME)));
                addCommands(new StartEndCommand(coralEffector::runOuttake, coralEffector::stop, coralEffector).withTimeout(CORAL_SCORE_WAIT_TIME));                
                addCommands(new MoveEndEffector(Constants.Position.BACK_INTAKE, elevator, pivot, LOWER_ELEVATOR_WAIT_TIME));
                */
                

        }
            
        } catch (Exception e) {
            DriverStation.reportError("Unable to load PP path Test", true);
            m_initPose = new Pose2d();
        }
    }
    
    @Override
    public Pose2d getInitialPose() {
        return FieldConstants.flipPose(m_initPose);
    }
}
