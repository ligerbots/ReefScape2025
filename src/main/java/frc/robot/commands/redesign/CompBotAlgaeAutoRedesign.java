// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.redesign;

import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import frc.robot.FieldConstants;
import frc.robot.PathFactory;
import frc.robot.Robot;
import frc.robot.commands.redesign.ReefscapeAbstractAutoRedesign;
import frc.robot.subsystems.AlgaeEffector;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.CoralGroundIntakeRedesign;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffectorPivot;
import frc.robot.subsystems.EndEffectorWrist;

public class CompBotAlgaeAutoRedesign extends ReefscapeAbstractAutoRedesign {
    private static final double CORAL_SCORE_WAIT_TIME = 0.1;
    public static final double RAISE_ELEVATOR_WAIT_TIME = 2.0;
    private static final double LOWER_ELEVATOR_WAIT_TIME = 0.5;  // maybe can be lower

    private static final boolean SHED_PRACTICE = false;

    private DriveTrain m_driveTrain;
    
    PathConstraints constraints =  new PathConstraints(
    4.0, 2.0,
    Math.toRadians(540), Math.toRadians(720));

    /** Creates a new NoteAuto. */
    public CompBotAlgaeAutoRedesign(Pose2d startPoint, Pose2d sourcePoint, Pose2d[] reefPoints, DriveTrain driveTrain, 
    Elevator elevator, Claw claw, EndEffectorWrist wrist, EndEffectorPivot pivot, CoralGroundIntakeRedesign coralGround, 
    boolean isProcessorSide) {
        super(startPoint, sourcePoint, reefPoints, driveTrain, elevator, claw, wrist, pivot, coralGround,
                isProcessorSide);
        m_driveTrain = driveTrain;
        
        double ALGAE_PICKUP_WAIT_TIME;
        if (Robot.isSimulation()) {
            ALGAE_PICKUP_WAIT_TIME = 1.0;
        } else {
            // in real life, we wait for the coral to hit the limit switch
            ALGAE_PICKUP_WAIT_TIME = 5.0;
        }

        Pose2d REEF_ALGAE_EF_AUTO_PICKUP = new Pose2d(5.140, 2.893, Rotation2d.fromDegrees(120.0));
        Pose2d REEF_ALGAE_GH_AUTO_PICKUP = new Pose2d(5.74, 4.021, Rotation2d.fromDegrees(180.0));
        Pose2d REEF_ALGAE_IJ_AUTO_PICKUP = new Pose2d(5.140, 5.148, Rotation2d.fromDegrees(-120.0));

        try {
            PathPlannerPath startPath = PathFactory.getPath("Start2 to ReefH", isProcessorSide);
            
            m_initPose = startPath.getStartingHolonomicPose().get();
            // m_initPose = FieldConstants.flipPose(driveTrain.getPose());
            
            addCommands(new InstantCommand(m_claw::hasCoral));
            addCommands(m_driveTrain.followPath(startPath).alongWith(
                    new MoveEndEffectorRedesign(Constants.Position.L4_PREP_ALT, elevator, pivot, wrist).withTimeout(1)));
            addCommands(new Score(() -> Position.L4_ALT, pivot, wrist, elevator, claw).withTimeout(0.5));
            addCommands(m_driveTrain.followPath(PathFactory.getPath("Algae backup path", isProcessorSide)));

            addCommands(m_driveTrain.pathFindToPose(FieldConstants.flipPose(REEF_ALGAE_GH_AUTO_PICKUP), constraints).alongWith(
                new MoveEndEffectorRedesign(Constants.Position.L2_ALGAE_ALT, elevator, pivot, wrist),
                new InstantCommand(m_claw::runIntake)
                ));
            addCommands(new WaitCommand(1.0).until(m_claw::hasAlgae));

            if(SHED_PRACTICE){
                addCommands(m_driveTrain.followPath(PathFactory.getPath("AlgaeGH to Barge SHED", false)).alongWith((new MoveEndEffectorRedesign(Constants.Position.STOW, elevator, pivot, wrist).withTimeout(1))));
        } else {
                addCommands(m_driveTrain.followPath(PathFactory.getPath("AlgaeGH to Barge", false)).alongWith((new MoveEndEffectorRedesign(Constants.Position.STOW, elevator, pivot, wrist).withTimeout(1))));
            }
        //
        // score algae: raise elevator, outtake algae, lower elevator
        //
            if(SHED_PRACTICE){
                addCommands(m_driveTrain.followPath(PathFactory.getPath("Barge SHED to AlgaeApproachEF", false)).alongWith(
                    (new MoveEndEffectorRedesign(Constants.Position.STOW, elevator, pivot, wrist).withTimeout(1))
                    //Lower Elevator while driving
                    ));
                    // Note mirrorPose for IJ pickup spot == EF pickup spot, which we want in the shed ONLY
                addCommands(m_driveTrain.pathFindToPose(FieldConstants.flipPose(REEF_ALGAE_EF_AUTO_PICKUP), constraints).alongWith(
                    new MoveEndEffectorRedesign(Constants.Position.L3_ALGAE_ALT, elevator, pivot, wrist),
                    new InstantCommand(m_claw::runIntake)
                    ));
                addCommands(m_driveTrain.followPath(PathFactory.getPath("AlgaeEF to Barge SHED", false)).alongWith((new MoveEndEffectorRedesign(Constants.Position.STOW, elevator, pivot, wrist).withTimeout(1))));

            } else {
                // REAL FIELD
                addCommands(m_driveTrain.followPath(PathFactory.getPath("Barge to AlgaeApproachIJ", false)).alongWith(
                    (new MoveEndEffectorRedesign(Constants.Position.STOW, elevator, pivot, wrist).withTimeout(1))
                    //Lower Elevator while driving
                    ));
                addCommands(m_driveTrain.pathFindToPose(FieldConstants.flipPose(REEF_ALGAE_IJ_AUTO_PICKUP), constraints).alongWith(
                    new MoveEndEffectorRedesign(Constants.Position.L3_ALGAE_ALT, elevator, pivot, wrist),
                    new InstantCommand(m_claw::runIntake)
                    ));
                addCommands(m_driveTrain.followPath(PathFactory.getPath("AlgaeIJ to Barge", false)).alongWith((new MoveEndEffectorRedesign(Constants.Position.STOW, elevator, pivot, wrist).withTimeout(1))));

            }
        //
        // score algae: raise elevator, outtake algae, lower elevator
        //

            Pose2d bargeBackupSpot = new Pose2d(6.25, 5.326, Rotation2d.fromDegrees(180.0));
            if(SHED_PRACTICE){
                bargeBackupSpot = FieldConstants.mirrorPose(bargeBackupSpot); // only for shed 
            }

            addCommands(m_driveTrain.pathFindToPose(FieldConstants.flipPose(bargeBackupSpot), constraints));
            
            // addCommands(new StartEndCommand(algaeEffector::runIntake, algaeEffector::stop, algaeEffector).until(algaeEffector::hasAlgae).withTimeout(ALGAE_PICKUP_WAIT_TIME));
            // addCommands(m_driveTrain.followPath(PathFactory.getPath("AlgaeGH to Barge", false)).alongWith(
            //     new WaitCommand(0.2).andThen(new MoveEndEffector(Constants.Position.BARGE, elevator, pivot, LOWER_ELEVATOR_WAIT_TIME))));              
            // addCommands(new StartEndCommand(algaeEffector::score, algaeEffector::stop, algaeEffector).withTimeout(0.5));
            // addCommands(
            //         m_driveTrain.followPath(PathFactory.getPath("Barge to AlgaeApproachIJ", false)).alongWith(
            //             new WaitCommand(.1).andThen(
            //             new MoveEndEffector(Constants.Position.L3_ALGAE, elevator, pivot, RAISE_ELEVATOR_WAIT_TIME))));

//             addCommands(
//                 Commands.sequence(
//                     // m_driveTrain.followPath(PathFactory.getPath("Barge to AlgaeApproachIJ", false)).alongWith(
//                     //     new WaitCommand(.1).andThen(
//                     //     new MoveEndEffector(Constants.Position.STOW, elevator, pivot, RAISE_ELEVATOR_WAIT_TIME))),
// /* 
//                     m_driveTrain.pathFindToPose(FieldConstants.flipPose(REEF_ALGAE_IJ_AUTO_PICKUP), constraints).alongWith(
//                         new MoveEndEffector(Constants.Position.L3_ALGAE, elevator, pivot, LOWER_ELEVATOR_WAIT_TIME))
// */
//                     // new StartEndCommand(algaeEffector::runIntake, algaeEffector::stop, algaeEffector).until(algaeEffector::hasAlgae).withTimeout(ALGAE_PICKUP_WAIT_TIME),
            
//                     // m_driveTrain.followPath(PathFactory.getPath("AlgaeIJ to Barge", false)).alongWith(
//                     //     new WaitCommand(.5).andThen(
//                     //     new MoveEndEffector(Constants.Position.BARGE, elevator, pivot, RAISE_ELEVATOR_WAIT_TIME))),

//                     // new StartEndCommand(algaeEffector::score, algaeEffector::stop, algaeEffector).withTimeout(0.5)

//                     ));
                
            // Pose2d bargeBackupSpot = new Pose2d(6.25, 5.326, Rotation2d.fromDegrees(180.0));
            // addCommands(m_driveTrain.pathFindToPose(FieldConstants.flipPose(bargeBackupSpot), constraints));
            // addCommands(new MoveEndEffector(Constants.Position.BACK_INTAKE, elevator, pivot, LOWER_ELEVATOR_WAIT_TIME));

   
            
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
