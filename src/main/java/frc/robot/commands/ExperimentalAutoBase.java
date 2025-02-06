// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.kitbot.KitbotRoller;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExperimentalAutoBase extends AutoCommandInterface {
    private DriveTrain m_driveTrain;
    private Pose2d m_initPose;

    /** Creates a new NoteAuto. */
    public ExperimentalAutoBase(DriveTrain driveTrain, KitbotRoller roller, boolean processorSideAuto) {
        m_driveTrain = driveTrain;

        try {

            if(processorSideAuto) {
                PathPlannerPath startPath = PathPlannerPath.fromPathFile("Start1 to ReefF");
                m_initPose = startPath.getStartingDifferentialPose();
                
                // addCommands(m_driveTrain.followPath(startPath));
                // addCommands(AutoBuilder.followPath(startPath));
                List<Pose2d> j = startPath.getPathPoses(); 
                Pose2d targetDest = FlippingUtil.flipFieldPose(j.get(j.size()-1));
                // PathConstraints(double maxVelocityMPS, double maxAccelerationMPSSq, double maxAngularVelocityRadPerSec, double maxAngularAccelerationRadPerSecSq)
                PathConstraints pConstraints = new PathConstraints(4.5, 4.0, 540.0, 720.0);
                // "globalConstraints": {
                //     "maxVelocity": 4.5,
                //     "maxAcceleration": 4.0,
                //     "maxAngularVelocity": 540.0,
                //     "maxAngularAcceleration": 720.0,
                //     "nominalVoltage": 12.0,
                //     "unlimited": false
                //   },
                
                addCommands(AutoBuilder.pathfindToPose(targetDest, pConstraints));

                addCommands(new ScoreCommandKitBot(roller).withTimeout(.3));

                addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("ReefF to Source1")));
                // addCommands(new WaitCommand(.1));
                addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("Source1 to ReefD")));
                addCommands(new ScoreCommandKitBot(roller).withTimeout(.3));

                
                addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("Shot2 J Path")));

                addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("Shot2 J Path ReefB")));


                // addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("Shot2 to Source1")));
                // // addCommands(new WaitCommand(.1));

                // addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("Source1 to ReefB")));
                // addCommands(new StartEndCommand(roller::runRollerOut, roller::stop, roller).withTimeout(.3));

                // addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("ReefB to Source1")));
                // // addCommands(new WaitCommand(.1));
                // addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("Source1 to ReefB")));
                // addCommands(new StartEndCommand(roller::runRollerOut, roller::stop, roller).withTimeout(.3));
            } else {
                PathPlannerPath startPath = PathPlannerPath.fromPathFile("Start2 to ReefJ");
                m_initPose = startPath.getStartingDifferentialPose();
                
                addCommands(m_driveTrain.followPath(startPath));
                addCommands(new ScoreCommandKitBot(roller).withTimeout(.3));

                addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("ReefJ to Source2")));
                addCommands(new WaitCommand(.75));
                addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("Source2 to ReefK")));
                addCommands(new ScoreCommandKitBot(roller).withTimeout(.3));
                addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("ReefK to Source2")));
                addCommands(new WaitCommand(.75));
                addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("Source2 to ReefA")));
                addCommands(new ScoreCommandKitBot(roller).withTimeout(.3));

                addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("ReefA to Source2")));
                addCommands(new WaitCommand(.75));
                addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("Source2 to ReefA")));
                addCommands(new ScoreCommandKitBot(roller).withTimeout(.3));
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
