// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.kitbot;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.FieldConstants;
import frc.robot.PathFactory;
import frc.robot.commands.AutoCommandInterface;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.kitbot.KitbotRoller;

public class KitBotGenericAutoBase extends AutoCommandInterface {
    private DriveTrain m_driveTrain;
    private Pose2d m_initPose;

    /** Creates a new NoteAuto. */
    public KitBotGenericAutoBase(Pose2d startPoint, Pose2d sourcePoint, Pose2d[] reefPoints, DriveTrain driveTrain, KitbotRoller roller, boolean isProcessorSide) {
        m_driveTrain = driveTrain;

        try {
                PathPlannerPath startPath = PathFactory.getPath(startPoint, reefPoints[0], isProcessorSide);

                m_initPose = startPath.getStartingDifferentialPose();
                
                addCommands(m_driveTrain.followPath(startPath));
                addCommands(new ScoreCommandKitBot(roller).withTimeout(.3));

                addCommands(m_driveTrain.followPath(PathFactory.getPath(reefPoints[0], sourcePoint, isProcessorSide)));
                addCommands(new WaitCommand(.75));
                addCommands(m_driveTrain.followPath(PathFactory.getPath(sourcePoint, reefPoints[1], isProcessorSide)));
                addCommands(new ScoreCommandKitBot(roller).withTimeout(.3));
                addCommands(m_driveTrain.followPath(PathFactory.getPath(reefPoints[1], sourcePoint, isProcessorSide)));

                addCommands(new WaitCommand(.75));
                addCommands(m_driveTrain.followPath(PathFactory.getPath(sourcePoint, reefPoints[2], isProcessorSide)));

                addCommands(new ScoreCommandKitBot(roller).withTimeout(.3));                
           
                if(reefPoints.length >3) {
                    addCommands(m_driveTrain.followPath(PathFactory.getPath(reefPoints[2], sourcePoint, isProcessorSide)));
                    addCommands(new WaitCommand(.75));
                    addCommands(m_driveTrain.followPath(PathFactory.getPath(sourcePoint, reefPoints[3], isProcessorSide)));
    
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
