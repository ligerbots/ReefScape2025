// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;

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
public class HelloWorldAuto2 extends AutoCommandInterface {
    private DriveTrain m_driveTrain;
    private Pose2d m_initPose;

    /** Creates a new NoteAuto. */
    public HelloWorldAuto2(DriveTrain driveTrain, KitbotRoller roller, boolean awaySideAuto) {
        m_driveTrain = driveTrain;

        try {

            if(awaySideAuto) {
                PathPlannerPath startPath = PathPlannerPath.fromPathFile("Start2 to Shot1a");
                m_initPose = startPath.getStartingDifferentialPose();
                
                addCommands(m_driveTrain.followPath(startPath));
                addCommands(new StartEndCommand(roller::runRollerOut, roller::stop, roller).withTimeout(.5));

                addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("Shot1a to Source2")));
                addCommands(new WaitCommand(.75));
                addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("Source2 to Shot2a")));
                addCommands(new StartEndCommand(roller::runRollerOut, roller::stop, roller).withTimeout(.5));

                addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("Shot2a to Source2")));
                addCommands(new WaitCommand(.75));
                addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("Source2 to ReefA")));
                addCommands(new StartEndCommand(roller::runRollerOut, roller::stop, roller).withTimeout(.5));

                addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("ReefA to Source2")));
                addCommands(new WaitCommand(.75));
                addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("Source2 to ReefA")));
                addCommands(new StartEndCommand(roller::runRollerOut, roller::stop, roller).withTimeout(.5));
            } else {
                PathPlannerPath startPath = PathPlannerPath.fromPathFile("Start1 to Shot1");
                m_initPose = startPath.getStartingDifferentialPose();
                
                addCommands(m_driveTrain.followPath(startPath));
                addCommands(new StartEndCommand(roller::runRollerOut, roller::stop, roller).withTimeout(.3));

                addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("Shot1 to Source1")));
                // addCommands(new WaitCommand(.1));
                addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("Source1 to Shot2")));
                addCommands(new StartEndCommand(roller::runRollerOut, roller::stop, roller).withTimeout(.3));
                
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
