// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.kitbot;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.FieldConstants;
import frc.robot.commands.AutoCommandInterface;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.kitbot.KitbotRoller;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HelloWorldAuto extends AutoCommandInterface {
    private DriveTrain m_driveTrain;
    private Pose2d m_initPose;

    /** Creates a new NoteAuto. */
    public HelloWorldAuto(DriveTrain driveTrain, KitbotRoller roller) {
        m_driveTrain = driveTrain;

        try {
            // PathPlannerAuto firstAuto = new PathPlannerAuto("Shot1 to Source1");

            PathPlannerPath startPath = PathPlannerPath.fromPathFile("Start1 to Shot1");
            m_initPose = startPath.getStartingDifferentialPose();
            
            addCommands(m_driveTrain.followPath(startPath));
            addCommands(new StartEndCommand(roller::runRollerOut, roller::stop, roller).withTimeout(1));

            addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("Shot1 to Source1")));
            addCommands(new WaitCommand(1));
            addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("Source1 to Shot2")));
            addCommands(new StartEndCommand(roller::runRollerOut, roller::stop, roller).withTimeout(1));

            addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("Shot2 to Source1")));
            addCommands(new WaitCommand(1));

            addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("Source1 to Shot2")));
            addCommands(new StartEndCommand(roller::runRollerOut, roller::stop, roller).withTimeout(1));

            // addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("Shot2 to Source1")));
            // addCommands(m_driveTrain.followPath(PathPlannerPath.fromPathFile("Source1 to Shot2")));

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
