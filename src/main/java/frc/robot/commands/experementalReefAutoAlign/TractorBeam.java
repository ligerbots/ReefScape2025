package frc.robot.commands.experementalReefAutoAlign;

import java.util.function.Supplier;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class TractorBeam implements Supplier<Command> {
    private final DriveTrain m_driveTrain;

    private final Pose2d m_goalPose;

    private final PathConstraints m_constraints = new PathConstraints(
            3.0, 2.5,
            Math.toRadians(540), Math.toRadians(720));

    public TractorBeam(DriveTrain driveTrain, Pose2d goalPose2d) {
        m_driveTrain = driveTrain;
        m_goalPose = goalPose2d;

        // do not Require the drivetrain - the outside commands handles that
    }

    @Override
    public Command get() { 
        return m_driveTrain.pathFindToPose(m_goalPose, m_constraints);
    }
}
