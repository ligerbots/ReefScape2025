package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;

public class ReefTractorBeam implements Supplier<Command> {

    private final DriveTrain m_driveTrain;
    private final boolean m_goLeft;
    private final PathConstraints m_constraints =  new PathConstraints(
        3.0, 3.0,
        Math.toRadians(540), Math.toRadians(720));


    public ReefTractorBeam(DriveTrain driveTrain, boolean goLeft) {
        m_driveTrain = driveTrain;
        m_goLeft = goLeft;

        // do not Require the drivetrain - the outside command handles that
    }

    @Override
    public Command get() {  // getDriveToNearestPole
        Pose2d currentPose = FieldConstants.flipPose(m_driveTrain.getPose());
        // System.out.println("Current Pose: " + currentPose);
        // SmartDashboard.putData("Current Pose:", currentPose);

        Pose2d nearestPole = FieldConstants.flipPose(
            currentPose.nearest(m_goLeft ? FieldConstants.REEF_SCORING_LOCATIONS_LEFT : FieldConstants.REEF_SCORING_LOCATIONS_RIGHT));
        // System.out.println("Nearest Pole: " + nearestPole);

        return m_driveTrain.pathFindToPose(nearestPole, m_constraints);
    }
}
