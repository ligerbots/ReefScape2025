package frc.robot.commands.experementalReefAutoAlign;

import java.util.List;

import java.util.function.Supplier;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.DriveTrain;

public class TractorBeamWithDirectPath implements Supplier<Command> {
    private final DriveTrain m_driveTrain;

    private final Pose2d m_goalPose;

    private final PathConstraints m_constraints = new PathConstraints(
            3.0, 2.5,
            Math.toRadians(540), Math.toRadians(720));

    public TractorBeamWithDirectPath(DriveTrain driveTrain, Pose2d goalPose2d) {
        m_driveTrain = driveTrain;
        m_goalPose = goalPose2d;

        // do not Require the drivetrain - the outside commands handles that
    }

    @Override
    public Command get() { // getDriveToNearestPole
        Pose2d currentPose = m_driveTrain.getPose();

        Translation2d fieldCentricRelativePose = m_goalPose.getTranslation().minus(currentPose.getTranslation());
        Rotation2d angleToGoal = new Rotation2d(fieldCentricRelativePose.getX(), fieldCentricRelativePose.getY());
        // The rotation component of the pose should be the direction of travel. Do not
        // use holonomic(field centric) rotation.
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(currentPose.getX(), currentPose.getY(), angleToGoal),
                new Pose2d(m_goalPose.getX(), m_goalPose.getY(), angleToGoal)); // Start pose, then end pose

        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                m_constraints,
                null, 
                // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
                new GoalEndState(0.0, m_goalPose.getRotation()) 
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;

        return m_driveTrain.followPath(path);
    }
}
