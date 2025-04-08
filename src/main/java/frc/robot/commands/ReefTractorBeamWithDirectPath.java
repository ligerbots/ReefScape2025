package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;

public class ReefTractorBeamWithDirectPath implements Supplier<Command> {

    private static final HashMap<Pose2d, Pair<Pose2d, Pose2d>> REEF_POSITIONS = new HashMap<Pose2d, Pair<Pose2d, Pose2d>>()
    {
        {
            // Pair will be <Left, Right>
            // Note that left/right is from the *Driver's* perspective
            put(FieldConstants.REEF_ALGAE_AB, new Pair<>(FieldConstants.REEF_A, FieldConstants.REEF_B));
            put(FieldConstants.REEF_ALGAE_CD, new Pair<>(FieldConstants.REEF_C, FieldConstants.REEF_D));
            put(FieldConstants.REEF_ALGAE_EF, new Pair<>(FieldConstants.REEF_F, FieldConstants.REEF_E));
            put(FieldConstants.REEF_ALGAE_GH, new Pair<>(FieldConstants.REEF_H, FieldConstants.REEF_G));
            put(FieldConstants.REEF_ALGAE_IJ, new Pair<>(FieldConstants.REEF_J, FieldConstants.REEF_I));
            put(FieldConstants.REEF_ALGAE_KL, new Pair<>(FieldConstants.REEF_K, FieldConstants.REEF_L));
        }
    };    
    // create this once, for efficiency
    private static final List<Pose2d> REEF_ALGAE_POSES = new ArrayList<>(REEF_POSITIONS.keySet());

    private final DriveTrain m_driveTrain;
    private final boolean m_goLeft;
    private final BooleanSupplier m_hasCoral;
    private static final double PATHFIND_TIMEOUT = 2.0;

    private final PathConstraints m_constraints = new PathConstraints(
            3.0, 2.5,
            Math.toRadians(540), Math.toRadians(720));

    public ReefTractorBeamWithDirectPath(DriveTrain driveTrain, boolean goLeft, BooleanSupplier hasCoral) {
        m_driveTrain = driveTrain;
        m_goLeft = goLeft;
        m_hasCoral = hasCoral;
        
        // do not Require the drivetrain - the outside command handles that
    }

    @Override
    public Command get() {
        return getPathPlannerCommand(getTargetPose()).withTimeout(PATHFIND_TIMEOUT); 
    }

    private Pose2d getTargetPose() {
        Pose2d currentPose = FieldConstants.flipPose(m_driveTrain.getPose());

        Pose2d nearestAlgae = currentPose.nearest(REEF_ALGAE_POSES);

        Pose2d destination;
        if (!m_hasCoral.getAsBoolean()) {
            // not holding a Coral. Go to the Algae position
            destination = nearestAlgae;
        } else {
            Pair<Pose2d, Pose2d> coralLeftRight = REEF_POSITIONS.get(nearestAlgae);
            destination = m_goLeft ? coralLeftRight.getFirst() : coralLeftRight.getSecond();
        }

        destination = FieldConstants.flipPose(destination); // flip back over from calculations
        return destination;
    }

    private Command getPathPlannerCommand(Pose2d goalPose) {

        Pose2d currentPose = m_driveTrain.getPose();

        Translation2d fieldCentricRelativePose = goalPose.getTranslation().minus(currentPose.getTranslation());
        Rotation2d angleToGoal = new Rotation2d(fieldCentricRelativePose.getX(), fieldCentricRelativePose.getY());
        // The rotation component of the pose should be the direction of travel. Do not use holonomic(field centric) rotation.
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(currentPose.getX(), currentPose.getY(), angleToGoal),
                new Pose2d(goalPose.getX(), goalPose.getY(), angleToGoal)); // Start pose, then end pose

        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
                waypoints,
                m_constraints,
                null,
                // Goal end state. You can set a holonomic rotation here. If using a differential drive>train, the rotation will have no effect.
                new GoalEndState(0.0, goalPose.getRotation()) 
        );

        // Prevent the path from being flipped if the coordinates are already correct
        path.preventFlipping = true;

        return m_driveTrain.followPath(path);
    }
}
