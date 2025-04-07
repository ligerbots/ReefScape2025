package frc.robot.commands.experementalReefAutoAlign;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.FieldConstants;
import frc.robot.commands.ReefTractorBeam;
import frc.robot.subsystems.DriveTrain;

public class ReefTractorBeamDecider implements Supplier<Command> {

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

    private final boolean m_useDirectPath;


    public ReefTractorBeamDecider(DriveTrain driveTrain, boolean goLeft, BooleanSupplier hasCoral, boolean useDirectPath) {
        m_driveTrain = driveTrain;
        m_goLeft = goLeft;
        m_hasCoral = hasCoral;
        m_useDirectPath = useDirectPath; // allows us to choose between the direct path or the normal pathfinding more easly for on the fly testing
        
        // do not Require the drivetrain - the outside command handles that
    }

    @Override
    public Command get() {  // getDriveToNearestPole
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

        double distanceToGoal = destination.getTranslation().minus(currentPose.getTranslation()).getNorm();

        destination = FieldConstants.flipPose(destination); // flip back over from calculations
        if (distanceToGoal <= 0.5) {
            // use PID to get to the goal
            return new TractorBeamPID(m_driveTrain, destination).withTimeout(PATHFIND_TIMEOUT); 
        } else {
            //Use path planner methods as we are far enough away for them to work
            if (m_useDirectPath) {
                return new TractorBeamWithDirectPath(m_driveTrain, destination).get().withTimeout(PATHFIND_TIMEOUT); 
            } else {
                return new ReefTractorBeam(m_driveTrain, m_goLeft, m_hasCoral).get().withTimeout(PATHFIND_TIMEOUT); 
            }
        }
    }
}
