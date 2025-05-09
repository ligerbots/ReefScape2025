package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;

public class ReefTractorBeam implements Supplier<Command> {

    private static final HashMap<Pose2d, Pair<Pose2d, Pose2d>> REEF_POSITIONS = new HashMap<Pose2d, Pair<Pose2d, Pose2d>>()
    {
        {
            // Pair will be <Left, Right>
            // Note that left/right is from the *Driver's* perspective
            put(FieldConstants.REEF_ALGAE_AB, new Pair<>(FieldConstants.REEF_A, FieldConstants.REEF_B));
            put(FieldConstants.REEF_ALGAE_CD, new Pair<>(FieldConstants.REEF_C, FieldConstants.REEF_D));
            put(FieldConstants.REEF_ALGAE_EF, new Pair<>(FieldConstants.REEF_E, FieldConstants.REEF_F));
            put(FieldConstants.REEF_ALGAE_GH, new Pair<>(FieldConstants.REEF_H, FieldConstants.REEF_G));
            put(FieldConstants.REEF_ALGAE_IJ, new Pair<>(FieldConstants.REEF_I, FieldConstants.REEF_J));
            put(FieldConstants.REEF_ALGAE_KL, new Pair<>(FieldConstants.REEF_K, FieldConstants.REEF_L));
        }
    };    
    // create this once, for efficiency
    private static final List<Pose2d> REEF_ALGAE_POSES = new ArrayList<>(REEF_POSITIONS.keySet());

    private final DriveTrain m_driveTrain;
    private final boolean m_goLeft;
    private final BooleanSupplier m_hasCoral;
    private static final double PATHFIND_TIMEOUT = 2.0;

    private final PathConstraints m_constraints =  new PathConstraints(
        3.0, 2.5,
        Math.toRadians(540), Math.toRadians(720));


    public ReefTractorBeam(DriveTrain driveTrain, boolean goLeft, BooleanSupplier hasCoral) {
        m_driveTrain = driveTrain;
        m_goLeft = goLeft;
        m_hasCoral = hasCoral;

        // do not Require the drivetrain - the outside command handles that
    }

    @Override
    public Command get() {  // getDriveToNearestPole
        Pose2d currentPose = FieldConstants.flipPose(m_driveTrain.getPose());
        // System.out.println("Current Pose: " + currentPose);
        // SmartDashboard.putData("Current Pose:", currentPose);

        Pose2d nearestAlgae = currentPose.nearest(REEF_ALGAE_POSES);

        Pose2d destination;
        if (!m_hasCoral.getAsBoolean()) {
            // not holding a Coral. Go to the Algae position
            destination = nearestAlgae;
        } else {
            Pair<Pose2d, Pose2d> coralLeftRight = REEF_POSITIONS.get(nearestAlgae);
            destination = m_goLeft ? coralLeftRight.getFirst() : coralLeftRight.getSecond();
        }

        return m_driveTrain.pathFindToPose(FieldConstants.flipPose(destination), m_constraints).withTimeout(PATHFIND_TIMEOUT);
    }
}
