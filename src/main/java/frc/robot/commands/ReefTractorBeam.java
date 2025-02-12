package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;

public class ReefTractorBeam implements Supplier<Command> {

    private static final List<Pose2d> REEF_POLES = List.of( FieldConstants.REEF_A, FieldConstants.REEF_B, FieldConstants.REEF_C, 
                                                            FieldConstants.REEF_D, FieldConstants.REEF_E, FieldConstants.REEF_F, 
                                                            FieldConstants.REEF_G, FieldConstants.REEF_H, FieldConstants.REEF_I, 
                                                            FieldConstants.REEF_J, FieldConstants.REEF_K, FieldConstants.REEF_L);

    private DriveTrain m_driveTrain;

    public ReefTractorBeam(DriveTrain driveTrain) {
        m_driveTrain = driveTrain;
        // addRequirements(driveTrain);
        // addCommands(driveToNearestPole(driveTrain));
    }
    @Override
    public Command get() {  // getDriveToNearestPole
        PathConstraints constraints =  PathConstraints.unlimitedConstraints(12);

        Pose2d currentPose = FieldConstants.flipPose(m_driveTrain.getPose());
        System.out.println("Current Pose: " + currentPose);
        // SmartDashboard.putData("Current Pose:", currentPose);

        Pose2d nearestPole = FieldConstants.flipPose(currentPose.nearest(REEF_POLES));
        System.out.println("Nearest Pole: " + nearestPole);
        return m_driveTrain.pathFindToPose(nearestPole, constraints);
    }



    
}
