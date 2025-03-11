package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;

public class ReefTractorBeam implements Supplier<Command> {

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

        Pose2d nearestPole = FieldConstants.flipPose(currentPose.nearest(FieldConstants.REEF_POLES));
        System.out.println("Nearest Pole: " + nearestPole);
        return m_driveTrain.pathFindToPose(nearestPole, constraints);
    }



    
}
