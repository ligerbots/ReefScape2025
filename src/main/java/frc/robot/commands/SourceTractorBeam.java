package frc.robot.commands;

import java.util.List;
import java.util.function.Supplier;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FieldConstants;
import frc.robot.subsystems.DriveTrain;

public class SourceTractorBeam implements Supplier<Command> {
    private DriveTrain m_driveTrain;

    public SourceTractorBeam(DriveTrain driveTrain) {
        m_driveTrain = driveTrain;
        // addRequirements(driveTrain);
        // addCommands(driveToNearestPole(driveTrain));
    }

    @Override
    public Command get() {  // getDriveToNearestPole
        PathConstraints constraints =  PathConstraints.unlimitedConstraints(12);
        return m_driveTrain.pathFindToPose(m_driveTrain.getLikelyPickupLocation(), constraints);
    }   
}
