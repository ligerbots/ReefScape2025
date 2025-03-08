package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.FieldConstants;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffectorPivot;

public abstract class ReefscapeAbstractAuto extends AutoCommandInterface {

    protected Pose2d m_initPose;

    ReefscapeAbstractAuto(Pose2d startPoint, Pose2d sourcePoint, Pose2d[] reefPoints, DriveTrain driveTrain,
            Elevator elevator, CoralEffector coralEffector, EndEffectorPivot pivot, boolean isProcessorSide) {
    }

    @Override
    public Pose2d getInitialPose() {
        return FieldConstants.flipPose(m_initPose);
    }
}
