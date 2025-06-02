package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.subsystems.AlgaeEffector;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffectorPivot;

public abstract class ReefscapeAbstractAuto extends AutoCommandInterface {

    protected static final double CORAL_SCORE_WAIT_TIME = 0.1;
    public static final double RAISE_ELEVATOR_WAIT_TIME = 1.8;
    protected static final double LOWER_ELEVATOR_WAIT_TIME = 0.5;  // maybe can be lower
    protected static final double RAISE_ELEVATOR_AFTER_PATH_START = 1.4;
    protected static final double START_INTAKE_AFTER_PATH_START = 0.5;

    protected Pose2d m_initPose;
    protected Pose2d m_sourcePoint;
    protected DriveTrain m_driveTrain;
    protected Elevator m_elevator;
    protected CoralEffector m_coralEffector;
    protected AlgaeEffector m_algaeEffector;
    protected EndEffectorPivot m_pivot;
    protected boolean m_isProcessorSide;

    ReefscapeAbstractAuto(Pose2d startPoint, Pose2d sourcePoint, Pose2d[] reefPoints, DriveTrain driveTrain,
            Elevator elevator, CoralEffector coralEffector, AlgaeEffector algaeEffector, EndEffectorPivot pivot, boolean isProcessorSide) {
                m_sourcePoint = sourcePoint;
                m_driveTrain = driveTrain;
                m_elevator = elevator;
                m_coralEffector = coralEffector;
                m_algaeEffector = algaeEffector;
                m_pivot = pivot;
                m_isProcessorSide = isProcessorSide;
    }

    @Override
    public Pose2d getInitialPose() {
        return FieldConstants.flipPose(m_initPose);
    }

    public Command raiseToL4ScorePosition() {
        return new MoveEndEffector(Constants.Position.L4, m_elevator, m_pivot, RAISE_ELEVATOR_WAIT_TIME);
    }
}
