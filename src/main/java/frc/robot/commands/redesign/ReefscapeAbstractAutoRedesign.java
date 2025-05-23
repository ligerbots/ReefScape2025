package frc.robot.commands.redesign;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.commands.AutoCommandInterface;
import frc.robot.subsystems.AlgaeEffector;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.CoralGroundIntakeRedesign;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffectorPivot;
import frc.robot.subsystems.EndEffectorWrist;

public abstract class ReefscapeAbstractAutoRedesign extends AutoCommandInterface {

    protected static final double CORAL_SCORE_WAIT_TIME = 0.1;
    public static final double RAISE_ELEVATOR_WAIT_TIME = 1.8;
    protected static final double LOWER_ELEVATOR_WAIT_TIME = 0.5;  // maybe can be lower
    protected static final double RAISE_ELEVATOR_AFTER_PATH_START = 0.7;
    protected static final double START_INTAKE_AFTER_PATH_START = 0.5;

    protected Pose2d m_initPose;
    protected Pose2d m_sourcePoint;
    protected DriveTrain m_driveTrain;
    protected Elevator m_elevator;
    protected EndEffectorPivot m_pivot;
    protected EndEffectorWrist m_wrist;
    protected Claw m_claw;
    protected CoralGroundIntakeRedesign  m_coralGround;
    protected boolean m_isProcessorSide;

    ReefscapeAbstractAutoRedesign(Pose2d startPoint, Pose2d sourcePoint, Pose2d[] reefPoints, DriveTrain driveTrain,
            Elevator elevator, Claw claw, EndEffectorWrist wrist, EndEffectorPivot pivot, CoralGroundIntakeRedesign coralGround, boolean isProcessorSide) {
                m_sourcePoint = sourcePoint;
                m_driveTrain = driveTrain;
                m_elevator = elevator;
                m_claw = claw;
                m_wrist = wrist;
                m_pivot = pivot;
                m_coralGround = coralGround;
                m_isProcessorSide = isProcessorSide;
    }

    @Override
    public Pose2d getInitialPose() {
        return FieldConstants.flipPose(m_initPose);
    }

    public Command prepForL4Command() {
        return new MoveEndEffectorRedesign(Constants.Position.L4_PREP, m_elevator, m_pivot, m_wrist);
    }
}
