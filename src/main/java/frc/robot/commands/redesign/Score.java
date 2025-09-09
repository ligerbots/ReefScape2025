package frc.robot.commands.redesign;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffectorPivot;
import frc.robot.subsystems.EndEffectorWrist;

public class Score extends Command {
    private EndEffectorPivot m_pivot;
    private EndEffectorWrist m_wrist;
    private Elevator m_elevator;
    private Claw m_claw;
    private Supplier<Position> m_robotStateSupplier;  // Store the Supplier<Position>
    private Position m_robotState;  // Actual robot state value
    private boolean L1Mode = false;
    
    private Timer m_commandTimeout = new Timer();
    private static final double L1_TIMEOUT = 0.5;
    
    private Command m_moveCommand;

    // Constructor expects a Supplier<Position> instead of Position directly
    public Score(Supplier<Position> robotState, EndEffectorPivot pivot, EndEffectorWrist wrist, Elevator elevator, Claw claw) {
        m_pivot = pivot;
        m_wrist = wrist;
        m_elevator = elevator;
        m_claw = claw;
        m_robotStateSupplier = robotState;  // Store the Supplier<Position>
    }
    
    @Override
    public void initialize() {
        // Access the value from the Supplier to get the current robot state
        m_robotState = m_robotStateSupplier.get();
                
        // Select the move command based on the position
        switch (m_robotState) {
            case L2_PREP:
                m_moveCommand = new MoveEndEffectorRedesign(Constants.Position.L2, m_elevator, m_pivot, m_wrist);
                break;
            case L3_PREP:
                m_moveCommand = new MoveEndEffectorRedesign(Constants.Position.L3, m_elevator, m_pivot, m_wrist);
                break;
            case L4_PREP:
                m_moveCommand = new MoveEndEffectorRedesign(Constants.Position.L4, m_elevator, m_pivot, m_wrist);
                break;
            case L2_PREP_ALT:
                m_moveCommand = new MoveEndEffectorRedesign(Constants.Position.L2_ALT, m_elevator, m_pivot, m_wrist);
                break;
            case L3_PREP_ALT:
                m_moveCommand = new MoveEndEffectorRedesign(Constants.Position.L3_ALT, m_elevator, m_pivot, m_wrist);
                break;
            case L4_PREP_ALT:
                m_moveCommand = new MoveEndEffectorRedesign(Constants.Position.L4_ALT, m_elevator, m_pivot, m_wrist);
                break;
            case L1:
                m_moveCommand = new MoveEndEffectorRedesign(Constants.Position.L1, m_elevator, m_pivot, m_wrist);
                m_claw.runOuttake();
                L1Mode = true;
            case L1_ALT:
                m_moveCommand = new MoveEndEffectorRedesign(Constants.Position.L1_ALT, m_elevator, m_pivot, m_wrist);
                m_claw.runOuttake();
                L1Mode = true;
            default:
                m_moveCommand = new MoveEndEffectorRedesign(m_robotState, m_elevator, m_pivot, m_wrist);
                break;
        }
        
        // We can't actually schedule the move command
        //  that kills auto because of conflicting Requirements
        // Run move command ourselves
        m_moveCommand.initialize();

        m_commandTimeout.restart();
    }
    
    @Override
    public void execute() {
        m_moveCommand.execute();
    }
    
    @Override
    public void end(boolean interrupted) {
        m_moveCommand.end(interrupted);
        m_claw.stop();
    }
    
    @Override
    public boolean isFinished() {
        if (L1Mode) {
            return m_commandTimeout.hasElapsed(L1_TIMEOUT);
        } 

        return m_moveCommand.isFinished();
    }
}
