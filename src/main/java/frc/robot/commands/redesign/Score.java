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

  private Timer m_commandTimeout = new Timer();
  private double m_timeoutDelay = 2;

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

      Command moveCommand;

      // Select the move command based on the position
      switch (m_robotState) {
          case L2_PREP:
              moveCommand = new MoveEndEffectorRedesign(Constants.Position.L2, m_elevator, m_pivot, m_wrist);
              break;
          case L3_PREP:
              moveCommand = new MoveEndEffectorRedesign(Constants.Position.L3, m_elevator, m_pivot, m_wrist);
              break;
          case L4_PREP:
              moveCommand = new MoveEndEffectorRedesign(Constants.Position.L4, m_elevator, m_pivot, m_wrist);
              break;
          default:
              moveCommand = new MoveEndEffectorRedesign(m_robotState, m_elevator, m_pivot, m_wrist);
              break;
      }

      // Schedule the move command
      moveCommand.schedule();
      m_commandTimeout.restart();
  }

  @Override
  public void execute() {
      // Execution logic if needed
  }

  @Override
  public void end(boolean interrupted) {
      // Handle cleanup if needed
  }

  @Override
  public boolean isFinished() {
      return m_elevator.lengthWithinTolerance() && m_pivot.angleWithinTolerance() && m_wrist.angleWithinTolerance()
              || m_commandTimeout.hasElapsed(m_timeoutDelay);
  }
}
