// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffectorPivot;
import frc.robot.subsystems.EndEffectorWrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Score extends Command {
   EndEffectorPivot m_pivot;
   Elevator m_elevator;
   EndEffectorWrist m_wrist;
   Claw m_claw;
   Position m_robotstate;
   String m_robotStateString;
   DoubleSupplier m_height;
   double m_wantedElevatorHeight;
   Timer m_commandTimeout = new Timer();
   double m_timeoutDelay = 2;


  /** Creates a new Score. */
  public Score(Position robotState, EndEffectorPivot pivot, EndEffectorWrist wrist, Elevator elevator, Claw claw, DoubleSupplier elevatorHeight) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pivot = pivot;
    m_claw = claw;
    m_elevator = elevator;
    m_wrist = wrist;
    m_robotstate = robotState;
    m_height = elevatorHeight;


    m_robotStateString = robotState.toString();
    
    switch (m_robotStateString){
      case "L2_PREP":
        m_wantedElevatorHeight = MoveEndEffectorRedesign.L2_HEIGHT;
      case "L3_PREP":
        m_wantedElevatorHeight = MoveEndEffectorRedesign.L3_HEIGHT;
      case "L4_PREP":
        m_wantedElevatorHeight = MoveEndEffectorRedesign.L4_HEIGHT;
      
    }

  


  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (m_robotstate) {
      case L2_PREP:
        new MoveEndEffectorRedesign(Constants.Position.L2, m_elevator, m_pivot, m_wrist);
        break;
      case L3 :
        new MoveEndEffectorRedesign(Constants.Position.L3, m_elevator, m_pivot, m_wrist);
        break;
      case L4:
        new MoveEndEffectorRedesign(Constants.Position.L4, m_elevator, m_pivot, m_wrist);
        break;
      default:
        break;
    }
    m_commandTimeout.restart();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(m_height.getAsDouble()-m_wantedElevatorHeight) <= 0.5
    || m_commandTimeout.hasElapsed(m_timeoutDelay);
  }
}
