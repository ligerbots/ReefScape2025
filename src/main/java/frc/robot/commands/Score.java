// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffectorPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Score extends Command {
  /** Creates a new Score. */
  EndEffectorPivot m_pivot;
  Elevator m_elevator;
  boolean m_cancel;

  double m_desiredAngle;
  double m_desiredHeight;
  Constants.Position m_position;

  private static final HashMap<Position, Pair<Double, Double>> POSITIONS = new HashMap<Position, Pair<Double, Double>>(){
    {
    put(Position.L1, new Pair<>(1.0,1.0));
    put(Position.L2, new Pair<>(1.0,1.0));
    put(Position.L3, new Pair<>(1.0,1.0));
    put(Position.L4, new Pair<>(1.0,1.0));
    put(Position.BARGE, new Pair<>(1.0,1.0));
    put(Position.FRONT_INTAKE, new Pair<>(1.0,1.0));
    put(Position.BACK_INTAKE, new Pair<>(1.0,1.0));

    }
  };

  public Score(Elevator elevator, EndEffectorPivot pivot) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pivot = pivot;
    m_elevator = elevator;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

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
    return false;
  }
}
