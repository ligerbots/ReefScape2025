// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.Position;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffectorPivot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveEndEffector extends Command {
  /** Creates a new Score. */
  EndEffectorPivot m_pivot;
  Elevator m_elevator;
  boolean m_cancel;

  double m_desiredAngle;
  double m_desiredHeight;
  Constants.Position m_position;

  public static final double L1_ANGLE = 1.0;
  public static final double L1_HEIGHT = 1.0;
  public static final double L2_ANGLE = 1.0;
  public static final double L2_HEIGHT = 1.0;
  public static final double L3_ANGLE = 1.0;
  public static final double L3_HEIGHT = 1.0;
  public static final double L4_ANGLE = 1.0;
  public static final double L4_HEIGHT = 1.0;
  
  public static final double STOW_ANGLE = 1.0;
  public static final double STOW_HEIGHT = 1.0;

  public static final double BARGE_HEIGHT = 1.0;
  public static final double BARGE_ANGLE = 1.0;

  public static final double FRONT_INTAKE_HEIGHT = 1.0;
  public static final double FRONT_INTAKE_ANGLE = 1.0;
  public static final double BACK_INTAKE_HEIGHT = 1.0;
  public static final double BACK_INTAKE_ANGLE = 1.0;

  public static final double L1_ALGAE_HEIGHT= 1.0;
  public static final double L1_ALGAE_ANGLE = 1.0;

    public static final double L2_ALGAE_HEIGHT= 1.0;
  public static final double L2_ALGAE_ANGLE = 1.0;






  private static final HashMap<Position, Pair<Double, Double>> POSITIONS = new HashMap<Position, Pair<Double, Double>>(){
    {
    put(Position.L1, new Pair<>(L1_HEIGHT,L1_ANGLE));
    put(Position.L2, new Pair<>(L2_HEIGHT,L2_ANGLE));
    put(Position.L3, new Pair<>(L3_HEIGHT,L3_ANGLE));
    put(Position.L4, new Pair<>(L4_HEIGHT,L4_ANGLE));
    put(Position.BARGE, new Pair<>(BARGE_HEIGHT,BARGE_ANGLE));
    put(Position.FRONT_INTAKE, new Pair<>(FRONT_INTAKE_HEIGHT,FRONT_INTAKE_ANGLE));
    put(Position.BACK_INTAKE, new Pair<>(BACK_INTAKE_HEIGHT,BACK_INTAKE_ANGLE));
    put(Position.L1_ALGAE, new Pair<>(L1_ALGAE_HEIGHT, L1_ALGAE_ANGLE));
    put(Position.L2_ALGAE, new Pair<>(L2_ALGAE_HEIGHT, L2_ALGAE_ANGLE));
    put(Position.STOW, new Pair<>(STOW_HEIGHT,STOW_ANGLE));
    }
  };

  public MoveEndEffector(Elevator elevator, EndEffectorPivot pivot, Constants.Position position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_pivot = pivot;
    m_elevator = elevator;
    m_position = position;
    Pair<Double, Double> desiredPos = POSITIONS.get(position);

    m_desiredHeight = desiredPos.getFirst();
    m_desiredAngle = desiredPos.getSecond();

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.setHeight(m_desiredHeight);
    m_pivot.setAngle(Rotation2d.fromDegrees(m_desiredAngle));
    
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
    
    if(m_elevator.lengthWithinTolerance() && m_pivot.angleWithinTolerance()){
      return true;
    }
    else{
    return false;
    }
  }
}
