// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralGroundIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreL1 extends Command {
  /** Creates a new scoreL1. */
  private final CoralGroundIntake m_coralGroundIntake;

  public ScoreL1(CoralGroundIntake CoralGroundIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_coralGroundIntake = CoralGroundIntake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_coralGroundIntake.goToScoreAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_coralGroundIntake.stow();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
