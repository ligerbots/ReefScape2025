// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralGroundIntake;
import frc.robot.subsystems.CoralGroundIntake.CoralGroundIntakeState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralGround extends Command {
  /** Creates a new scoreL1. */
  private final CoralGroundIntake m_coralGroundIntake;
  private boolean m_finished = false;
  private final Timer m_timer = new Timer();

  public CoralGround(CoralGroundIntake CoralGroundIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_coralGroundIntake = CoralGroundIntake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_coralGroundIntake.m_state == CoralGroundIntakeState.SCORE_ANGLE) {
      m_coralGroundIntake.score();
      m_timer.reset();
      m_timer.start();
    } else if (m_coralGroundIntake.m_state == CoralGroundIntakeState.DEPLOY) {
      m_coralGroundIntake.stow();
    } else if (m_coralGroundIntake.m_state == CoralGroundIntakeState.STOW) {
      m_coralGroundIntake.deploy();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Score, intake again incase the first time did not work & try again.
    if (!m_timer.isRunning()) {
      m_finished = true;
    } else if (m_timer.get() > .5) {
      m_coralGroundIntake.goToScoreAngle();
    } else if (m_timer.get() > 1) {
      m_coralGroundIntake.score();
    } else if (m_timer.get() > 1.5) {
      m_coralGroundIntake.stow();
      m_finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
