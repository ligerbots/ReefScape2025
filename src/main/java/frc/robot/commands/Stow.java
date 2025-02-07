// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeEffector;
import frc.robot.subsystems.CoralEffector;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffectorPivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Stow extends ParallelCommandGroup {
  /** Creates a new Stow. */
  public Stow(CoralEffector coralEffector, AlgaeEffector algaeEffector, EndEffectorPivot pivot, Elevator elevator) {
    addCommands(
    new Score(elevator, pivot, Constants.Position.STOW),
    new InstantCommand(coralEffector::setHold),
    new InstantCommand(algaeEffector::setHold)
    );
  }
}
