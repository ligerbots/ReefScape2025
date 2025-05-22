// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CoralGroundIntakeRedesign;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffectorPivot;
import frc.robot.subsystems.EndEffectorWrist;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Transfer extends SequentialCommandGroup {
  /** Creates a new Transfer. */
  EndEffectorPivot m_pivot;
  Elevator m_elevator;
  EndEffectorWrist m_wrist;
  Claw m_claw;
  CoralGroundIntakeRedesign m_coralGround;
  DoubleSupplier m_height;
  Timer m_commandTimeout = new Timer();
  double m_timeoutDelay = 2;
  double transferTime = 0.5;

  public Transfer(EndEffectorPivot pivot, EndEffectorWrist wrist, Elevator elevator, Claw claw, DoubleSupplier elevatorHeight, CoralGroundIntakeRedesign coralGround) {
    m_pivot = pivot;
    m_claw = claw;
    m_elevator = elevator;
    m_wrist = wrist;
    m_height = elevatorHeight;
    m_coralGround = coralGround;

    addCommands(
      new InstantCommand(m_coralGround::goToTransferPose).alongWith(new MoveEndEffectorRedesign(Constants.Position.TRANSFER, elevator, pivot, wrist)),
      new InstantCommand(m_claw::runIntake),
      new WaitCommand(.1),
      new InstantCommand(m_coralGround::TransferCoral),
      new WaitCommand(transferTime),
      new InstantCommand(m_coralGround::stow).alongWith(new InstantCommand(m_claw::stop)).alongWith(new MoveEndEffectorRedesign(Constants.Position.STOW, elevator, pivot, wrist))
    );

  };
}
