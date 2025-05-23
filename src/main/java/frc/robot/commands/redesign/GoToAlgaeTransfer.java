// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.redesign;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeGroundIntakeRedesign;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.CoralGroundIntakeRedesign;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.EndEffectorPivot;
import frc.robot.subsystems.EndEffectorWrist;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GoToAlgaeTransfer extends SequentialCommandGroup {
  /** Creates a new AlgaeTransfer. */
   EndEffectorPivot m_pivot;
  Elevator m_elevator;
  EndEffectorWrist m_wrist;
  Claw m_claw;
  CoralGroundIntakeRedesign m_coralGround;
  AlgaeGroundIntakeRedesign m_algaeGround;
  DoubleSupplier m_height;
  Timer m_commandTimeout = new Timer();
  double m_timeoutDelay = 2;
  double transferTime = 0.5;
  double coralGroundMoveTime = .2;
  public GoToAlgaeTransfer(EndEffectorPivot pivot, EndEffectorWrist wrist, Elevator elevator, Claw claw, CoralGroundIntakeRedesign coralGround, AlgaeGroundIntakeRedesign algaeGround ) {
    // Add your commands in the addCommands() call, e.g.
    m_pivot = pivot;
    m_claw = claw;
    m_elevator = elevator;
    m_wrist = wrist;
    m_coralGround = coralGround;
    m_algaeGround = algaeGround;
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(m_coralGround::ALGAE_TRANSFER),
      new WaitCommand(coralGroundMoveTime),
      new MoveEndEffectorRedesign(Constants.Position.ALGAE_TRANSFER, elevator, pivot, wrist).alongWith(new InstantCommand(m_algaeGround::Transfer)).alongWith(new InstantCommand(claw::runIntake))
    );
  }
}
