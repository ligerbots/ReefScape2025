// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase {
  private final SparkFlex m_deployPivot;
  private RelativeEncoder m_deployPivotEncoder;
  private final double DEPLOY_PIVOT_OUT_SPEED = 1; // TODO
  private final double DEPLOY_PIVOT_IN_SPEED = -1; // TODO
  private final double DEPLOY_PIVOT_END_POSITION = 10; // TODO

  private final SparkFlex m_roller;
  private final double ROLLER_INTAKE_SPEED = 1; // TODO
  private final double ROLLER_OUTAKE_SPEED = -1; // TODO
  private final double ROLLER_HOLD_SPEED = 0.1; // TODO

  private Timer m_timer = new Timer();
  private final double INTAKE_SECONDS = 3; // TODO
  private final double OUTTAKE_SECONDS = 3; // TODO

  private enum Task {
    INTAKE, OUTTAKE, IDLE;
  }

  private Task m_task = Task.IDLE;

  private enum DeployState {
    DEPLOY_OUT, DEPLOY_IN, WAIT, IDLE;
  }

  private DeployState m_deployState = DeployState.IDLE;

  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {
    m_deployPivot = new SparkFlex(Constants.ALGAE_INTAKE_DEPLOY_PIVOT_ID, MotorType.kBrushless);
    m_deployPivotEncoder = m_deployPivot.getEncoder();
    m_deployPivotEncoder.setPosition(0);

    m_roller = new SparkFlex(Constants.ALGAE_INTAKE_ROLLER_ID, MotorType.kBrushless);

    m_timer.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // TODO put stuff into SmartDashboard

    double deployPosition = m_deployPivotEncoder.getPosition();

    // Deploy pivot state machine
    switch (m_deployState) {
      case DEPLOY_OUT:
        deployPivotOut();
        if (deployPosition >= DEPLOY_PIVOT_END_POSITION) {
          m_deployState = DeployState.WAIT;

          // start wait timer
          m_timer.reset();
          m_timer.start();
        }
        break;
      case DEPLOY_IN:
        deployPivotIn();
        if (deployPosition <= 0) {
          // we have finished out task of either intaking or outtaking
          m_deployState = DeployState.IDLE;
          m_task = Task.IDLE;
        }
        break;
      case WAIT:
        if ((m_task == Task.INTAKE && m_timer.hasElapsed(INTAKE_SECONDS)) ||
            (m_task == Task.OUTTAKE && m_timer.hasElapsed(OUTTAKE_SECONDS))) {
          m_deployState = DeployState.DEPLOY_IN;
        }
        break;
      case IDLE:
        stopDeployPivot();
        break;
    }
  }
  
  private void deployPivotOut() {
    m_deployPivot.set(DEPLOY_PIVOT_OUT_SPEED);
  }
  private void deployPivotIn() {
    m_deployPivot.set(DEPLOY_PIVOT_IN_SPEED);
  }
  private void stopDeployPivot() {
    m_deployPivot.set(0);
  }
  private void rollerIntake() {
    m_roller.set(ROLLER_INTAKE_SPEED);
  }
  private void rollerOuttake() {
    m_roller.set(ROLLER_OUTAKE_SPEED);
  }
  private void rollerHold() {
    m_roller.set(ROLLER_HOLD_SPEED);
  }
  private void stopRoller() {
    m_roller.set(0);
  }
}
