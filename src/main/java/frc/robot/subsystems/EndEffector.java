// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {
  private final SparkMax m_pivot;
  private final SparkMax m_coralIntake;
  private final SparkMax m_algaeIntake;

  static final int MOTOR_CURRENT_LIMIT = 20; // TODO: this might be wrong
  static final double MOTOR_VOLTAGE_COMP = 10;

  /** Creates a new EndEffector. */
  public EndEffector() {
    // Initialize motors
    m_pivot = new SparkMax(Constants.END_EFFECTOR_PIVOT_ID, MotorType.kBrushed);
    m_coralIntake = new SparkMax(Constants.END_EFFECTOR_CORAL_INTAKE_ID, MotorType.kBrushed);
    m_algaeIntake = new SparkMax(Constants.END_EFFECTOR_ALGAE_INTAKE_ID, MotorType.kBrushed);

    initializeMotor(m_algaeIntake);
    initializeMotor(m_coralIntake);
  }

  private void initializeMotor(SparkMax motor) {
    motor.setCANTimeout(250);

    // Create and apply configuration for roller motor. Voltage compensation helps
    // the roller behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the roller stalls.
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.voltageCompensation(MOTOR_VOLTAGE_COMP);
    rollerConfig.smartCurrentLimit(MOTOR_CURRENT_LIMIT);
    rollerConfig.idleMode(IdleMode.kCoast);
    motor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
