// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeIntake extends SubsystemBase {
  private final SparkFlex m_deployPivot;
  private final SparkFlex m_roller;

  /** Creates a new AlgaeIntake. */
  public AlgaeIntake() {
    m_deployPivot = new SparkFlex(Constants.ALGAE_INTAKE_DEPLOY_PIVOT_ID, MotorType.kBrushless);
    m_roller = new SparkFlex(Constants.ALGAE_INTAKE_ROLLER_ID, MotorType.kBrushless);
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
