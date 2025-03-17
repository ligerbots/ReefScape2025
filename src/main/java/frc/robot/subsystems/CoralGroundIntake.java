// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralGroundIntake extends SubsystemBase {
  /** Creates a new CoralGroundIntake. */
  private final SparkFlex m_coralGroundRoller;
  private static final int CURRENT_LIMIT = 60;

  public CoralGroundIntake() {
    
  }

  @Override
  public void periodic() {
    // NOTE: 1 neo with gear ratio and one vortex

    // This method will be called once per scheduler run
  }
}
