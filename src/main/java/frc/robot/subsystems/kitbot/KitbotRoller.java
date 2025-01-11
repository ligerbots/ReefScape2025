// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kitbot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class KitbotRoller extends SubsystemBase {
  /** Creates a new kitbotRoller. */

  final SparkMax m_roller;
  final SparkClosedLoopController m_rollerPID;

  public KitbotRoller() {
      m_roller = new SparkMax(1, MotorType.kBrushless);
      m_rollerPID = m_roller.getClosedLoopController();


SparkMaxConfig config = new SparkMaxConfig();

config
    .inverted(true)
    .idleMode(IdleMode.kBrake);
config.encoder
    .positionConversionFactor(1000)
    .velocityConversionFactor(1000);
config.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .pid(1.0, 0.0, 0.0);
    
    m_roller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runRollerOut() {
    m_rollerPID.setReference(.44, SparkBase.ControlType.kVelocity);

  }

  public void stopRoller() {
    m_rollerPID.setReference(0, SparkBase.ControlType.kVelocity);

  }
}
