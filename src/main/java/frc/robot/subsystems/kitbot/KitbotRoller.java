// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.kitbot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class KitbotRoller extends SubsystemBase {
  public static final int MOTOR_CURRENT_LIMIT = 20;
  public static final double MOTOR_VOLTAGE_COMP = 10; //This sets a limit for voltage to 10 so it is repeatable untill the battery dips below 10 volts
  public static final double EJECT_VALUE = 0.44;

  private final SparkMax rollerMotor;

  public KitbotRoller() {
    // Set up the roller motor as a brushed motor
    rollerMotor = new SparkMax(Constants.KITBOT_ROLLER_ID, MotorType.kBrushed);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.
    rollerMotor.setCANTimeout(250);

    // Create and apply configuration for roller motor. Voltage compensation helps
    // the roller behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the roller stalls.
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.voltageCompensation(MOTOR_VOLTAGE_COMP);
    rollerConfig.smartCurrentLimit(MOTOR_CURRENT_LIMIT);
    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void runRollerOut() {
    rollerMotor.set(EJECT_VALUE);
  }

  public void stopRoller() {
    rollerMotor.stopMotor();
  }
}
