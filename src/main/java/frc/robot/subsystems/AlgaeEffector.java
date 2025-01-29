// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class AlgaeEffector extends SubsystemBase {
    static final int MOTOR_CURRENT_LIMIT = 30;
    static final double MOTOR_VOLTAGE_COMP = 12; //This sets a limit for voltage to 10 so it is repeatable untill the battery dips below 10 volts
    static final double EJECT_VALUE = 1;
    static final double INTAKE_VALUE = -0.5;
    
    private final SparkMax m_algaeMotor;
    
    public AlgaeEffector() {
        // Set up the coral and algae motor as brushed motors
        m_algaeMotor = new SparkMax(Constants.END_EFFECTOR_ALGAE_INTAKE_ID, MotorType.kBrushless);

        // Set can timeout. Because this project only sets parameters once on
        // construction, the timeout can be long without blocking robot operation. Code
        // which sets or gets parameters during operation may need a shorter timeout.
        m_algaeMotor.setCANTimeout(250);
        
        // Create and apply configuration for roller motor. Voltage compensation help
        // the roller behave the same as the battery
        // voltage dips. The current limit helps prevent breaker trips or burning out
        // the motor in the event the roller stalls.
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false);
        config.voltageCompensation(MOTOR_VOLTAGE_COMP);
        config.smartCurrentLimit(MOTOR_CURRENT_LIMIT);
        
        m_algaeMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    

    
    public void ejectAlgae() {
        // System.out.print("command scheduled");
        m_algaeMotor.set(EJECT_VALUE);
    }
    
    public void intakeAlgae() {
        m_algaeMotor.set(INTAKE_VALUE);
    }

    public void stop() {
        m_algaeMotor.stopMotor();
    }
}
