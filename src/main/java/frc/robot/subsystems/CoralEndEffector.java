// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralEndEffector extends SubsystemBase {
    /** Creates a new CoralEndEffector. */
    private static final int MOTOR_CURRENT_LIMIT = 30;
    private static final double MOTOR_VOLTAGE_COMP = 10; // This sets a limit for voltage to 10 so it is repeatable
                                                         // untill the battery dips below 10 volts

    // Speeds
    // TODO: set these to real speeds
    private static final double INTAKE_SPEED = -1;
    private static final double OUTTAKE_SPEED = 1;
    private static final double HOLD_SPEED = -0.05;

    // Motor
    private final SparkMax m_motor;

    // Limit Switch
    private final SparkLimitSwitch m_limitSwitch;

    // State
    private enum State {
        IDLE, INTAKE, OUTTAKE, HOLD;
    }

    private State m_state = State.IDLE;

    public CoralEndEffector() {
        // Set up the coral motor as brushed motors
        m_motor = new SparkMax(Constants.END_EFFECTOR_CORAL_INTAKE_ID, MotorType.kBrushless);
        m_limitSwitch = m_motor.getReverseLimitSwitch();

        // Set can timeout. Because this project only sets parameters once on
        // construction, the timeout can be long without blocking robot operation. Code
        // which sets or gets parameters during operation may need a shorter timeout.
        m_motor.setCANTimeout(250);

        // Create and apply configuration for roller motor. Voltage compensation help
        // the roller behave the same as the battery
        // voltage dips. The current limit helps prevent breaker trips or burning out
        // the motor in the event the roller stalls.
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(false);
        config.voltageCompensation(MOTOR_VOLTAGE_COMP);
        config.smartCurrentLimit(MOTOR_CURRENT_LIMIT);

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("endEffector/coralLimitSwitch", m_limitSwitch.isPressed());
        switch (m_state) {
            case IDLE:
                m_motor.set(0);
                break;
            case INTAKE:
                m_motor.set(INTAKE_SPEED);
                if (m_limitSwitch.isPressed()) {
                    m_state = State.HOLD;
                }
                break;
            case OUTTAKE:
                m_motor.set(OUTTAKE_SPEED);
                break;
            case HOLD:
                m_motor.set(HOLD_SPEED);
                break;
        }
    }

    public void runIntake() {
        m_state = State.INTAKE;
    }

    public void runOuttake() {
        m_state = State.OUTTAKE;
    }

    public void stop() {
        m_state = State.IDLE;
    }
}
