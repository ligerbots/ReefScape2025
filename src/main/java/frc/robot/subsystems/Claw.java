// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.redesign.*;
import frc.robot.subsystems.ValueThreshold.Direction;

public class Claw extends SubsystemBase {
    /** Creates a new CoralEndEffector. */
    private static final int MOTOR_CURRENT_LIMIT = 60;

    // This sets a limit for voltage to 10 so it is repeatable
    // until the battery dips below 10 volts
    private static final double MOTOR_VOLTAGE_COMP = 10;

    // Speeds
    private static final double INTAKE_SPEED = -0.4;
    private static final double OUTTAKE_SPEED = 0.7;
    private static final double HOLD_SPEED = -0.05;
    private static final double OUTTAKE_L1_SPEED = 0.15;

    // Max velocity indicating the motor has stalled
    private final static double STALL_VELOCITY_LIMIT = 2000;

    // Motor
    private final TalonFX m_motor;


    private final ValueThreshold m_speedThres = new ValueThreshold(Direction.FALLING, STALL_VELOCITY_LIMIT);
    private static final double STOP_INTAKE_DELAY = 0.5;
    private final Timer m_intakeStopTimer = new Timer();
    private final DoubleSupplier m_elevatorHeight;
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    
    // State
    //  IDLE = nothing happening
    //  INTAKE = run the intake
    //  INTAKE_HAS_CORAL = we think we have a coral, but keep running the intake 
    //          - helps if the limit switch gets stuck on
    //  INTAKE_STOPPING = keep running the intake a little after the button is released
    //          - give more time for the velocity check to trigger
    //  OUTTAKE = run the motor for scoring
    //  HOLD = run intake at low speed to hold the coral in
    private enum State {
        IDLE, INTAKE, INTAKE_HAS_CORAL, INTAKE_STOPPING, HOLD, OUTTAKE;
    }

    private State m_state = State.HOLD;

    public Claw(DoubleSupplier elevatorHeight) {
        m_elevatorHeight = elevatorHeight;
        // Set up the coral motor as brushless motor
        m_motor = new TalonFX(Constants.CLAW_CAN_ID);

        CurrentLimitsConfigs currentLimits = new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(MOTOR_CURRENT_LIMIT)
            .withStatorCurrentLimit(MOTOR_CURRENT_LIMIT);
        talonFXConfigs.withCurrentLimits(currentLimits);
        
        m_motor.getConfigurator().apply(talonFXConfigs);
        
        // enable brake mode (after main config)
        m_motor.setNeutralMode(NeutralModeValue.Brake);

    }

    @Override
    public void periodic() {
        // always want this to run
        double velocity = m_motor.getVelocity().getValueAsDouble();
        // detect a stall as when the velocity *falls* below a threshold
        boolean stalled = m_speedThres.compute(Math.abs(velocity));


        // Running the intake. Look for getting a coral, but don't switch to HOLD until
        // the button is let go
        if (m_state == State.INTAKE && (stalled)) {
            // leave motor running. This might be a stuck limit switch
            m_state = State.INTAKE_HAS_CORAL;
        }

        if (m_state == State.INTAKE_STOPPING) {
            if (stalled) {
                m_state = State.HOLD;
            } else if (m_intakeStopTimer.hasElapsed(STOP_INTAKE_DELAY)) {
                m_state = State.IDLE;
            }
        }

        // set motor speed for transitions that may have just happened
        // HOLD and IDLE might be used at startup, so we want to set them anyway
        if (m_state == State.HOLD) {
            m_motor.set(HOLD_SPEED);
        } else if (m_state == State.IDLE) {
            m_motor.set(0);
        }


        SmartDashboard.putString("coralEffector/state", m_state.toString());
        SmartDashboard.putNumber("coralEffector/setSpeed", m_motor.get());
        SmartDashboard.putNumber("coralEffector/velocity", velocity);
    }

    public void runIntake() {
        m_motor.set(INTAKE_SPEED);
        m_state = State.INTAKE;
    }

    public void runOuttake() {
        double elevatorH = m_elevatorHeight.getAsDouble();
        if (Math.abs(elevatorH - MoveEndEffectorRedesign.L1_HEIGHT) < 0.01) {
            m_motor.set(OUTTAKE_L1_SPEED);
        } else {
            m_motor.set(OUTTAKE_SPEED);
        }

        m_state = State.OUTTAKE;
    }

    public void stop() {
        if (m_state == State.INTAKE_HAS_CORAL) {
            m_motor.set(HOLD_SPEED);
            m_state = State.HOLD;
        }

        if (m_state == State.INTAKE) {
            // we have not register picking up a Coral.
            // leave intake on for a little longer in hopes of triggering the speed test
            m_intakeStopTimer.restart();
            m_state = State.INTAKE_STOPPING;
        } else if (m_state != State.HOLD) {
            m_state = State.IDLE;
            m_motor.set(0);
        }
    }

    public boolean hasCoral() {
        return m_state == State.HOLD || m_state == State.INTAKE_HAS_CORAL;
    }

    

}
