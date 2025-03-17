// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

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
import frc.robot.commands.MoveEndEffector;
import frc.robot.subsystems.ValueThreshold.Direction;

public class CoralEffector extends SubsystemBase {
    /** Creates a new CoralEndEffector. */
    private static final int MOTOR_CURRENT_LIMIT = 30;

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
    private final SparkMax m_motor;
    private final RelativeEncoder m_encoder;

    // Limit Switch
    private final SparkLimitSwitch m_limitSwitch;
    private final Debouncer m_limitDebouncer = new Debouncer(0.025, DebounceType.kFalling);
    private boolean m_limitSwitchDebounced = false;
    // private final BooleanLogEntry m_limitSwitchLogger;

    private final ValueThreshold m_speedThres = new ValueThreshold(Direction.FALLING, STALL_VELOCITY_LIMIT);
    private static final double STOP_INTAKE_DELAY = 0.5;
    private final Timer m_intakeStopTimer = new Timer();
    private final DoubleSupplier m_elevatorHeight;
    
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

    public CoralEffector(DoubleSupplier elevatorHeight) {
        m_elevatorHeight = elevatorHeight;
        // Set up the coral motor as brushless motor
        m_motor = new SparkMax(Constants.CORAL_EFFECTOR_INTAKE_ID, MotorType.kBrushless);

        // Set can timeout. Because this project only sets parameters once on
        // construction, the timeout can be long without blocking robot operation. Code
        // which sets or gets parameters during operation may need a shorter timeout.
        m_motor.setCANTimeout(250);

        // Create and apply configuration for roller motor. Voltage compensation help
        // the roller behave the same as the battery
        // voltage dips. The current limit helps prevent breaker trips or burning out
        // the motor in the event the roller stalls.
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.voltageCompensation(MOTOR_VOLTAGE_COMP);
        config.smartCurrentLimit(MOTOR_CURRENT_LIMIT);

        // include the config of the limit switch, for completeness
        LimitSwitchConfig lsConfig = new LimitSwitchConfig();
        lsConfig.reverseLimitSwitchType(Type.kNormallyOpen);
        // don't shut off motor when pressed. We will handle that.
        lsConfig.reverseLimitSwitchEnabled(false);
        config.apply(lsConfig);

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        m_limitSwitch = m_motor.getReverseLimitSwitch();
        m_encoder = m_motor.getEncoder();

        // log the raw limit switch. Probably should be turned off after debugging
        // m_limitSwitchLogger = new BooleanLogEntry(DataLogManager.getLog(), "/coralEffector/limitSwitch");
    }

    @Override
    public void periodic() {
        // always want this to run
        double velocity = m_encoder.getVelocity();
        // detect a stall as when the velocity *falls* below a threshold
        boolean stalled = m_speedThres.compute(Math.abs(velocity));

        // allow IDLE -> HOLD. This can happen at the start of a match, or manually.
        if (m_state == State.IDLE && m_limitSwitchDebounced) {
            m_state = State.HOLD;
        }

        // Running the intake. Look for getting a coral, but don't switch to HOLD until
        // the button is let go
        if (m_state == State.INTAKE && (m_limitSwitchDebounced || stalled)) {
            // leave motor running. This might be a stuck limit switch
            m_state = State.INTAKE_HAS_CORAL;
        }

        if (m_state == State.INTAKE_STOPPING) {
            if (m_limitSwitchDebounced || stalled) {
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


        SmartDashboard.putBoolean("coralEffector/limitSwitchDebounced", m_limitSwitchDebounced);
        SmartDashboard.putString("coralEffector/state", m_state.toString());
        SmartDashboard.putNumber("coralEffector/setSpeed", m_motor.get());
        SmartDashboard.putNumber("coralEffector/current", m_motor.getOutputCurrent());
        SmartDashboard.putNumber("coralEffector/velocity", velocity);
    }

    public void runIntake() {
        m_motor.set(INTAKE_SPEED);
        m_state = State.INTAKE;
    }

    public void runOuttake() {
        double elevatorH = m_elevatorHeight.getAsDouble();
        if (Math.abs(elevatorH - MoveEndEffector.L1_HEIGHT) < 0.01) {
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

    public Runnable updateLimitSwitch() {
        return () -> {
            boolean isPressed = m_limitSwitch.isPressed();
            m_limitSwitchDebounced = m_limitDebouncer.calculate(isPressed);

            // also log it to see how it behaves. This might not be fast enough, but worth a try
            // probably turn off after debugging
            // m_limitSwitchLogger.append(isPressed);
        };
    }
}
