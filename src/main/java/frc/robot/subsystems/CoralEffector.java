// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
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
import frc.robot.subsystems.ValueThreshold.Direction;

public class CoralEffector extends SubsystemBase {
    /** Creates a new CoralEndEffector. */
    private static final int MOTOR_CURRENT_LIMIT = 30;

    // This sets a limit for voltage to 10 so it is repeatable
    // until the battery dips below 10 volts
    private static final double MOTOR_VOLTAGE_COMP = 10;

    // Speeds
    private static final double INTAKE_SPEED = -0.4;
    private static final double OUTTAKE_SPEED = 0.5;
    private static final double HOLD_SPEED = -0.05;

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
    
    // State
    private enum State {
        IDLE, INTAKE, OUTTAKE, HOLD;
    }

    private State m_state = State.HOLD;

    public CoralEffector() {
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
        LimitSwitchConfig lsConfig = new  LimitSwitchConfig();
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
        // allow IDLE -> HOLD. This can happen at the start of a match, or manually.
        if (m_limitSwitchDebounced && (m_state == State.IDLE || m_state == State.INTAKE)) {
            m_motor.set(HOLD_SPEED);
            m_state = State.HOLD;        
        }
        
        double velocity = m_encoder.getVelocity();
        boolean stalled = m_speedThres.compute(Math.abs(velocity));
        if (m_state == State.INTAKE && stalled) {
            m_motor.set(HOLD_SPEED);
            m_state = State.HOLD;        
        }

        // isRunning and hasElapsed is probably redundant here, but better safe
        if (m_state == State.INTAKE && m_intakeStopTimer.isRunning() && m_intakeStopTimer.hasElapsed(STOP_INTAKE_DELAY)) {
            // note: don't call stop() - that would leave it on forever
            m_motor.set(0);
            m_state = State.IDLE;
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

        // stop and zero the timer so it does not trip
        m_intakeStopTimer.stop();
        m_intakeStopTimer.reset();
    }

    public void runOuttake() {
        m_motor.set(OUTTAKE_SPEED);
        m_state = State.OUTTAKE;
    }

    public void stop() {
        // test timer, for safety
        if (m_state == State.INTAKE && !m_intakeStopTimer.isRunning()) {
            // we have not register picking up a Coral.
            // leave INTAKE on for a little longer in hopes of triggering the speed test
            m_intakeStopTimer.restart();
        }
        else if (m_state != State.HOLD) {
            m_state = State.IDLE;
            m_motor.stopMotor();
        }
    }

    public boolean hasCoral() {
        return m_state == State.HOLD || m_limitSwitchDebounced;
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
