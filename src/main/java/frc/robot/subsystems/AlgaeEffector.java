// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLimitSwitch;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.ValueThreshold.Direction;

public class AlgaeEffector extends SubsystemBase {

    static final int MOTOR_CURRENT_LIMIT = 30;
   
    static final double INTAKE_VOLTAGE = -6.0;
    static final double PROCESSOR_VOLTAGE = 6.0;
    static final double BARGE_VOLTAGE = 10.0;
    static final double HOLD_VOLTAGE = -6.0;

    // Max velocity indicating the motor has stalled
    private final static double STALL_VELOCITY_LIMIT = 1000;

    // elevator is at the bottom when scoring in the Processor. Use some small height
    private final static double PROCESSOR_HEIGHT_MAX = Units.inchesToMeters(10.0);

    private final SparkMax m_motor;
    private final SparkLimitSwitch m_limitSwitch;
    private final RelativeEncoder m_encoder;

    // State
    private enum State {
        IDLE, INTAKE, BARGE, PROCESSOR, HOLD;
    }

    private State m_state = State.IDLE;

    private final DoubleSupplier m_elevatorHeight;

    private final ValueThreshold m_speedThres = new ValueThreshold(Direction.FALLING, STALL_VELOCITY_LIMIT);

    public AlgaeEffector(DoubleSupplier elevatorHeight) {
        m_elevatorHeight = elevatorHeight;

        // Set up the   motor as a brushed motor
        m_motor = new SparkMax(Constants.ALGAE_EFFECTOR_INTAKE_ID, MotorType.kBrushless);

        // // Set can timeout. Because this project only sets parameters once on
        // // construction, the timeout can be long without blocking robot operation. Code
        // // which sets or gets parameters during operation may need a shorter timeout.
        // m_motor.setCANTimeout(250);
                
        // Configuration for the motor
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        // always set a current limit
        config.smartCurrentLimit(MOTOR_CURRENT_LIMIT);
        
        // include the config of the limit switch, for completeness
        LimitSwitchConfig lsConfig = new  LimitSwitchConfig();
        lsConfig.reverseLimitSwitchType(Type.kNormallyOpen);
        // don't shut off motor when pressed. We will handle that.
        lsConfig.reverseLimitSwitchEnabled(false);
        config.apply(lsConfig);

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_encoder = m_motor.getEncoder();

        // Get the reverse limit switch
        m_limitSwitch = m_motor.getReverseLimitSwitch();
    }

    @Override
    public void periodic() {
        boolean pressed = m_limitSwitch.isPressed();
        if (pressed && (m_state == State.IDLE || m_state == State.INTAKE)) {
            m_motor.setVoltage(HOLD_VOLTAGE);
            m_state = State.HOLD;        
        }

        double velocity = m_encoder.getVelocity();
        boolean stalled = m_speedThres.compute(Math.abs(velocity));
        if (m_state == State.INTAKE && stalled) {
            m_motor.setVoltage(HOLD_VOLTAGE);
            m_state = State.HOLD;        
        }

        SmartDashboard.putString("algaeEffector/state", m_state.toString());
        SmartDashboard.putBoolean("algaeEffector/limitSwitch", pressed);
        SmartDashboard.putNumber("algaeEffector/setSpeed", m_motor.get());
        SmartDashboard.putNumber("algaeEffector/current", m_motor.getOutputCurrent());
        SmartDashboard.putNumber("algaeEffector/velocity", velocity);
    }

    public void runIntake() {
        m_motor.setVoltage(INTAKE_VOLTAGE);
        m_state = State.INTAKE;
    }

    public void score() {
        double elevatorH = m_elevatorHeight.getAsDouble();
        if (elevatorH >= PROCESSOR_HEIGHT_MAX) {
            m_motor.setVoltage(BARGE_VOLTAGE);
            m_state = State.BARGE;
        } else {
            m_motor.setVoltage(PROCESSOR_VOLTAGE);
            m_state = State.PROCESSOR;    
        }
    }

    public void stop() {
        if (m_state != State.HOLD) {
            m_motor.stopMotor();
            m_state = State.IDLE;
        }
    }

    public boolean hasAlgae() {
        return m_state == State.HOLD;
    }
}
