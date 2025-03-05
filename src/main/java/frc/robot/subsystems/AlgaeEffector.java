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
import com.revrobotics.spark.SparkLimitSwitch;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeEffector extends SubsystemBase {
    private static final int MOTOR_CURRENT_LIMIT = 20;
   
    private static final double INTAKE_VOLTAGE = -6.0;
    private static final double PROCESSOR_VOLTAGE = 6.0;
    private static final double BARGE_VOLTAGE = 12.0;
    private static final double HOLD_VOLTAGE = -4.0;

    private final static double INTAKE_CURRENT_THRESHOLD = 15;

    // elevator is at the bottom when scoring in the Processor. Use some small height
    private final static double PROCESSOR_HEIGHT_MAX = Units.inchesToMeters(10.0);

    private final SparkMax m_motor;
    private final SparkLimitSwitch m_limitSwitch;

    // State
    private enum State {
        IDLE, INTAKE, BARGE, PROCESSOR, HOLD;
    }

    private State m_state = State.IDLE;

    // median filter to filter the feeder current, to signal holding an algae
    private final MedianFilter m_medianFilter = new MedianFilter(5);
    private double m_medianCurrent = 0.0;
    private boolean m_prevCurrentTrigger;
    private boolean m_pastFirstCurrentSpike;

    private final DoubleSupplier m_elevatorHeight;

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

        boolean currentTriggered = m_medianCurrent > INTAKE_CURRENT_THRESHOLD;;
        if (m_state == State.INTAKE && !m_prevCurrentTrigger && currentTriggered) {
            if (!m_pastFirstCurrentSpike) {
                m_pastFirstCurrentSpike = true;
            } else {
                // current has spiked a 2nd time, so assume we have an Algae
                m_motor.setVoltage(HOLD_VOLTAGE);
                m_state = State.HOLD;
            }
        }
        m_prevCurrentTrigger = currentTriggered;

        SmartDashboard.putString("algaeEffector/state", m_state.toString());
        SmartDashboard.putBoolean("algaeEffector/limitSwitch", pressed);
        SmartDashboard.putNumber("algaeEffector/speed", m_motor.get());
        SmartDashboard.putNumber("algaeEffector/current", m_motor.getOutputCurrent());
        SmartDashboard.putNumber("algaeEffector/medianCurrent", m_medianCurrent);
        SmartDashboard.putBoolean("algaeEffector/currentTrigger", currentTriggered);
    }

    public void runIntake() {
        m_prevCurrentTrigger = false;
        m_pastFirstCurrentSpike = false;

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

    public Runnable updateCurrentReading(){
        return () -> {
            m_medianCurrent = m_medianFilter.calculate(m_motor.getOutputCurrent());
        };
    }
}
