// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
// import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

    private static final double GEAR_REDUCTION = (1.0 / 4.0) * (14.0 / 60.0);

    private static final double MAX_LENGTH_METERS = Units.inchesToMeters(13.5);
    private static final double MIN_LENGTH_METERS = Units.inchesToMeters(0.25);
    private static int m_lengthAdjustment = 1;
    
    
        // Tolerance for commands
        private static final double LENGTH_TOLERANCE_METERS = Units.inchesToMeters(0.5);
        
        // For initial testing, these should be very slow.
        // We can update them as we get more confidence.
        private static final double MAX_VEL_METER_PER_SEC = Units.inchesToMeters(30.0);
    
        // private static final double ELEVATOR_MAX_ACC_METER_PER_SEC_SQ = Units.inchesToMeters(50.0);
    
        private static final double MAX_ACC_METER_PER_SEC_SQ = Units.inchesToMeters(75.0);
    
        private static final double METER_PER_REVOLUTION = Units.inchesToMeters((1.504*Math.PI)*GEAR_REDUCTION);
        
        private static final int CURRENT_LIMIT = 45;
    
        // PID Constants for the reacher PID controller
        // Since we're using Trapeziodal control, all values will be 0 except for P
        private static final double K_P = 20.0; 
        private static final double K_I = 0.0;
        private static final double K_D = 0.0;
        private static final double K_FF = 0.0;
    
        // constants for various commands
        public static final double STOW_LENGTH = Units.inchesToMeters(0.5);
        public static final double AMP_SCORE_LENGTH = Units.inchesToMeters(13.0);
    
        private static final double OFFSET_METER = 0.0;
    
        private static final double ADJUSTMENT_STEP = 1.0;
        private double m_goal = 0;//this should be in inches 
        
        // initializing Potentiometer
        private final int POTENTIOMETER_CHANNEL = 2; //TODO: Update with actual value
        private final double POTENTIOMETER_RANGE_METERS = -2.625; // meters, the string potentiometer on takes in range in integers TODO: update to correct value
        private final double POTENTIOMETER_OFFSET = 2.51; //TODO: Find inital value and update
    
        // Define the motor and encoders
        private final TalonFX m_motor;
        private final TalonFXConfiguration m_talonFXConfigs;
        private final MotionMagicConfigs m_magicConfigs;
        private final Slot0Configs m_slot0configs;
        private final AnalogPotentiometer m_stringPotentiometer;
    
    
        /** Creates a new Elevator. */
        public Elevator() {
         m_talonFXConfigs = new TalonFXConfiguration();
    
    // set slot 0 gains
        m_motor = new TalonFX(Constants.ELEVATOR_CAN_ID);
        m_slot0configs = m_talonFXConfigs.Slot0;
        m_slot0configs.kS = 0.25; // Add 0.25 V output to overcome static friction //TODO Change values 
        m_slot0configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        m_slot0configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        m_slot0configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
        m_slot0configs.kI = 0; // no output for integrated error
        m_slot0configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output
    
    // set Motion Magic settings
        m_magicConfigs = m_talonFXConfigs.MotionMagic;
    
        m_magicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps //TODO Change values 
        m_magicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        m_magicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
    
        m_motor.getConfigurator().apply(m_talonFXConfigs);
        m_stringPotentiometer = new AnalogPotentiometer(POTENTIOMETER_CHANNEL, POTENTIOMETER_RANGE_METERS, POTENTIOMETER_OFFSET);
            
        m_motor.setPosition(positionToRotations(m_stringPotentiometer.get()));
          
        }
    
        @Override
        public void periodic() {
            SmartDashboard.putNumber("elevator/encoder", Units.metersToInches(getLength()));
            SmartDashboard.putBoolean("elevator/onGoal", lengthWithinTolerance());
            
            // SmartDashboard.putNumber("elevator/stringPot", Units.metersToInches(getPotentiometerReadingMeters()));
    
            // useful for initial calibration; comment out later?
            // SmartDashboard.putNumber("elevator/encoderMeter", m_encoder.getPosition());
            // SmartDashboard.putNumber("elevator/stringPotMeter", getPotentiometerReadingMeters());
    
        }
    
       
    
    
    
        public double getLength() {
            return m_stringPotentiometer.get();  
        }
    
        public void zeroElevator() {
            m_motor.setPosition(OFFSET_METER);
            // updateMotorEncoderOffset();
        }
    
        // public double getPotentiometerReadingMeters(){
        //     return m_stringPotentiometer.get();
        // }
    
        // public void updateMotorEncoderOffset() {
        //     m_encoder.setPosition(getPotentiometerReadingMeters());
        // }

        public static double positionToRotations(double position){
            return 
            //TODO add conversion 
        }
    
        private static double limitElevatorLength(double length){
            return MathUtil.clamp(length, MIN_LENGTH_METERS, MAX_LENGTH_METERS);
        }
    
        // set elevator length in meters
        public void setLength(double goal, boolean includeAdjustment) {
            final MotionMagicVoltage m_request = new MotionMagicVoltage(0);        
            goal = limitElevatorLength(positionToRotations(goal) + (includeAdjustment ? 1 : 0) * m_lengthAdjustment);
            m_motor.setControl(m_request.withPosition(goal))
            SmartDashboard.putNumber("elevator/goal", Units.metersToInches(goal));
        }
    
        public void adjustLength(boolean goUp) {
            double adjust = (goUp ? 1 : -1) * ADJUSTMENT_STEP;
            m_lengthAdjustment += adjust;
        setLength(getLength() + adjust, false);
    }

    public boolean lengthWithinTolerance() {
        return Math.abs(getLength() - m_goal) < LENGTH_TOLERANCE_METERS;
    }

}