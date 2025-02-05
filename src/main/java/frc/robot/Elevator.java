// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants;

public class Elevator extends TrapezoidProfileSubsystem {

    private static final double GEAR_REDUCTION = (1.0 / 4.0) * (14.0 / 60.0);

    private static final double MAX_LENGTH_METERS = Units.inchesToMeters(13.5);
    private static final double MIN_LENGTH_METERS = Units.inchesToMeters(0.25);

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
    
    // initializing Potentiometer
    // private final int POTENTIOMETER_CHANNEL = 2; //TODO: Update with actual value
    // private final double POTENTIOMETER_RANGE_METERS = -2.625; // meters, the string potentiometer on takes in range in integers TODO: update to correct value
    // private final double POTENTIOMETER_OFFSET = 2.51; //TODO: Find inital value and update

    // Define the motor and encoders
    private final CANSparkMax m_motor;
    private final RelativeEncoder m_encoder;

    // private final AnalogPotentiometer m_stringPotentiometer;
    private final SparkPIDController m_PIDController;

    private double m_goal = 0;
    private double m_lengthAdjustment = 0;

    /** Creates a new Elevator. */
    public Elevator() {
        super(new TrapezoidProfile.Constraints(MAX_VEL_METER_PER_SEC, MAX_ACC_METER_PER_SEC_SQ));

        // Create the motor, PID Controller and encoder.
        m_motor = new CANSparkMax(Constants.ELEVATOR_CAN_ID, CANSparkMax.MotorType.kBrushless);
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(true);
        
        m_motor.setSmartCurrentLimit(CURRENT_LIMIT);

        m_PIDController = m_motor.getPIDController();
        m_PIDController.setP(K_P);
        m_PIDController.setI(K_I);
        m_PIDController.setD(K_D);
        m_PIDController.setFF(K_FF);

        m_encoder = m_motor.getEncoder();
        // Set the position conversion factor.
        m_encoder.setPositionConversionFactor(METER_PER_REVOLUTION);

        // m_stringPotentiometer = new AnalogPotentiometer(POTENTIOMETER_CHANNEL, POTENTIOMETER_RANGE_METERS, POTENTIOMETER_OFFSET);
        zeroElevator();

        SmartDashboard.putBoolean("elevator/coastMode", false);
        setCoastMode();

        // Create SD values needed during testing. Here so that they are visible in NetworkTables
        SmartDashboard.putNumber("elevator/testLength", 0);

        if (Constants.SPARKMAX_BURN_FLASH) {
            m_motor.burnFlash();
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elevator/encoder", Units.metersToInches(getLength()));
        SmartDashboard.putNumber("elevator/current", m_motor.getOutputCurrent());
        SmartDashboard.putBoolean("elevator/onGoal", lengthWithinTolerance());
        SmartDashboard.putNumber("elevator/adjustment", Units.metersToInches(m_lengthAdjustment));
        
        // SmartDashboard.putNumber("elevator/stringPot", Units.metersToInches(getPotentiometerReadingMeters()));

        // useful for initial calibration; comment out later?
        // SmartDashboard.putNumber("elevator/encoderMeter", m_encoder.getPosition());
        // SmartDashboard.putNumber("elevator/stringPotMeter", getPotentiometerReadingMeters());

        setCoastMode();

        super.periodic();
    }

    @Override
    protected void useState(TrapezoidProfile.State setPoint) {
        // Remember that the encoder was already set to account for the gear ratios.

        // TODO include FF?
        m_PIDController.setReference(setPoint.position, CANSparkMax.ControlType.kPosition); 
        SmartDashboard.putNumber("elevator/setPoint", Units.metersToInches(setPoint.position));
    }

    public double getLength() {
        return m_encoder.getPosition();
    }

    public void zeroElevator() {
        m_encoder.setPosition(OFFSET_METER);
        // updateMotorEncoderOffset();
    }

    // public double getPotentiometerReadingMeters(){
    //     return m_stringPotentiometer.get();
    // }

    // public void updateMotorEncoderOffset() {
    //     m_encoder.setPosition(getPotentiometerReadingMeters());
    // }

    private static double limitElevatorLength(double length){
        return MathUtil.clamp(length, MIN_LENGTH_METERS, MAX_LENGTH_METERS);
    }

    // set elevator length in meters
    public void setLength(double goal, boolean includeAdjustment) {
        m_goal = limitElevatorLength(goal + (includeAdjustment ? 1 : 0) * m_lengthAdjustment);
        super.setGoal(m_goal);
        SmartDashboard.putNumber("elevator/goal", Units.metersToInches(m_goal));
    }

    public void adjustLength(boolean goUp) {
        double adjust = (goUp ? 1 : -1) * ADJUSTMENT_STEP;
        m_lengthAdjustment += adjust;
        setLength(getLength() + adjust, false);
    }

    public boolean lengthWithinTolerance() {
        return Math.abs(getLength() - m_goal) < LENGTH_TOLERANCE_METERS;
    }

    public void setCoastMode(){
        boolean coastMode = SmartDashboard.getBoolean("elevator/coastMode", false);
        if (coastMode) {
            m_motor.setIdleMode(IdleMode.kCoast);
            m_motor.stopMotor();
        } else
            m_motor.setIdleMode(IdleMode.kBrake);
    }
}

// package frc.robot;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Elevator extends SubsystemBase {
//        // Elevator height (each )
//        private enum Height {
//         l1, l2, l3, l4;
//     }
// }
