package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import edu.wpi.first.math.MathUtil;

public class Climber extends SubsystemBase {
    
    private final SparkMax m_climberMotor;
    private final RelativeEncoder m_climberMotorEncoder;
    
    // Constants to be used in this class
    private static final double DEPLOYED_ROTATIONS = -10.0;
    private static final double CLIMB_ROTATIONS = 60.0; 
    
    // Protection values
    private static final double MAX_WINCH_ROTATIONS_ALLOWED = 70.0;
    private static final double MAX_WINCH_CURRENT = 100.0;
    // Current limit in the SparkMax
    private static final int WINCH_CURRENT_LIMIT = 40;

    // Winch motor speed values
    private static final double IDLE_MOTOR_SPEED = -0.01;
    private static final double WINCH_EXTEND_MAX_SPEED = -0.2;
    private static final double WINCH_EXTEND_MIN_SPEED = -0.1;
    private static final double WINCH_RETRACT_SPEED = 0.5;
    public static final double WINCH_MANUAL_SPEED = 0.3;
    private static final double WINCH_CLIMB_SPEED = 0.5;
    
    private static final double EXTEND_SLOWDOWN_INTERVAL = 30.0;
    
    // speed adjustment amount if Roll is off by ROLL_ANGLE_TOLERANCE
    // private static final double WINCH_CLIMB_ADJUST_SPEED = 0.1;
    private static final double MOTOR_VOLTAGE_COMP = 12;
    
    // State definitions:
    // IDLE - Winches holding hooks in place, Should be used for entire match until End Game.
    // DEPLOYING - Winches unwinding ropes to allow hooks to raise.
    // WAITING - Hooks at max height. Waiting dor robot to get close enough to chain to lower hooks.
    // CLIMBING - Hooks have engaged with chain. As the winches retract further, the robot goes up.
    //      Since the robot is no longer onthe ground, we need to adjust the climb speeds to keep the robot level.
    // HOLDING - Normally, this would mean the the robot is now off the floor and the ratchets are engaged.
    //      In an emergency, if the robot rools too far, we will stop the motors and let the ratchets hold the robot.
    //      In this state, the runWinch() operation can allow further adjustments as needed.
    private enum ClimberState {IDLE, DEPLOYING, WAITING, CLIMBING, HOLDING};
    private ClimberState m_climberState = ClimberState.IDLE;
    
    private final DriveTrain m_driveTrain;
    
    public Climber(DriveTrain drivetrain) {
        m_driveTrain = drivetrain;
        
        m_climberMotor = new SparkMax(Constants.CLIMBER_ID, MotorType.kBrushless);
        
        SparkMaxConfig config = new SparkMaxConfig();
        
        config.voltageCompensation(MOTOR_VOLTAGE_COMP);
        config.smartCurrentLimit(WINCH_CURRENT_LIMIT);
        config.inverted(false);//guess 
        config.idleMode(IdleMode.kBrake);
        // m_climberMotor.setCANTimeout(250);
        
        m_climberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        m_climberMotorEncoder = m_climberMotor.getEncoder();
        m_climberMotorEncoder.setPosition(0.0);
    }
    
    @Override
    public void periodic() {
        double position = m_climberMotorEncoder.getPosition();
        
        double climberCurrrent = m_climberMotor.getOutputCurrent();
        
        SmartDashboard.putNumber("climber/Speed", m_climberMotor.get());
        SmartDashboard.putNumber("climber/Position", position);
        SmartDashboard.putNumber("climber/Current", climberCurrrent);
        SmartDashboard.putNumber("climber/pitch", m_driveTrain.getPitch().getDegrees());
        SmartDashboard.putString("climber/state", m_climberState.toString());
        
        // While idle, we want a small voltage applied to hold the hooks in place.
        if (m_climberState == ClimberState.IDLE) {
            m_climberMotor.set(IDLE_MOTOR_SPEED);
            // Note that the DEPLOYING state is entered via a command, so here we just stay IDLE
        }
        else if (m_climberState == ClimberState.DEPLOYING) {
            // use a "trapezoidal" profile to slow down near the end
            
            // Deploy goes NEGATIVE!!
            // double error = Math.abs(DEPLOYED_ROTATIONS - position);
            if (position <= DEPLOYED_ROTATIONS) {
                // Stop winch
                m_climberMotor.set(0.0);
                m_climberState = ClimberState.WAITING;
            }
            // else {
            //     // slow down over the last N rotations
            //     double speed =  error / EXTEND_SLOWDOWN_INTERVAL * WINCH_EXTEND_MAX_SPEED;
            //     m_climberMotor.set(MathUtil.clamp(speed, WINCH_EXTEND_MIN_SPEED, WINCH_EXTEND_MAX_SPEED));
            // }
        }
        else if (m_climberState == ClimberState.WAITING) {
            // Nothing to do here since both winches are stopped and the hooks are as high as they can go.
            // The CLIMBING state is only entered via a command
        }
        else if (m_climberState == ClimberState.CLIMBING) {
            m_climberMotor.set(WINCH_CLIMB_SPEED);
        }
        else if (m_climberState == ClimberState.CLIMBING) {          
            if (position >= CLIMB_ROTATIONS) {
                m_climberMotor.set(0.0);
                m_climberState = ClimberState.HOLDING;
            }
        }
        
        // Protection: stop if position is at max or current is too high
        // Test no matter what State we are in. 
        // Do this at the end to override any settings
        if (position >= MAX_WINCH_ROTATIONS_ALLOWED || climberCurrrent > MAX_WINCH_CURRENT)
            m_climberMotor.set(0.0);
    }
    
    private double limitWinch(RelativeEncoder encoder, double speed) {
        if (encoder.getPosition() >= MAX_WINCH_ROTATIONS_ALLOWED) {
            return 0.0;
        } else
        return speed;
    }
    
    public void run(double climbSpeed) {
        m_climberMotor.set(limitWinch(m_climberMotorEncoder, climbSpeed));
    }
    
    public double getPosition(){
        return m_climberMotorEncoder.getPosition();
    }
    
    public double getVelocity(){
        return m_climberMotorEncoder.getVelocity();
    }
    
    public void deploy() {
        m_climberState = ClimberState.DEPLOYING;
        run(WINCH_EXTEND_MAX_SPEED);
    }
    
    public void climb() {
        m_climberState = ClimberState.CLIMBING;
        run(WINCH_RETRACT_SPEED);
    }
    
    public void hold() {
        m_climberState = ClimberState.HOLDING;
        m_climberMotor.set(0.0);
    }
}
