package frc.robot.subsystems.kitbot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

// TODO THIS IS JUST FOR TESTING DO NOT USE WITHOUT KNOWING WHAT YOU ARE DOING
public class NeoClimber extends SubsystemBase {
    
    private final SparkMax m_motor;
    private final RelativeEncoder m_encoder;
    
    // Constants to be used in this class
    private static final double DEPLOYED_ROTATIONS = -20.0;
    private static final double CLIMB_ROTATIONS = 200.0; 
    
    // Protection values
    // private static final double MAX_ROTATIONS_ALLOWED = 710.0;
    // private static final double MAX_CURRENT = 100.0;
    
    // Current limit in the motor
    private static final int CURRENT_LIMIT = 40;
    
    // Winch motor speed values
    private static final double DEPLOY_MAX_SPEED = -0.2;
    private static final double CLIMB_SPEED = 0.5;
    
    public static final double MANUAL_SPEED = 0.3;
        
    private static final double MOTOR_VOLTAGE_COMP = 12;
    
    // State definitions:
    // IDLE - winch holding system in place, should be used for entire match until End Game.
    // DEPLOYING - winch unwinding ropes to allow climber to raise.
    // WAITING - waiting for robot to get close enough to cage to climb.
    // CLIMBING - climb. Stop automatically when we think we are high enough (based on rope length)
    // HOLDING - Normally, this would mean the the robot is now off the floor and the ratchets are engaged.
    //      Allow the driver to pull a little further, within reason
    private enum ClimberState {IDLE, DEPLOYING, WAITING, CLIMBING, HOLDING};
    private ClimberState m_climberState = ClimberState.IDLE;
    
    public NeoClimber() {
        m_motor = new SparkMax(Constants.CLIMBER_ID, MotorType.kBrushless);
        m_motor.setCANTimeout(250);
        
        // Create and apply configuration for roller motor. Voltage compensation help
        // the roller behave the same as the battery
        // voltage dips. The current limit helps prevent breaker trips or burning out
        // the motor in the event the roller stalls.
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.voltageCompensation(MOTOR_VOLTAGE_COMP);
        config.smartCurrentLimit(CURRENT_LIMIT);

        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_encoder = m_motor.getEncoder();
    }
    
    @Override
    public void periodic() {
        
        double position = getPosition();

        SmartDashboard.putNumber("climber/position", position);
        SmartDashboard.putNumber("climber/speed", m_motor.get());
        SmartDashboard.putString("climber/state", m_climberState.toString());
        
        if (m_climberState == ClimberState.DEPLOYING) {
            // Deploy goes NEGATIVE!!
            if (position <= DEPLOYED_ROTATIONS) {
                // Stop winch
                m_motor.set(0.0);
                m_climberState = ClimberState.WAITING;
            }
        }
        else if (m_climberState == ClimberState.CLIMBING) {          
            if (position >= CLIMB_ROTATIONS) {
                m_motor.set(0.0);
                m_climberState = ClimberState.HOLDING;
            }
        }
        
        // Protection: stop if position is at max or current is too high
        // Test no matter what State we are in. 
        // // Do this at the end to override any settings
        // if (position >= MAX_ROTATIONS_ALLOWED || climberCurrrent > MAX_CURRENT)
        //     m_motor.set(0.0);
    }
    
    public void run(double climbSpeed) {
        m_motor.set(climbSpeed);
        // m_climberMotor.set( climbSpeed);
    }
    
    public double getPosition(){
        return m_encoder.getPosition();
    }
    
    public void deploy() {
        m_climberState = ClimberState.DEPLOYING;
        run(DEPLOY_MAX_SPEED);
    }
    
    public void climb() {
        m_climberState = ClimberState.CLIMBING;
        run(CLIMB_SPEED);
    }
    
    public void hold() {
        m_climberState = ClimberState.HOLDING;
        m_motor.set(0);
    }
}
