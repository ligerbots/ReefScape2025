package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Position;

public class RobotState extends SubsystemBase {
    // Supplier to dynamically get the current robot position
    private Position m_currRobotState;

    public RobotState() {
        // Initialize with a Supplier that returns the starting position
        m_currRobotState = Position.STARTING_CONFIG;
    }

    @Override
    public void periodic() {
        // Any periodic code that needs to run every cycle can be added here.
        SmartDashboard.putString("RobotState", m_currRobotState.toString());
    }

    // Method to set a new robot state by changing the Supplier
    public void setRobotState(Position pos) {
        m_currRobotState = pos;
    }

    // Method to get the current robot state
    public Position getRobotState() {
        return m_currRobotState;
    }

    public void setRobotStateL2(){
        m_currRobotState = Position.L2;
    }
    public void setRobotStateL3(){
        m_currRobotState = Position.L3;
    }
    public void setRobotStateL4(){
        m_currRobotState = Position.L4;
    }
    public void setRobotStateL2_ALGAE(){
        m_currRobotState = Position.L2_ALGAE;
    }
    public void setRobotStateL3_ALGAE(){
        m_currRobotState = Position.L3_ALGAE;
    }
    public void setRobotStateBARGE(){
        m_currRobotState = Position.BARGE;
    }
    public void setRobotStateSTOW(){
        m_currRobotState = Position.STOW;
    }
    public void setRobotStateL2_PREP(){
        m_currRobotState = Position.L2_PREP;
    }
    public void setRobotStateL3_PREP(){
        m_currRobotState = Position.L3_PREP;
    }
    public void setRobotStateL4_PREP(){
        m_currRobotState = Position.L4_PREP;
    }
}
