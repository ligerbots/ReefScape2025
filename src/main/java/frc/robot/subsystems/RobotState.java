package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Position;

public class RobotState extends SubsystemBase {
    // Supplier to dynamically get the current robot position
    private Position m_currRobotState;
    private boolean m_hasCoralInEE;
    private boolean m_hasCoralInGroundIntake;
    private boolean m_hasAlgaeInEE;

    public RobotState() {
        // Initialize with a Supplier that returns the starting position
        m_currRobotState = Position.STARTING_CONFIG;
        m_hasCoralInEE = false;
        m_hasCoralInGroundIntake = false;
        m_hasAlgaeInEE = false;
    }

    @Override
    public void periodic() {
        // Any periodic code that needs to run every cycle can be added here.
        SmartDashboard.putString("RobotState/robotEEPose", m_currRobotState.toString());
        SmartDashboard.putBoolean("RobotState/hasCoralInEE", m_hasCoralInEE);
        SmartDashboard.putBoolean("RobotState/hasCoralInGroundIntake", m_hasCoralInGroundIntake);
        SmartDashboard.putBoolean("RobotState/hasAlgaeInEE", m_hasAlgaeInEE);
        if(m_hasAlgaeInEE == true && m_hasCoralInEE == true ){
            setHasAlgaeInEEFalse();
            System.err.println("double ee err: has both algae and coral in ee, correcting to just have a coral");
        }

    }

    // Method to set a new robot state by changing the Supplier
    public void setRobotState(Position pos) {
        m_currRobotState = pos;
    }

    public void setHasCoralInEETrue(){
        m_hasCoralInEE = true;
    }

    public void setHasCoralInEEFalse(){
        m_hasCoralInEE = false;
    }

    public Boolean hasCoralInEE(){
        return m_hasCoralInEE;
    }

    public Boolean hasAlgaeInEE(){
        return m_hasAlgaeInEE;
    }

    public void setHasAlgaeInEETrue(){
        m_hasAlgaeInEE = true;
    }

    public void setHasAlgaeInEEFalse(){
        m_hasAlgaeInEE = false;
    }


    public void setHasCoralInGroundIntakeTrue(){
        m_hasCoralInGroundIntake = true;
    }

    public void setHasCoralInGroundIntakeFalse(){
        m_hasCoralInGroundIntake = false;
    }

    public Boolean hasCoralInGroundIntake(){
        return m_hasCoralInGroundIntake;
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
    public void setRobotStateL1_ALT(){
        m_currRobotState = Position.L1;
    }
    public void setRobotStateL2_ALT(){
        m_currRobotState = Position.L2_ALT;
    }
    public void setRobotStateL3_ALT(){
        m_currRobotState = Position.L3_ALT;
    }
    public void setRobotStateL4_ALT(){
        m_currRobotState = Position.L4_ALT;
    }
    public void setRobotStateBARGE_ALT(){
        m_currRobotState = Position.BARGE_ALT;
    }
    public void setRobotStateSTOW_ALT(){
        m_currRobotState = Position.STOW;
    }
    public void setRobotStateL2_PREP_ALT(){
        m_currRobotState = Position.L2_PREP;
    }
    public void setRobotStateL3_PREP_ALT(){
        m_currRobotState = Position.L3_PREP;
    }
    public void setRobotStateL4_PREP_ALT(){
        m_currRobotState = Position.L4_PREP;
    }

    public void setRobotStateL1(){
        m_currRobotState = Position.L1;
    }

    public boolean notHasCoralInGroundIntakeAndHasCoralInEE(){
        if (m_hasCoralInEE == true & m_hasCoralInGroundIntake == false){
            return true;
        }
        else{
            return false;
        }
    }
}
