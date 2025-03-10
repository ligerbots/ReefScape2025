// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class Rumble extends Command {
    XboxController m_xbox;
    boolean m_rumbling;

    /** Creates a new Rumble. */
    public Rumble(XboxController xbox) {
        // Use addRequirements() here to declare subsystem dependencies.
        m_xbox = xbox;
        m_rumbling = false;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // stop rumbling
        m_xbox.setRumble(RumbleType.kBothRumble, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public void start(int value) {
        // start rumbling
        m_xbox.setRumble(RumbleType.kBothRumble, value);
    }

    public void stop() {
        m_xbox.setRumble(RumbleType.kBothRumble, 0);
    }
}