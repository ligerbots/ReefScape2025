package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.kitbot.KitbotRoller;

public class ScoreCommandKitBot extends StartEndCommand {
    public ScoreCommandKitBot(KitbotRoller roller) {
        super(roller::runRollerOut, roller::stop, roller);
    }
}
