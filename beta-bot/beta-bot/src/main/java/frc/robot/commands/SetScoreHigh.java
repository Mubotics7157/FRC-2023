package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SuperStructure;

public class SetScoreHigh extends InstantCommand {

    private boolean scoreHigh;

    public SetScoreHigh(boolean scoreHigh){
        this.scoreHigh = scoreHigh;
    }

    @Override
    public void initialize() {
        SuperStructure.getInstance().setScorePosition(scoreHigh);       
    }
    
}
