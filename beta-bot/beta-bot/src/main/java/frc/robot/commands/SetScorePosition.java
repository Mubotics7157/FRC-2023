package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.ScoringPosition;

public class SetScorePosition extends InstantCommand {

    private ScoringPosition position;

    public SetScorePosition(ScoringPosition position){
        this.position = position;
    }

    @Override
    public void initialize() {
        SuperStructure.getInstance().setScorePosition(position);

            if(position == ScoringPosition.HIGH){
                SmartDashboard.putBoolean("score High?", true);   
                SmartDashboard.putBoolean("score Mid?", false);  
                SmartDashboard.putBoolean("score Hybrid?", false);  
            }  
            else if(position == ScoringPosition.MID){
                SmartDashboard.putBoolean("score High?", false);   
                SmartDashboard.putBoolean("score Mid?", true);  
                SmartDashboard.putBoolean("score Hybrid?", false);  
            }  
            else {
                SmartDashboard.putBoolean("score High?", false);   
                SmartDashboard.putBoolean("score Mid?", false);  
                SmartDashboard.putBoolean("score Hybrid?", true);  
            }
    }
    
}
