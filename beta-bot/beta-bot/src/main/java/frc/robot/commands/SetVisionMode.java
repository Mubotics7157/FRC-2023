package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.VisionManager;
import frc.robot.subsystems.VisionManager.VisionState;

public class SetVisionMode extends InstantCommand {

    private VisionManager vision;
    private VisionState wantedState;

    public SetVisionMode(VisionManager vision,VisionState wantedState){
        this.vision = vision;
        this.wantedState= wantedState;
        addRequirements(vision);
    }

    @Override
    public void execute() {
        vision.setTargetLLState(wantedState);
    }
    
}
