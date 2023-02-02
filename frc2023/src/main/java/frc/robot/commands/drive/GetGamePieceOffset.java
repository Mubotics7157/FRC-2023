package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.VisionManager;

public class GetGamePieceOffset extends CommandBase {
    private VisionManager vision;
    private boolean onPipeline;
    private double offset = 200;

    public GetGamePieceOffset(VisionManager instance){
        vision = instance;

        addRequirements(vision);
    }

    @Override
    public void initialize() {
        vision.changePipeline(1);
    }

    @Override
    public void execute() {
        onPipeline = vision.getPipelineIndex() == 1.0;

        if(onPipeline && vision.hasTargets()){
            offset = vision.saveObjectOffset();
        }

        
    }

    @Override
    public boolean isFinished() {
        return offset!=200;
    }

    @Override
    public void end(boolean interrupted) {
        vision.changePipeline(0);
    }
    
}
