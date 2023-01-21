package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.VisionManager;

public class Align extends CommandBase{
    private Drive drive;
    private VisionManager vision;


    public Align(Drive dInstance, VisionManager vInstance){
        drive = dInstance;
        vision = vInstance;
        addRequirements(drive);
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
