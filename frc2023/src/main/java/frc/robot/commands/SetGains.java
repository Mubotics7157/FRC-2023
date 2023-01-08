package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class SetGains extends CommandBase{
    Drive drive;


    public SetGains(Drive instance){
        drive = instance;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.setGains(
            SmartDashboard.getNumber("Module P Gain", 0),
            SmartDashboard.getNumber("Module D Gain", 0)
            );
    }

    @Override
    public void end(boolean interrupted) {
        
    }
}
