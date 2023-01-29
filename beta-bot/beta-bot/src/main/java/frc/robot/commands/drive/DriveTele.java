package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveTele extends CommandBase {

    private Drive drive;
    private double left;
    private double right;

    private void applyDeadband(){
        if(Math.abs(left)<.2)
            left = 0;
        if(Math.abs(right)<.2)
            right = 0;
    }

    public DriveTele(DoubleSupplier l, DoubleSupplier r, Drive instance){
        left = l.getAsDouble();
        right = r.getAsDouble();
        drive = instance;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        applyDeadband();
        drive.drive(left, right);
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0,0);
    }

    
}
