package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

public class DriveTele extends CommandBase {

    private Drive drive;
    private DoubleSupplier left;
    private DoubleSupplier right;

    /* 
    private void applyDeadband(){
        if(Math.abs(left.getAsDouble())<.2)
            left.getAsDouble() = 0;
        if(Math.abs(right.getAsDouble())<.2)
            right.getAsDouble() = 0;
    }
    */

    public DriveTele(DoubleSupplier l, DoubleSupplier r, Drive instance){
        left = l;
        right = r;
        drive = instance;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        //applyDeadband();
        drive.drive(left.getAsDouble(), right.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        drive.drive(0,0);
    }

    
}
