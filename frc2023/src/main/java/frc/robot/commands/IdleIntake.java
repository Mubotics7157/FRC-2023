package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IdleIntake extends CommandBase {

    private Intake intake;


    public IdleIntake(Intake instance){
        intake = instance;
    }

    @Override
    public void initialize() {
        intake.currentLimit(true);
    }
    
    @Override
    public void execute() {
        intake.setMotors(.05);
    }

    @Override
    public void end(boolean interrupted) {
        intake.currentLimit(false);
    }
}
