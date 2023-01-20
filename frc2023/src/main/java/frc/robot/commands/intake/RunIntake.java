package frc.robot.commands.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Intake.IntakeState;

public class RunIntake extends CommandBase {
    
    private Intake intake;
    private double val;

    public RunIntake(Intake instance, double val){

        intake = instance;

        this.val = val;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setMotors(val);
    }

    @Override
    public void end(boolean interrupted) {
        
        intake.setMotors(0);

    }

}
