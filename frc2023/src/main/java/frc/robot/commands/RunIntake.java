package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class RunIntake extends CommandBase {
    
    private Intake intake;
    private boolean reverse;
    private Rotation2d angle;

    public RunIntake(boolean reverse, Intake instance, Rotation2d jawAngle){
        this.reverse = reverse;

        intake = instance;

        angle = jawAngle;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        if(!reverse)
            intake.runIntake(SmartDashboard.getNumber("Intake Speed", 0));
        else
            intake.runIntake(-SmartDashboard.getNumber("Intake Speed", 0));

        intake.setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("Intake Angle Degrees", 0)));
    }

    @Override
    public void end(boolean interrupted) {
        intake.runIntake(0);
    }

}
