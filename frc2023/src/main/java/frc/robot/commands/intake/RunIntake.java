package frc.robot.commands.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class RunIntake extends CommandBase {
    
    private Intake intake;
    private boolean reverse;
    private Rotation2d angle;
    private boolean useSD;

    public RunIntake(boolean reverse, Intake instance, Rotation2d jawAngle,boolean useSD){
        this.reverse = reverse;

        intake = instance;

        angle = jawAngle;

        this.useSD = useSD;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        if(!reverse)
            intake.runIntake(SmartDashboard.getNumber("Intake Speed", 0));
        else
            intake.runIntake(-SmartDashboard.getNumber("Intake Speed", 0));

        if(useSD)
            intake.setAngle(Rotation2d.fromDegrees(SmartDashboard.getNumber("Intake Angle Degrees", 0)));
        else
            intake.setAngle(angle);
    }

    @Override
    public void end(boolean interrupted) {
        intake.runIntake(0);
    }

}
