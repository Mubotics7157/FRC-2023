package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class MoveJaw extends CommandBase {
    private double val;
    private Intake intake;

    public MoveJaw(double val, Intake instance){
        this.val = val;
        intake = instance;

        addRequirements(instance);
    }

    @Override
    public void execute() {
        intake.jogJawMotor(val);
    }

    @Override
    public void end(boolean interrupted) {
        intake.jogJawMotor(0);
    }
}
