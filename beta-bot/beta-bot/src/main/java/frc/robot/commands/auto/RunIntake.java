package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Intake.IntakeState;

public class RunIntake extends CommandBase {
    
    private Intake intake;
    private IntakeState state;

    public RunIntake(IntakeState state){
        this.state = state;
    }

    @Override
    public void execute() {
        intake = Intake.getInstance();
        intake.setIntakeState(state); 
    }

    @Override
    public void end(boolean interrupted) {
        //intake.setMotors(0);
        
        if(state == IntakeState.INTAKE || state == IntakeState.INTAKE_CONE || state == IntakeState.INTAKE_CUBE)
            intake.setIntakeState(IntakeState.IDLE);
        else
            intake.setIntakeState(IntakeState.OFF);
            
        
            

    }

}
