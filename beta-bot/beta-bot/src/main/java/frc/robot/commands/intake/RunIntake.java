package frc.robot.commands.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Intake.IntakeState;

public class RunIntake extends InstantCommand {
    
    private Intake intake;
    private IntakeState state;

    public RunIntake(Intake instance, IntakeState state){

        intake = instance;

        this.state = state;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeState(state); 

    }
    
    @Override
    public void end(boolean interrupted) {
        //intake.setMotors(0);
        
       // if(state == IntakeState.INTAKE || state == IntakeState.INTAKE_CONE || state == IntakeState.INTAKE_CUBE)
         //   intake.setIntakeState(IntakeState.IDLE);
        //else
            //intake.setIntakeState(IntakeState.OFF);
        
    }

}
