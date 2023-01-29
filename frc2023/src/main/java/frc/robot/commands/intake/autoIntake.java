package frc.robot.commands.intake;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Intake.IntakeState;

public class autoIntake extends CommandBase {
    
    private Intake intake;


    public autoIntake(){

        intake = Intake.getInstance();
  
    }

    @Override
    public void execute() {
        Intake.getInstance().setIntakeState(IntakeState.OUTTAKE);
    }

    @Override
    public void end(boolean interrupted) {
        //intake.setMotors(0);
        
        Intake.getInstance().setIntakeState(IntakeState.OFF);
            
        
            

        //intake.setIntakeState(IntakeState.OFF);
    }

}
