package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

public class RunIntake extends CommandBase {
    
    private Intake intake;
    private boolean reverse;
    private String objectType;

    public RunIntake(boolean reverse, Intake instance, String objectType){
        this.reverse = reverse;

        intake = instance;

        this.objectType = objectType;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        if(objectType== "cones"){

        if (reverse && Wrist.getInstance().atSetpoint()){
            SmartDashboard.putString("running?", "reverse");
            intake.setMotors( "cones", -SmartDashboard.getNumber("Intake Speed", .5));
        }
        else if (!reverse && Wrist.getInstance().atSetpoint()){
            intake.setMotors("cones", SmartDashboard.getNumber("Intake Speed", .5));
            SmartDashboard.putString("running?", "forward");
        }
    }

        else if(objectType== "cubes"){

            if (reverse && Wrist.getInstance().atSetpoint()){
                SmartDashboard.putString("running?", "reverse");
                intake.setMotors( "cubes", -SmartDashboard.getNumber("Intake Speed", .5));
            }
            else if (!reverse && Wrist.getInstance().atSetpoint()){
                intake.setMotors("cubes", SmartDashboard.getNumber("Intake Speed", .5));
                SmartDashboard.putString("running?", "forward");
            }
    }
    }

    @Override
    public void end(boolean interrupted) {
        if(reverse && objectType == "cones"){
            intake.setMotors("cones", .05);
            //intake.currentLimit(true);
        }
        else{
            intake.setMotors("cones", 0);
            //intake.currentLimit(false);
        }
        SmartDashboard.putString("running?", "no");
    }

}
