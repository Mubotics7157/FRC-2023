package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class OpenDoor extends CommandBase {
    private SuperStructure superStructure;
    Timer timer = new Timer();
    private double time;

    public OpenDoor(SuperStructure instance, double time){
        superStructure = instance;
        this.time = time;
        addRequirements(superStructure);
    }

    @Override
    public void initialize() {
        superStructure.setState(SuperStructureState.OPEN_DOOR);
        
    }

    @Override
    public boolean isFinished() {
        return true;//Wrist.getInstance().atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
      
    }
}
