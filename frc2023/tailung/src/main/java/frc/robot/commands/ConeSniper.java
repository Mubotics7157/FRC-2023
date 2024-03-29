package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class ConeSniper extends CommandBase{
      
    private SuperStructure superStructure;

    public ConeSniper(SuperStructure instance){

        superStructure = instance;

        addRequirements(superStructure);
    }

    @Override
    public void initialize() {
        Drive.getInstance().changeMax();
        superStructure.setState(SuperStructureState.CONE_SNIPER);
    }

    @Override
    public boolean isFinished() {
        return superStructure.atSetpoint();       
    }

}
