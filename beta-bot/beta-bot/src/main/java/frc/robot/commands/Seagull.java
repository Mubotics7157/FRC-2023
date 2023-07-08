package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class Seagull extends CommandBase{
      
    private SuperStructure superStructure;

    public Seagull(SuperStructure instance){

        superStructure = instance;

        addRequirements(superStructure);
    }

    @Override
    public void initialize() {
        superStructure.setState(SuperStructureState.SEAGULL);
    }

    @Override
    public boolean isFinished() {
        return superStructure.atSetpoint();       
    }

}
