package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class ScoreConeMid extends CommandBase{

    private SuperStructure superStructure;

    public ScoreConeMid(SuperStructure instance){

        superStructure = instance;

        addRequirements(superStructure);
    }

    @Override
    public void initialize() {
        superStructure.setState(SuperStructureState.CONE_MID);
    }

    @Override
    public boolean isFinished() {
        return superStructure.atSetpoint();       
    }
}
