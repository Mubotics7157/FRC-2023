package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class ScoreCubeHighShoot extends CommandBase {
    private SuperStructure superStructure;

    public ScoreCubeHighShoot(SuperStructure instance){

        superStructure = instance;

        addRequirements(superStructure);
    }

    @Override
    public void initialize() {
        superStructure.setState(SuperStructureState.CUBE_HIGH_SHOOT);
    }

    @Override
    public boolean isFinished() {
        return superStructure.atSetpoint();       
    }
}
