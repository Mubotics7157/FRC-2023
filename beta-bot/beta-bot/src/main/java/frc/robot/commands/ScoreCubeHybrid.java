package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class ScoreCubeHybrid extends CommandBase{
      
    private SuperStructure superStructure;

    public ScoreCubeHybrid(SuperStructure instance){

        superStructure = instance;

        addRequirements(superStructure);
    }

    @Override
    public void initialize() {
        superStructure.setState(SuperStructureState.CUBE_HYBRID);
    }

    @Override
    public boolean isFinished() {
        return superStructure.atSetpoint();       
    }

    @Override
    public void end(boolean interrupted) {
    }

}
