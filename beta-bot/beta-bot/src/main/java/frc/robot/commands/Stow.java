package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class Stow extends CommandBase {
    private SuperStructure superStructure;

    public Stow(SuperStructure instance){
        superStructure = instance;
    }

    @Override
    public void initialize() {
        superStructure.setState(SuperStructureState.STOWED);
    }

    @Override
    public boolean isFinished() {
        return superStructure.atSetpoint();
    }
}
