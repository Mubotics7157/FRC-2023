package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class Intake extends CommandBase {
    private SuperStructure superStructure;
    private boolean intakeCone;

    public Intake(SuperStructure instance,boolean intakeCone){
        superStructure = instance;
        this.intakeCone = intakeCone;

        addRequirements(superStructure);
    }

    @Override
    public void initialize() {
        if(intakeCone)
            superStructure.setState(SuperStructureState.FALLEN_CONE);
        else
            superStructure.setState(SuperStructureState.CUBE_INTAKE);
    }

    @Override
    public void end(boolean interrupted) {
        superStructure.setState(SuperStructureState.STOWED);
    }
}
