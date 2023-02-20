package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class CustomSetpoints extends CommandBase{

    private SuperStructure superStructure;

    public CustomSetpoints(SuperStructure instance){

        superStructure = instance;

        addRequirements(superStructure);
    }

    @Override
    public void initialize() {
        superStructure.setState(SuperStructureState.CUSTOM);
        Intake.getInstance().setIntakeState(IntakeState.IDLE);
    }

    @Override
    public boolean isFinished() {
        return superStructure.atSetpoint();       
    }
}
