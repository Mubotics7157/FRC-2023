package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class CustomSetpoints extends CommandBase{

    private SuperStructure superStructure;
    private boolean isIntaking;

    public CustomSetpoints(SuperStructure instance, boolean isIntaking){

        superStructure = instance;
        this.isIntaking = isIntaking;

        addRequirements(superStructure);
    }

    @Override
    public void initialize() {
        superStructure.setState(SuperStructureState.CUSTOM);
        
        if(isIntaking)
            Intake.getInstance().setIntakeState(IntakeState.INTAKE_CONE);
    }

    @Override
    public boolean isFinished() {
        return superStructure.atSetpoint();       
    }
}
