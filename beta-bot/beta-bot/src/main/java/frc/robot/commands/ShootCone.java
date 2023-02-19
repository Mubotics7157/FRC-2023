package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Intake.IntakeState;

public class ShootCone extends CommandBase {


    @Override
    public void initialize() {
        SuperStructure.getInstance().shoot(true);
    }
    

}
