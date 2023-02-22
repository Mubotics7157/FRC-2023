package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Intake.IntakeState;

public class ShootCone extends InstantCommand {


    @Override
    public void initialize() {
        SuperStructure.getInstance().shoot();
    }
    

}
