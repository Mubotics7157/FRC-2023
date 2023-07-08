package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SuperStructure;

public class ShootCone extends InstantCommand {


    @Override
    public void initialize() {
        SuperStructure.getInstance().shoot();
    }
    

}
