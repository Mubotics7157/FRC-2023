package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class ShootCube extends InstantCommand {
   
    @Override
    public void initialize() {
        SuperStructure.getInstance().setState(SuperStructureState.CUBE_HIGH);
    }
}
