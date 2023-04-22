package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ScoreConeHigh;
import frc.robot.commands.ShootCone;
import frc.robot.commands.Stow;
import frc.robot.subsystems.SuperStructure;

public class Preload extends SequentialCommandGroup {
    
    public Preload(SuperStructure superStructure){
        addCommands(
        new ScoreConeHigh(superStructure),
        new WaitCommand(.4),
        new ShootCone(),
        new WaitCommand(0.5),
        new Stow(superStructure)
        );

    }
}
