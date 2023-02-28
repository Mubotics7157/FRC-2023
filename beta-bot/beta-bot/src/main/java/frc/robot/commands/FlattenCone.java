package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class FlattenCone extends SequentialCommandGroup{

    private frc.robot.subsystems.Intake intake;

    public FlattenCone(frc.robot.subsystems.Intake instance){
        intake = instance;

        addCommands(
            new InstantCommand(intake::openJaws),
            new WaitCommand(0.25),
            new InstantCommand(intake::closeJaws)
            );
    }
    


}