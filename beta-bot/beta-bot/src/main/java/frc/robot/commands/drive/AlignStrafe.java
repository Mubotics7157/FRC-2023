package frc.robot.commands.drive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Tracker;

public class AlignStrafe extends CommandBase{
    private Drive drive;
    private Tracker tracker;
    private Command run;


    public AlignStrafe(Drive dInstance,Tracker tracker){
        drive = dInstance;
        this.tracker = tracker;
        addRequirements(drive);
        addRequirements(tracker);
    }

    @Override
    public void initialize() {
        tracker.regeneratePath();
        run = tracker.getPathFollowingCommand();
        run.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        run.end(true);
    }
    




  

}
