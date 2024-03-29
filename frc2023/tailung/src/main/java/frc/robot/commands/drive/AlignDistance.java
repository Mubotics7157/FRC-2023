package frc.robot.commands.drive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Tracker;

public class AlignDistance extends CommandBase{
    private Drive drive;
    private Tracker tracker;
    private Command run;


    public AlignDistance(Drive dInstance,Tracker tracker){
        drive = dInstance;
        this.tracker = tracker;
        addRequirements(drive);
        addRequirements(tracker);
    }

    @Override
    public void initialize() {
        tracker.regenDistancePath();
        run = tracker.getPathFollowingCommand();
        run.schedule();
    }

    @Override
    public void execute() {
        if(Math.abs(RobotContainer.driverController.getLeftY())>.2||Math.abs(RobotContainer.driverController.getLeftX())>.2){
            this.end(true);
        }
    }

    @Override
    public boolean isFinished() {
        return run.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        run.end(true);
    }
    




  

}
