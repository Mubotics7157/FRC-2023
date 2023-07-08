package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Tracker;

public class SetVisionDeviations extends InstantCommand{
    private double x, y, rot;

    public SetVisionDeviations(double x, double y, double rot){
        this.x = x;
        this.y = y;
        this.rot = rot;
    }

    @Override
    public void execute() {
        Tracker.getInstance().setVisionDeviations(x, y, rot);
    }
}
