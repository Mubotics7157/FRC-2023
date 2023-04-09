package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Tracker;

public class ResetDrivePose extends InstantCommand {
    private Pose2d pose;
    private Tracker tracker;

    public ResetDrivePose(Pose2d pose, Tracker tracker){
        this.tracker = tracker;
        this.pose = pose;
    }

    @Override
    public void execute() {
        tracker.setPose(pose);
    }
    
}
