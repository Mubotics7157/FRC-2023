package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Tracker;

public class ChangeNode extends InstantCommand{
    double nodeY;

    public ChangeNode(double nodeY){
        this.nodeY = nodeY;
    }

    @Override
    public void initialize() {
        Tracker.getInstance().editNodePose(nodeY);
    }


}
