package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Tracker;

public class AutoRoutine {
    private HashMap<String, Command> events;
    private ArrayList<PathPlannerTrajectory> pathGroup;
    private Tracker tracker;
    private Drive drive;

    public AutoRoutine(String autoPath, PathConstraints constraints, HashMap<String,Command> events){
        pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(autoPath,constraints);
        this.events = events;
        tracker = Tracker.getInstance();
        drive = Drive.getInstance();
    }

    public Command buildAuto(){
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
          tracker::getOdometry, 
          tracker::setOdometry, 
          DriveConstants.DRIVE_KINEMATICS, 
          AutoConstants.X_Y_CONTROLLER,
          AutoConstants.ROT_CONTROLLER,
          drive::setModuleStates,
          events,
          false,
          drive
          ); 

        return autoBuilder.fullAuto(pathGroup);
    }
}
