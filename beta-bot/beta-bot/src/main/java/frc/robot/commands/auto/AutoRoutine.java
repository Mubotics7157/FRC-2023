package frc.robot.commands.auto;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
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
        tracker.plotAuto(pathGroup.get(0));
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
          tracker::getOdometry, // Pose2d supplier
          tracker::setOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
          DriveConstants.DRIVE_KINEMATICS, // SwerveDriveKinematics
          AutoConstants.X_Y_CONTROLLER,
          AutoConstants.ROT_CONTROLLER,
          drive::setModuleStates, // Module states consumer used to output to the drive subsystem
          events,
          false, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          drive // The drive subsystem. Used to properly set the requirements of path following commands
          ); 

          return autoBuilder.fullAuto(pathGroup);

    }



     
}
