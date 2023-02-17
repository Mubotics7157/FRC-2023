package frc.robot.commands.drive;

import java.util.HashMap;
import java.util.function.DoubleSupplier;
import java.util.stream.Stream;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Tracker;
import frc.robot.subsystems.VisionManager;

public class AlignStrafe extends CommandBase{
    private Drive drive;
    private Tracker tracker;
    private Command run;


    public AlignStrafe(Drive dInstance,Tracker tracker){
        drive = dInstance;
        this.tracker = tracker;
        addRequirements(drive);
        
        addRequirements(tracker);
        //tracker.regeneratePath();
        //addCommands(tracker.getCommand());
    }

    @Override
    public void initialize() {
        tracker.regeneratePath();
        run = tracker.getCommand();
        run.schedule();
    }

    @Override
    public void end(boolean interrupted) {
        run.end(true);
    }
    




  

}
