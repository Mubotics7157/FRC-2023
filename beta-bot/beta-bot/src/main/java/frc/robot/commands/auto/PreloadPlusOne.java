package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ScoreConeHigh;
import frc.robot.commands.ShootCone;
import frc.robot.commands.Stow;
import frc.robot.commands.drive.AlignObject;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.VisionManager;

public class PreloadPlusOne extends SequentialCommandGroup{

    public PreloadPlusOne(Drive drive,VisionManager vision, SuperStructure superStructure){
        PathPlannerTrajectory driveToCube = PathPlanner.loadPath("Preload Plus One Part One", 3,4);

        addCommands(
         new Stow(superStructure),
         new ScoreConeHigh(superStructure),
         new ShootCone(),
         new WaitCommand(.2),
         new Stow(superStructure),
         drive.followPath(driveToCube),
         new AlignObject(drive, vision)

        );
    }
    
}
