package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ScoreConeHigh;
import frc.robot.commands.ScoreCubeHigh;
import frc.robot.commands.SetIntakingHeight;
import frc.robot.commands.ShootCone;
import frc.robot.commands.Stow;
import frc.robot.commands.drive.AlignObject;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Tracker;
import frc.robot.subsystems.VisionManager;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.VisionManager.VisionState;

public class PreloadPlusOne extends SequentialCommandGroup{

    public PreloadPlusOne(Drive drive,VisionManager vision, SuperStructure superStructure,Tracker tracker){
        PathPlannerTrajectory driveToCube = PathPlanner.loadPath("PL + intake", 3,4);
        PathPlannerTrajectory driveToCubeNodeOne = PathPlanner.loadPath("PreloadPlusThreePart2", 3,4);
        vision.setTargetLLState(VisionState.CUBE);

        addCommands(
        //Commands.runOnce(vision::togglePipeline, vision),
         new Stow(superStructure),
         new ScoreConeHigh(superStructure),
         new ShootCone(),
         new WaitCommand(.2),
         new Stow(superStructure),
         new ParallelCommandGroup(drive.followPath(driveToCube,true),new SequentialCommandGroup(new WaitCommand(.7),new SetIntakingHeight(superStructure, SuperStructureState.CUBE_INTAKE))).andThen(new ParallelCommandGroup(new AlignObject(drive, vision)),
         new DriveBackwards( 1, drive, tracker)),
         new ParallelCommandGroup(new Stow(superStructure),drive.followPath(driveToCubeNodeOne,false)),
         new SequentialCommandGroup(new ScoreCubeHigh(superStructure), new ShootCone(), new WaitCommand(.6))
        );
    }
    
}
