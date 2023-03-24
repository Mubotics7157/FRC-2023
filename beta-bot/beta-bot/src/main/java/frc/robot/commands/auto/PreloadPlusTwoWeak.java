package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
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

public class PreloadPlusTwoWeak extends SequentialCommandGroup{

    public PreloadPlusTwoWeak(Drive drive,VisionManager vision, SuperStructure superStructure,Tracker tracker){
        PathPlannerTrajectory driveToCube = PathPlanner.loadPath("PreloadPlusTwoWeakSidePart1", 1,1);
        PathPlannerTrajectory driveToCubeNodeOne = PathPlanner.loadPath("PreloadPlusTwoWeakSidePart2", 1,1);
        PathPlannerTrajectory driveToCubeTwo = PathPlanner.loadPath("PreloadPlusTwoWeakSidePart3", 1,1);
        PathPlannerTrajectory driveToCubeNodeTwo = PathPlanner.loadPath("PreloadPlusTwoWeakSidePart4", 1,1);
        vision.setTargetLLState(VisionState.CUBE);

        addCommands(
        new Stow(superStructure),
         new ScoreConeHigh(superStructure),
         new ShootCone(),
         new WaitCommand(.2),
         new Stow(superStructure),
         new ParallelCommandGroup(drive.followPath(driveToCube,true),new SequentialCommandGroup(new WaitCommand(4),new SetIntakingHeight(superStructure, SuperStructureState.CUBE_INTAKE))).andThen(new ParallelCommandGroup(new AlignObject(drive, vision)),
         new DriveBackwards( .36, drive, tracker,PathPlannerTrajectory.transformTrajectoryForAlliance(driveToCubeNodeOne, DriverStation.getAlliance()).getInitialHolonomicPose())),
         new ParallelCommandGroup(new Stow(superStructure),drive.followPath(driveToCubeNodeOne,false)),
         new SequentialCommandGroup(new ScoreCubeHigh(superStructure), new ShootCone(), new WaitCommand(.6), new Stow(superStructure)),
         new ParallelCommandGroup(drive.followPath(driveToCubeTwo, false),new SequentialCommandGroup(new WaitCommand(4).andThen(new SetIntakingHeight(superStructure, SuperStructureState.CUBE_INTAKE)))),
         new DriveBackwards(.36, drive, tracker, PathPlannerTrajectory.transformTrajectoryForAlliance(driveToCubeNodeTwo, DriverStation.getAlliance()).getInitialHolonomicPose()),
         new Stow(superStructure),
         new SequentialCommandGroup(drive.followPath(driveToCubeNodeTwo, false), new ScoreCubeHigh(superStructure), new WaitCommand(.5), new ShootCone(), new Stow(superStructure))
        );
    }

    

    
    
}

