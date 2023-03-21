package frc.robot.commands.auto;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.OpenDoor;
import frc.robot.commands.ResetDrivePose;
import frc.robot.commands.ScoreConeHigh;
import frc.robot.commands.ScoreCubeHigh;
import frc.robot.commands.ScoreCubeHybrid;
import frc.robot.commands.ScoreCubeMid;
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

public class PreloadPlusTwo extends SequentialCommandGroup{

    public PreloadPlusTwo(Drive drive,VisionManager vision, SuperStructure superStructure,Tracker tracker){
        PathPlannerTrajectory driveToCube = PathPlanner.loadPath("PL + intake", 3,4);
        PathPlannerTrajectory driveToCubeNodeOne = PathPlanner.loadPath("PreloadPlusThreePart2", 3,4);
        PathPlannerTrajectory driveToSecondCube = PathPlanner.loadPath("preloadplustwoclimbpart3",3,4);
        PathPlannerTrajectory driveToChargeStation = PathPlanner.loadPath("preloadplustwoclimbpart5",3,4);
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
         new SequentialCommandGroup(new ScoreCubeHigh(superStructure), new ShootCone(), new WaitCommand(.4)),
         new ParallelCommandGroup(new Stow(superStructure).andThen(new SequentialCommandGroup(new WaitCommand(.75),new SetIntakingHeight(superStructure, SuperStructureState.CUBE_INTAKE))),
          drive.followPath(driveToSecondCube, false)),
         new AlignObject(drive, vision).andThen(new DriveBackwards(.3, drive, tracker)),
         new ParallelCommandGroup(new ScoreCubeMid(superStructure),drive.followPath(driveToChargeStation, false).andThen(new ShootCone()))

        );
    }
    
}