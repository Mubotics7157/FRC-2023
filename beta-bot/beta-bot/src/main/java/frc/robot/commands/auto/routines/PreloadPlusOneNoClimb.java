package frc.robot.commands.auto.routines;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ScoreConeHigh;
import frc.robot.commands.ScoreCubeHigh;
import frc.robot.commands.SetIntakingHeight;
import frc.robot.commands.SetVisionMode;
import frc.robot.commands.ShootCone;
import frc.robot.commands.Stow;
import frc.robot.commands.auto.DriveBackwards;
import frc.robot.commands.drive.AlignObject;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Tracker;
import frc.robot.subsystems.VisionManager;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.VisionManager.VisionState;

public class PreloadPlusOneNoClimb extends SequentialCommandGroup{

    public 
    PreloadPlusOneNoClimb(Drive drive,VisionManager vision, SuperStructure superStructure,Tracker tracker){
        PathPlannerTrajectory driveToCube = PathPlanner.loadPath("PL + intake", 4,4);
        PathPlannerTrajectory driveToCubeNodeOne = PathPlanner.loadPath("PreloadPlusThreePart2", 4,3);
        vision.setTargetLLState(VisionState.CUBE);

        addCommands(
 
            new SetVisionMode(vision, VisionState.CUBE),
            new ScoreConeHigh(superStructure),
            new ShootCone(),
            new WaitCommand(.35),
            new Stow(superStructure),
            new ParallelCommandGroup(drive.followPath(driveToCube,true),new SequentialCommandGroup(new WaitCommand(1.25),new SetIntakingHeight(superStructure, SuperStructureState.CUBE_INTAKE))).andThen(new ParallelCommandGroup(new AlignObject(drive, vision)),
            new DriveBackwards( 1.7, drive, tracker,PathPlannerTrajectory.transformTrajectoryForAlliance(driveToCubeNodeOne, DriverStation.getAlliance()).getInitialHolonomicPose(),1.5)),
            new ParallelCommandGroup(new Stow(superStructure).andThen(new SetVisionMode(vision, VisionState.TAG)),drive.followPath(driveToCubeNodeOne,false)),
            new SequentialCommandGroup(new ScoreCubeHigh(superStructure), new ShootCone(), new WaitCommand(.4), new Stow(superStructure))
        );
    }

    

    
    
}
