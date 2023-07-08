package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ConeSniper;
import frc.robot.commands.IntakePortal;
import frc.robot.commands.MoveFork;
import frc.robot.commands.OpenDoor;
import frc.robot.commands.QuickDeployForks;
import frc.robot.commands.ScoreCone;
import frc.robot.commands.ScoreConeHigh;
import frc.robot.commands.ScoreConeMid;
import frc.robot.commands.ScoreCube;
import frc.robot.commands.ScoreCubeHigh;
import frc.robot.commands.ScoreCubeHighShoot;
import frc.robot.commands.Seagull;
import frc.robot.commands.SetClimbMode;
import frc.robot.commands.SetIntakingHeight;
import frc.robot.commands.SetScorePosition;
import frc.robot.commands.SetVisionMode;
import frc.robot.commands.ShootCone;
import frc.robot.commands.ShootPosition;
import frc.robot.commands.Stow;
import frc.robot.commands.WristClimb;
import frc.robot.commands.Zero;
import frc.robot.commands.auto.routines.PreloadPlusTwoWeak;
import frc.robot.commands.drive.AlignRotation;
import frc.robot.commands.drive.DriveTele;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Fork;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Tracker;
import frc.robot.subsystems.VisionManager;
import frc.robot.subsystems.SuperStructure.ScoringPosition;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
import frc.robot.subsystems.VisionManager.VisionState;
import java.util.HashMap;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  public static final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
 public static final CommandJoystick operatorController =
      new CommandJoystick(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  private final Drive drive = Drive.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Tracker tracker = Tracker.getInstance();
  private final VisionManager vision = VisionManager.getInstance();
  private final SuperStructure superStructure = SuperStructure.getInstance();
  private final Fork forks = Fork.getInstance();



  public RobotContainer() {
    configureBindings();
    drive.setDefaultCommand(new DriveTele(driverController::getLeftY, driverController::getLeftX, driverController::getRightX, drive,tracker));

  }

  private void configureBindings() {

    driverController.leftTrigger().and(operatorController.button(1).negate()).onTrue(new SetIntakingHeight(superStructure, SuperStructureState.FALLEN_CONE));
    driverController.leftTrigger().and(operatorController.button(1).negate()).onFalse(new Stow(superStructure));

    driverController.b().onTrue(new SetIntakingHeight(superStructure, SuperStructureState.CUBE_INTAKE));
    driverController.b().onFalse(new Stow(superStructure));


    driverController.povDown().onTrue(new SetIntakingHeight(superStructure, SuperStructureState.CONE_INTAKE));
    driverController.povDown().onFalse(new Stow(superStructure));

    driverController.leftStick().whileTrue(new ParallelCommandGroup(new IntakePortal(superStructure)));
    driverController.leftStick().onFalse(new Stow(superStructure));

    driverController.leftBumper().onTrue(new ScoreCone(superStructure));
    driverController.leftBumper().onFalse(new Stow(superStructure));

    driverController.rightBumper().onTrue(new ScoreCube(superStructure));
    driverController.rightBumper().onFalse(new Stow(superStructure));

    driverController.rightTrigger().and(operatorController.button(1).negate()).whileTrue(new ShootCone());
    driverController.rightTrigger().and(operatorController.button(1).negate()).onFalse(new InstantCommand(superStructure::enableIdling));
    
    driverController.rightStick().onTrue(new ConeSniper(superStructure));
    driverController.rightStick().onFalse(new Stow(superStructure));

    driverController.povUp().onTrue(new InstantCommand(drive::resetHeading));

    driverController.x().onTrue(new AlignRotation( driverController::getLeftY, driverController::getLeftX, Rotation2d.fromDegrees(-90)));
    driverController.y().onTrue(new AlignRotation(driverController::getLeftY, driverController::getLeftX, Rotation2d.fromDegrees(0)));

    driverController.povLeft().onTrue(new InstantCommand(intake::toggleJaws));

    driverController.button(7).onTrue(new Zero());

    driverController.button(8).whileTrue(new ParallelCommandGroup(new Seagull(superStructure)));
    driverController.button(8).onFalse(new Stow(superStructure));

    driverController.a().onTrue(new AlignRotation( driverController::getLeftY, driverController::getLeftX, Rotation2d.fromDegrees(180)));

    operatorController.button(7).onTrue(new SetScorePosition(ScoringPosition.HIGH));
    operatorController.button(9).onTrue(new SetScorePosition(ScoringPosition.MID));
    operatorController.button(11).onTrue(new SetScorePosition(ScoringPosition.HYBRID));
    operatorController.button(10).onTrue(new SetScorePosition(ScoringPosition.MID_SUPER));

    driverController.leftTrigger().and(operatorController.button(1)).whileTrue(new MoveFork(forks, driverController::getLeftTriggerAxis,true));
    driverController.rightTrigger().and(operatorController.button(1)).whileTrue(new SequentialCommandGroup(new QuickDeployForks(forks),new MoveFork(forks, driverController::getRightTriggerAxis,false)));

    operatorController.button(1).onTrue(new ParallelCommandGroup(new WristClimb(), new SetClimbMode(superStructure)));
  
    operatorController.button(2).onTrue(new ParallelCommandGroup(new InstantCommand(superStructure::emergencySetpointReset), new InstantCommand(intake::adjustmentReset)));

  }

  public Command getAutonomousCommand() {
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("score", new SequentialCommandGroup(new Stow(superStructure),new ScoreConeHigh(superStructure), new ShootCone(), new WaitCommand(.7)));
    eventMap.put("unstowed score cone", new SequentialCommandGroup(new Stow(superStructure), new ScoreConeHigh(superStructure), new WaitCommand(.2), new ShootCone()));
    eventMap.put("score-cone-mid", new SequentialCommandGroup(new ScoreConeMid(superStructure), new WaitCommand(.5), new ShootCone(), new WaitCommand(.4), new Stow(superStructure)));
    eventMap.put("intake-cone",new SequentialCommandGroup(new SetIntakingHeight(superStructure, SuperStructureState.FALLEN_CONE)));
    eventMap.put("intake-cube",new SequentialCommandGroup(new SetIntakingHeight(superStructure, SuperStructureState.CUBE_INTAKE)));
    eventMap.put("stow",new Stow(superStructure));
    eventMap.put("cook",new SequentialCommandGroup(new OpenDoor(superStructure), new WaitCommand(.25)));
    eventMap.put("uncook", new Stow(superStructure));
    eventMap.put("lock", new InstantCommand(drive::lockModules));
    eventMap.put("score-cube-mid", new SequentialCommandGroup(new ScoreCubeHigh(superStructure), new ShootCone(), new WaitCommand(0.3), new Stow(superStructure)));
    eventMap.put("go-to-shoot", new ShootPosition());
    eventMap.put("shoot", new ShootCone());
    eventMap.put("snipe cube high", new ScoreCubeHighShoot(superStructure));
    eventMap.put("score cube high", new SequentialCommandGroup(new Stow(superStructure), new ScoreCubeHigh(superStructure), new ShootCone(), new WaitCommand(.6)));
    eventMap.put("shoot preload", new SequentialCommandGroup(new ShootPosition(),new WaitCommand(.3),new ShootCone()));
    eventMap.put("reset", new InstantCommand(tracker::resetViaVision));
    eventMap.put("set tag", new SetVisionMode(vision, VisionState.TAG));
    eventMap.put("set cube", new SetVisionMode(vision, VisionState.CUBE));

    HashMap<String, Command> climbMap = new HashMap<>();
    climbMap.put("score", new SequentialCommandGroup(new Stow(superStructure),new WaitCommand(.2),new ScoreConeHigh(superStructure), new WaitCommand(0.75), new ShootCone(), new WaitCommand(.3), new Stow(superStructure)));
    climbMap.put("score", new SequentialCommandGroup(new Stow(superStructure),new WaitCommand(.25),new ScoreConeHigh(superStructure), new WaitCommand(0.55), new ShootCone(), new WaitCommand(.5), new Stow(superStructure)));
    climbMap.put("cook",new SequentialCommandGroup(new OpenDoor(superStructure), new WaitCommand(.5)));
    climbMap.put("uncook", new Stow(superStructure));
    climbMap.put("lock", new InstantCommand(drive::lockModules));

    drive.setLastAlliance(DriverStation.getAlliance());
    return new PreloadPlusTwoWeak(drive, vision, superStructure, tracker);
  }

  }
