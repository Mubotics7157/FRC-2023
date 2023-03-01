package frc.robot;

import frc.robot.AltConstants.FieldConstants.RedConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ConeSniper;
import frc.robot.commands.CustomSetpoints;
import frc.robot.commands.OpenDoor;
import frc.robot.commands.ScoreConeHigh;
import frc.robot.commands.ScoreConeMid;
import frc.robot.commands.ScoreCubeHigh;
import frc.robot.commands.Seagul;
import frc.robot.commands.SetIntakeState;
import frc.robot.commands.SetIntakingHeight;
import frc.robot.commands.SetScoreHigh;
import frc.robot.commands.ShootCone;
import frc.robot.commands.ShootCube;
import frc.robot.commands.ShootPosition;
import frc.robot.commands.Stow;
import frc.robot.commands.Zero;
import frc.robot.commands.auto.AutoRoutine;
import frc.robot.commands.drive.AlignStrafe;
import frc.robot.commands.drive.ChangeNode;
import frc.robot.commands.drive.DriveTele;
import frc.robot.commands.drive.HorizontalLock;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.Tracker;
import frc.robot.subsystems.VisionManager;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  public static final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
 private final CommandJoystick m_operatorController =
      new CommandJoystick(OperatorConstants.kOperatorControllerPort);

  private final Drive drive = Drive.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Tracker tracker = Tracker.getInstance();
  //private final VisionManager poleCam = VisionManager.getInstance();
  //private final LED led = LED.getInstance()
  private final SuperStructure superStructure = SuperStructure.getInstance();

  public RobotContainer() {
    configureBindings();
    drive.setDefaultCommand(new ParallelCommandGroup(new DriveTele(m_driverController::getLeftY, m_driverController::getLeftX, m_driverController::getRightX, drive), new InstantCommand(drive::changeMax)));

  }

  private void configureBindings() {

    m_driverController.leftTrigger().whileTrue(new SetIntakingHeight(superStructure, SuperStructureState.FALLEN_CONE));
    m_driverController.leftTrigger().onFalse(new Stow(superStructure));

    m_driverController.b().whileTrue(new SetIntakingHeight(superStructure, SuperStructureState.CUBE_INTAKE));
    m_driverController.b().onFalse(new Stow(superStructure));

    m_driverController.y().whileTrue(new SetIntakingHeight(superStructure, SuperStructureState.CUBE_HIGH));
    m_driverController.y().onFalse(new Stow(superStructure));

    m_driverController.a().whileTrue(new SetIntakingHeight(superStructure, SuperStructureState.CONE_INTAKE));
    m_driverController.a().onFalse(new Stow(superStructure));

    //m_driverController.x().whileTrue(new ConeSniper(superStructure));
    //m_driverController.x().onFalse(new Stow(superStructure));
    m_driverController.x().onTrue(new InstantCommand(tracker::resetViaVision));

    m_driverController.leftBumper().whileTrue(new ScoreConeHigh(superStructure));
    m_driverController.leftBumper().onFalse(new Stow(superStructure));

    m_driverController.rightBumper().onTrue(new ScoreConeMid(superStructure));
    m_driverController.rightBumper().onFalse(new Stow(superStructure));

    m_driverController.rightTrigger().whileTrue(new ShootCone());
    m_driverController.rightTrigger().onFalse(new Stow(superStructure));
    
    m_driverController.povDown().whileTrue(new AlignStrafe(drive, tracker));
    //m_driverController.povDown().whileTrue(new SetIntakingHeight(superStructure, SuperStructureState.CUBE_MID));
    //m_driverController.povDown().onFalse(new Stow(superStructure));

    m_driverController.povUp().onTrue(new InstantCommand(drive::resetHeading));
    //m_driverController.povRight().onTrue(new ParallelCommandGroup(new InstantCommand(intake::closeJaws), new InstantCommand(led::setYellow)));
    //m_driverController.povLeft().onTrue(new ParallelCommandGroup(new InstantCommand(intake::openJaws), new InstantCommand(led::setPurple)));

    m_driverController.povRight().onTrue(new ParallelCommandGroup(new InstantCommand(intake::closeJaws)));
    m_driverController.povLeft().onTrue(new ParallelCommandGroup(new InstantCommand(intake::openJaws)));

    //m_driverController.y().onTrue(new InstantCommand(poleCam::togglePipeline));

    //m_driverController.a().onTrue(new InstantCommand(tracker::adjustDeviation));
    //m_driverController.a().onTrue(new InstantCommand(poleCam::useVision));
    //m_driverController.b().onTrue(new InstantCommand(poleCam::noUseVision));
    m_driverController.button(7).onTrue(new Zero());
    m_driverController.button(8).whileTrue(new Seagul(superStructure));
    m_driverController.button(8).onFalse(new Stow(superStructure));

    //m_driverController.b().whileTrue(new ParallelCommandGroup(new SetElevatorHeight(0, elevator, false), new SetWristAngle(Rotation2d.fromDegrees(-123), wrist, false, false), new RunIntake(intake, IntakeState.INTAKE_CUBE)));
    //m_driverController.b().onFalse(new ParallelCommandGroup(new SetWristAngle(Rotation2d.fromDegrees(-7), wrist, false, false)/* , new InstantCommand(intake::closeJaws)*/));
    //intake CUBES (slower intake speed)
    
    //m_driverController.y().whileTrue(new ParallelCommandGroup(new RunIntake(intake, IntakeState.INTAKE), new SetElevatorHeight(-5, elevator, false), new SetWristAngle(Rotation2d.fromDegrees(-135), wrist, false, false)));
    //m_driverController.y().onFalse(new ParallelCommandGroup(new RunIntake(intake, IntakeState.OFF), new SetWristAngle(Rotation2d.fromDegrees(-7), wrist, false, false), new SetElevatorHeight(-0.25, elevator, false)));
    //ground intake upright CONES
    
    //m_operatorController.button(7).onTrue(new Zero());
    m_operatorController.button(7).onTrue(new ChangeNode(RedConstants.NODE_CONE_RED_5.getY()));
    m_operatorController.button(9).onTrue(new ChangeNode(RedConstants.NODE_CONE_RED_4.getY()));
    m_operatorController.button(11).onTrue(new ChangeNode(RedConstants.NODE_CONE_RED_3.getY()));
    m_operatorController.button(10).onTrue(new ChangeNode(RedConstants.NODE_CONE_RED_2.getY()));
    m_operatorController.button(12).onTrue(new ChangeNode(RedConstants.NODE_CONE_RED_1.getY()));

    m_operatorController.button(3).onTrue(new InstantCommand(tracker::setOffset));

    m_operatorController.povDown().whileTrue(new SetScoreHigh(true));
    m_operatorController.povDown().onFalse(new SetScoreHigh(false));

    //m_operatorController.button(3).whileTrue(new CustomSetpoints(superStructure, false)); //bottom left
    //m_operatorController.button(3).onFalse(new Stow(superStructure)); 

    //m_operatorController.button(1).whileTrue(new HorizontalLock(m_driverController::getLeftX, m_driverController::getRightY, drive));
  }

  public Command getAutonomousCommand() {
    HashMap<String, Command> eventMap = new HashMap<>();
    //eventMap.put("score", new SequentialCommandGroup(new Stow(superStructure),new WaitCommand(.5),new ScoreConeHigh(superStructure), new WaitCommand(1), new ShootCone(), new WaitCommand(.4), new Stow(superStructure)));
    eventMap.put("score", new SequentialCommandGroup(new ScoreConeHigh(superStructure), new WaitCommand(.5), new ShootCone(), new WaitCommand(.4), new Stow(superStructure)));
    eventMap.put("intake-cone",new SequentialCommandGroup(new SetIntakingHeight(superStructure, SuperStructureState.FALLEN_CONE)));
    eventMap.put("intake-cube",new SequentialCommandGroup(new SetIntakingHeight(superStructure, SuperStructureState.CUBE_INTAKE)));
    eventMap.put("stow",new Stow(superStructure));
    eventMap.put("cook",new SequentialCommandGroup(new OpenDoor(superStructure, 0.5), new WaitCommand(.5)));
    eventMap.put("uncook", new Stow(superStructure));
    eventMap.put("lock", new InstantCommand(drive::lockModules));
    eventMap.put("score-cube-mid", new SequentialCommandGroup(new ScoreCubeHigh(superStructure), new WaitCommand(0.15), new ShootCone(), new WaitCommand(0.1), new Stow(superStructure)));
    eventMap.put("go-to-shoot", new ShootPosition());
    eventMap.put("shoot", new ShootCone());
    //eventMap.put("score-1", new ShootCube());
    //eventMap.put("score-preload", new SequentialCommandGroup(new ScoreConeHigh(superStructure), new WaitCommand(0.75), new ShootCone()));
    //eventMap.put("intake",new frc.robot.commands.Intake(superStructure, true));
    //eventMap.put("stow", new Stow(superStructure));
    //eventMap.put("not-kadoomer", new ParallelCommandGroup(new SetWristAngle(Rotation2d.fromDegrees(-7), wrist, false, false), new RunIntake(intake, IntakeState.OFF)));
    //ooga-wooga

    HashMap<String, Command> climbMap = new HashMap<>();
    climbMap.put("score", new SequentialCommandGroup(new Stow(superStructure),new WaitCommand(.25),new ScoreConeHigh(superStructure), new WaitCommand(0.75), new ShootCone(), new WaitCommand(.5), new Stow(superStructure)));
    climbMap.put("score", new SequentialCommandGroup(new Stow(superStructure),new WaitCommand(.25),new ScoreConeHigh(superStructure), new WaitCommand(0.75), new ShootCone(), new WaitCommand(.5), new Stow(superStructure)));
    climbMap.put("cook",new SequentialCommandGroup(new OpenDoor(superStructure, 0.5), new WaitCommand(.5)));
    climbMap.put("uncook", new Stow(superStructure));
    climbMap.put("lock", new InstantCommand(drive::lockModules));

    //TODO: add wait until to check drive pitch before releasing door so we can engage on the path
    //eventMap.put("not-kadoomer", new ParallelCommandGroup(new SetWristAngle(Rotation2d.fromDegrees(-7), wrist, false, false), new RunIntake(intake, IntakeState.OFF)));
    //ooga-wooga

  return new AutoRoutine("Climb jawn Copy", new PathConstraints(3, 3), eventMap).buildAuto();//Autos.exampleAuto(m_exampleSubsystem);
    //return new AutoRoutine("Climb jawn Copy", new PathConstraints(2, 2), eventMap).buildAuto();//Autos.exampleAuto(m_exampleSubsystem);
  }

  public static boolean gotDriverInput(){
    return Math.abs(Math.max(m_driverController.getLeftX(), m_driverController.getLeftY())) > .2;
  }
}
