package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.drive.DriveTele;
import frc.robot.commands.elevator.SetElevatorHeight;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.wrist.SetWristAngle;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tracker;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Intake.IntakeState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final Drive drive = Drive.getInstance();
  private final Elevator elevator = Elevator.getInstance();
  private final Wrist wrist = Wrist.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Tracker tracker = Tracker.getInstance();

  public RobotContainer() {
    configureBindings();
    drive.setDefaultCommand(new ParallelCommandGroup(new DriveTele(m_driverController::getLeftY, m_driverController::getLeftX, m_driverController::getRightX, drive), new InstantCommand(drive::changeMax)));

  }

  private void configureBindings() {

    m_driverController.leftTrigger().whileTrue(new ParallelCommandGroup(new SetElevatorHeight(0, elevator, false), new SetWristAngle(Rotation2d.fromDegrees(-123), wrist, false, false), new RunIntake(intake, IntakeState.INTAKE)));
    m_driverController.leftTrigger().onFalse(new ParallelCommandGroup(new SetWristAngle(Rotation2d.fromDegrees(-7), wrist, false, false)/* , new InstantCommand(intake::closeJaws)*/));
    //ground intake tipped CONES
    m_driverController.leftBumper().whileTrue(new ParallelCommandGroup(new SetElevatorHeight(-26, elevator, false), new SetWristAngle(Rotation2d.fromDegrees(-125), wrist, false, false), new InstantCommand(drive::changeSlow)));
    m_driverController.leftBumper().onFalse(new ParallelCommandGroup(new SetElevatorHeight(-.25, elevator, false), new SetWristAngle(Rotation2d.fromDegrees(-7), wrist, false, false), new InstantCommand(drive::changeMax)));
    //high score CONES
    m_driverController.rightBumper().whileTrue(new ParallelCommandGroup(new SetElevatorHeight(-17, elevator, false), new SetWristAngle(Rotation2d.fromDegrees(-122), wrist, false, false), new InstantCommand(drive::changeSlow)));
    m_driverController.rightBumper().onFalse(new ParallelCommandGroup(new SetElevatorHeight(0, elevator, false), new SetWristAngle(Rotation2d.fromDegrees(-7), wrist, false, false), new InstantCommand(drive::changeMax)));
    //mid score CONES
    m_driverController.rightTrigger().whileTrue(new RunIntake(intake, IntakeState.OUTTAKE));
    //poop
    m_driverController.a().whileTrue(new RunIntake(intake, IntakeState.INTAKE));
    //eat

    //m_driverController.rightBumper().whileTrue(new SetWristAngle(Rotation2d.fromDegrees(-72), wrist, true));
    //m_driverController.rightBumper().onFalse(new SetWristAngle(Rotation2d.fromDegrees(-7), wrist, false));
    m_driverController.x().whileTrue(new ParallelCommandGroup(new SetWristAngle(Rotation2d.fromDegrees(-40), wrist, true, true), new SetElevatorHeight(-.25, elevator, true), new RunIntake(intake, IntakeState.INTAKE)));
    m_driverController.x().onFalse(new ParallelCommandGroup(new SetWristAngle(Rotation2d.fromDegrees(-7), wrist, false, false), new SetElevatorHeight(.25, elevator, false)));
    //cube testing shot

    m_driverController.povDown().onTrue(new InstantCommand(wrist::zeroOnboardEncoder, wrist));
    //m_driverController.povUp().onTrue(new SetWristAngle(Rotation2d.fromDegrees(0), wrist, false, false));
    m_driverController.povUp().onTrue(new InstantCommand(drive::resetHeading));
    m_driverController.povRight().onTrue(new InstantCommand(intake::closeJaws));
    m_driverController.povLeft().onTrue(new InstantCommand(intake::openJaws));

    m_driverController.b().whileTrue(new ParallelCommandGroup(new SetElevatorHeight(0, elevator, false), new SetWristAngle(Rotation2d.fromDegrees(-123), wrist, false, false), new RunIntake(intake, IntakeState.INTAKE_CUBE)));
    m_driverController.b().onFalse(new ParallelCommandGroup(new SetWristAngle(Rotation2d.fromDegrees(-7), wrist, false, false)/* , new InstantCommand(intake::closeJaws)*/));
    //intake CUBES (slower intake speed)
    
    m_driverController.y().whileTrue(new ParallelCommandGroup(new RunIntake(intake, IntakeState.INTAKE), new SetElevatorHeight(-5, elevator, false), new SetWristAngle(Rotation2d.fromDegrees(-135), wrist, false, false)));
    m_driverController.y().onFalse(new ParallelCommandGroup(new RunIntake(intake, IntakeState.OFF), new SetWristAngle(Rotation2d.fromDegrees(-7), wrist, false, false), new SetElevatorHeight(-0.25, elevator, false)));
    //ground intake upright CONES
  }

  public Command getAutonomousCommand() {
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
