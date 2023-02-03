// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.drive.DriveTele;
import frc.robot.commands.elevator.JogElevator;
import frc.robot.commands.elevator.SetElevatorHeight;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.wrist.JogWrist;
import frc.robot.commands.wrist.SetWristAngle;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Intake.IntakeState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final Drive drive = Drive.getInstance();
  private final Elevator elevator = Elevator.getInstance();
  private final Wrist wrist = Wrist.getInstance();
  private final Intake intake = Intake.getInstance();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    drive.setDefaultCommand(new DriveTele(m_driverController::getLeftY, m_driverController::getRightY, drive));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //m_driverController.leftBumper().whileTrue(new JogElevator(-.25, elevator));
    //m_driverController.rightBumper().whileTrue(new JogElevator(.25, elevator));

    //m_driverController.leftTrigger().whileTrue(new RunIntake(intake, IntakeState.INTAKE_CONE));
    //m_driverController.rightTrigger().whileTrue(new RunIntake(intake, IntakeState.OUTTAKE_CONE));

    //m_driverController.x().whileTrue(new RunIntake(intake, IntakeState.INTAKE_CUBE));
    //m_driverController.b().whileTrue(new RunIntake(intake, IntakeState.OUTTAKE_CUBE));

    //m_driverController.a().whileTrue(new SetWristAngle(Rotation2d.fromDegrees(-72), wrist, false));
    //m_driverController.a().onFalse(new SetWristAngle(Rotation2d.fromDegrees(0), wrist, false));

    //m_driverController.a().whileTrue(new JogWrist(wrist, -.55));
    //m_driverController.a().onFalse(new JogWrist(wrist, 0));
    //m_driverController.y().whileTrue(new JogWrist(wrist, .55));
    //m_driverController.y().onFalse(new JogWrist(wrist, 0));

    //m_driverController.leftBumper().whileTrue(new SetElevatorHeight(-8, elevator, true));
    //m_driverController.leftBumper().onFalse(new SetElevatorHeight(0, elevator, false));

    m_driverController.leftTrigger().whileTrue(new ParallelCommandGroup(new SetElevatorHeight(0, elevator, false), new SetWristAngle(Rotation2d.fromDegrees(-125), wrist, true), new RunIntake(intake, IntakeState.INTAKE)));
    m_driverController.leftTrigger().onFalse(new ParallelCommandGroup(new SetWristAngle(Rotation2d.fromDegrees(-7), wrist, false)/* , new InstantCommand(intake::closeJaws)*/));

    m_driverController.leftBumper().whileTrue(new ParallelCommandGroup(new SetElevatorHeight(-24.5, elevator, true), new SetWristAngle(Rotation2d.fromDegrees(-105), wrist, true)));
    m_driverController.leftBumper().onFalse(new ParallelCommandGroup(new SetElevatorHeight(-.25, elevator, false), new SetWristAngle(Rotation2d.fromDegrees(-7), wrist, false)));

    m_driverController.rightTrigger().whileTrue(new RunIntake(intake, IntakeState.OUTTAKE));
    //m_driverController.rightBumper().whileTrue(new SetWristAngle(Rotation2d.fromDegrees(-72), wrist, true));
    //m_driverController.rightBumper().onFalse(new SetWristAngle(Rotation2d.fromDegrees(-7), wrist, false));

    m_driverController.povDown().onTrue(new InstantCommand(wrist::zeroOnboardEncoder, wrist));
    m_driverController.povUp().onTrue(new InstantCommand(elevator::zeroElevator));
    m_driverController.povRight().onTrue(new InstantCommand(intake::closeJaws));
    m_driverController.povLeft().onTrue(new InstantCommand(intake::openJaws));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
