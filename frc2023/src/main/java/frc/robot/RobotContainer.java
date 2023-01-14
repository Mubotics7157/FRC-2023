// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveTele;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IdleElevator;
import frc.robot.commands.IdleIntake;
import frc.robot.commands.JogElevator;
import frc.robot.commands.JogWrist;
import frc.robot.commands.RunIntake;
import frc.robot.commands.SetElevatorHeight;
import frc.robot.commands.SetGains;
import frc.robot.commands.SetWristAngle;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
  
  private final CommandXboxController m_operatorController =
      new CommandXboxController(1);
  
  private final Drive drive = Drive.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Elevator elevator = Elevator.getInstance();
  private final Wrist wrist = Wrist.getInstance();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drive.setDefaultCommand(new DriveTele(m_driverController::getLeftY, m_driverController::getLeftX, m_driverController::getRightX, drive));
    intake.setDefaultCommand(new IdleIntake(intake));
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    m_driverController.leftTrigger().whileTrue(new RunIntake(false,intake));
    m_driverController.rightTrigger().whileTrue(new RunIntake(true,intake));

    m_driverController.leftBumper().whileTrue(new JogElevator(.30, elevator));
    m_driverController.rightBumper().whileTrue(new JogElevator(-.30, elevator));

    m_driverController.povUp().whileTrue(new InstantCommand(drive::resetHeading,drive));

    m_driverController.x().whileTrue(new SetWristAngle(Rotation2d.fromDegrees(0), wrist));

    m_driverController.y().whileTrue(new SetWristAngle(Rotation2d.fromDegrees(180.5), wrist));

    
     m_driverController.a().onTrue(Commands.parallel(new SetElevatorHeight(53822,elevator),new SetWristAngle(Rotation2d.fromDegrees(180.5), wrist)));
     m_driverController.b().onTrue(Commands.parallel(new SetElevatorHeight(0,elevator),new SetWristAngle(Rotation2d.fromDegrees(0), wrist)));
    


    /* 
    int x = Integer.MAX_VALUE;
    while(x > 1) {
      x++;
    }
    */
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
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
