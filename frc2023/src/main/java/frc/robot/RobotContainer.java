// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SetGains;
import frc.robot.commands.drive.DriveTele;
import frc.robot.commands.elevator.IdleElevator;
import frc.robot.commands.elevator.JogElevator;
import frc.robot.commands.elevator.SetElevatorHeight;
import frc.robot.commands.intake.IdleIntake;
import frc.robot.commands.intake.MoveIntakeAngle;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.wrist.JogWrist;
import frc.robot.commands.wrist.SetWristAngle;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tracker;
import frc.robot.subsystems.Wrist;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Tracer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
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
  private final Tracker tracker= Tracker.getInstance();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drive.setDefaultCommand(new DriveTele(m_driverController::getLeftY, m_driverController::getLeftX, m_driverController::getRightX, drive));
    intake.setDefaultCommand(new IdleIntake(intake));
    SmartDashboard.putNumber("test", 0);

  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    m_driverController.leftTrigger().whileTrue(new RunIntake(false, intake, Rotation2d.fromDegrees(0),true));
    m_driverController.leftTrigger().onFalse(new IdleIntake(intake));
    
    m_driverController.rightTrigger().whileTrue(new RunIntake(true, intake, Rotation2d.fromDegrees(0),true));
    m_driverController.rightTrigger().onFalse(new IdleIntake(intake));
  

    m_driverController.leftBumper().whileTrue(new JogElevator(.30, elevator));
    m_driverController.rightBumper().whileTrue(new JogElevator(-.30, elevator));

    m_driverController.povUp().whileTrue(new InstantCommand(drive::resetHeading,drive));

    //m_driverController


    
    //m_driverController.x().whileTrue(new SetWristAngle(Rotation2d.fromDegrees(0), wrist, false));

    //m_driverController.y().whileTrue(new SetWristAngle(Rotation2d.fromDegrees(180.5), wrist, false));

    m_driverController.b().whileTrue(new MoveIntakeAngle(intake, 1));
    m_driverController.b().onFalse(new MoveIntakeAngle(intake, 0));

    m_driverController.a().whileTrue(new MoveIntakeAngle(intake, -1));
    m_driverController.a().onFalse(new MoveIntakeAngle(intake, 0));

    
    m_driverController.x().whileTrue(new JogWrist(true, wrist));
    m_driverController.y().whileTrue(new JogWrist(false,wrist));

/* 
    //17791
     
    m_driverController.leftTrigger().onTrue(Commands.parallel(new SetElevatorHeight(32000,elevator, false),new SetWristAngle(Rotation2d.fromDegrees(200), wrist, false), new RunIntake(false, intake, "cones")));
    m_driverController.leftTrigger().onFalse(Commands.parallel(new SetElevatorHeight(0,elevator, false),new SetWristAngle(Rotation2d.fromDegrees(0), wrist,false)));

    m_driverController.leftBumper().onTrue(Commands.parallel(new SetElevatorHeight(0,elevator, false),new SetWristAngle(Rotation2d.fromDegrees(170), wrist, false), new RunIntake(false, intake, "cones")));
    m_driverController.leftBumper().onFalse(Commands.parallel(new SetElevatorHeight(0,elevator, false),new SetWristAngle(Rotation2d.fromDegrees(0), wrist,false)));


    //m_driverController.leftTrigger().whileTrue(new RunIntake(false, intake));

    //m_driverController.rightTrigger().onTrue(Commands.parallel(new SetElevatorHeight(0,elevator, true),new SetWristAngle(Rotation2d.fromDegrees(0), wrist,true), new RunIntake(true, intake)));
    m_driverController.rightTrigger().onFalse(Commands.parallel(new SetElevatorHeight(0,elevator, false),new SetWristAngle(Rotation2d.fromDegrees(0), wrist,false)));

    m_driverController.leftStick().onTrue(Commands.parallel(new SetElevatorHeight(45000,elevator, false),new SetWristAngle(Rotation2d.fromDegrees(200), wrist, false)));
    //m_driverController.leftStick().onFalse(Commands.parallel(new SetElevatorHeight(0,elevator, false),new SetWristAngle(Rotation2d.fromDegrees(0), wrist,false)));

    m_driverController.rightStick().onTrue(Commands.parallel(new SetElevatorHeight(60000,elevator, false),new SetWristAngle(Rotation2d.fromDegrees(200), wrist, false)));
    //m_driverController.rightStick().onFalse(Commands.parallel(new SetElevatorHeight(0,elevator, false),new SetWristAngle(Rotation2d.fromDegrees(0), wrist,false)));
    

    m_driverController.b().onTrue(Commands.parallel(new SetElevatorHeight(0,elevator, false),new SetWristAngle(Rotation2d.fromDegrees(0), wrist,false)));
    */


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
  public Command getAutonomousCommand(String autoToUse) {

    TrajectoryConfig config = new TrajectoryConfig(2, 1);
    PIDController xController = new PIDController(1.25, 0, 0);
    PIDController yController = new PIDController(1.25, 0, 0);
    
    xController.setTolerance(.05);
    yController.setTolerance(.05);

    Trajectory testTrajectory = new Trajectory() ;
    /*TrajectoryGenerator.generateTrajectory(
        new Pose2d(0.0, 0.0, new Rotation2d(0)),
        List.of(
          new Translation2d(0, .25),
          new Translation2d(0,.5)
        ),
        new Pose2d(0,.75 , Rotation2d.fromDegrees(90)),
        config
        );*/
        //Traje   ctory testTrajectory;
      try{
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(autoToUse);
        testTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      }
      catch(IOException ex){
        System.out.println();
      }
      
  

      SwerveControllerCommand swerveControllerCommand =
    new SwerveControllerCommand(
        testTrajectory,
        tracker::getOdometry, // Functional interface to feed supplier
        DriveConstants.DRIVE_KINEMATICS,
        // Position controllers 
        xController,
        yController,
        drive.getRotationController(),
        drive::setModuleStates,
        drive);

        
        return  swerveControllerCommand;
  }

  public void configurePath(){
    Trajectory testTrajectory = new Trajectory() ;
    /*TrajectoryGenerator.generateTrajectory(
        new Pose2d(0.0, 0.0, new Rotation2d(0)),
        List.of(
          new Translation2d(0, .25),
          new Translation2d(0,.5)
        ),
        new Pose2d(0,.75 , Rotation2d.fromDegrees(90)),
        config
        );*/
        //Trajectory testTrajectory;
      try{
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/huh.wpilib.json");
        testTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      }
      catch(IOException ex){
        System.out.println();
      }
  }
}
