// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.drive.AlignStrafe;
import frc.robot.commands.drive.AlignToTarget;
import frc.robot.commands.drive.DriveTele;
import frc.robot.commands.elevator.JogElevator;
import frc.robot.commands.elevator.SetElevatorHeight;
import frc.robot.commands.elevator.StowElevator;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.wrist.JogWrist;
import frc.robot.commands.wrist.SetWristAngle;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Tracker;
import frc.robot.subsystems.VisionManager;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Intake.IntakeState;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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
  public static final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  
  private final Drive drive = Drive.getInstance();
  private final Tracker tracker= Tracker.getInstance();
  private final Elevator elevator = Elevator.getInstance();
  private final Wrist wrist = Wrist.getInstance();
  private final Intake intake = Intake.getInstance();
  private final VisionManager vision = VisionManager.getInstance();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    drive.setDefaultCommand(new DriveTele(m_driverController::getLeftY, m_driverController::getLeftX, m_driverController::getRightX, drive));
    //intake.setDefaultCommand(new IdleIntake(intake));
    SmartDashboard.putNumber("test", 0);
    //elevator.setDefaultCommand(Commands.run(elevator::holdAtWantedState, elevator));

  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    m_driverController.povUp().whileTrue(new InstantCommand(drive::resetHeading,drive));

    //m_driverController.leftBumper().whileTrue(new JogElevator(.35, elevator));

    //m_driverController.leftBumper().whileTrue(new SetElevatorHeight(15, elevator, false));
    m_driverController.leftBumper().whileTrue(new ParallelCommandGroup(new SetElevatorHeight(15, elevator, false), new SetWristAngle(Rotation2d.fromDegrees(-72), wrist, false), new AlignStrafe(m_driverController::getLeftY, m_driverController::getLeftX, m_driverController::getRightX, drive, tracker, vision)));
    m_driverController.leftBumper().onFalse(new ParallelCommandGroup(new StowElevator(elevator), new SetWristAngle(Rotation2d.fromDegrees(-7), wrist, false)));
    //m_driverController.rightBumper().whileTrue(new JogElevator(-.35, elevator));

    //m_driverController.a().whileTrue(new JogWrist(false, wrist));
    //m_driverController.y().whileTrue(new JogWrist(true, wrist));

    //m_driverController.y().whileTrue(new AlignToTarget(m_driverController::getLeftY, m_driverController::getLeftX, m_driverController::getRightX, drive, vision));
    m_driverController.y().whileTrue(new AlignStrafe(m_driverController::getLeftY, m_driverController::getLeftX, m_driverController::getRightX, drive, tracker, vision));

    m_driverController.leftTrigger().whileTrue(new RunIntake(intake, IntakeState.INTAKE_CONE));
    m_driverController.rightTrigger().whileTrue(new RunIntake(intake, IntakeState.OUTTAKE_CONE));

    m_driverController.x().whileTrue(new RunIntake(intake, IntakeState.INTAKE_CUBE));
    m_driverController.b().whileTrue(new RunIntake(intake, IntakeState.OUTTAKE_CUBE));

    m_driverController.a().whileTrue(new SetWristAngle(Rotation2d.fromDegrees(-72), wrist, false));
    m_driverController.a().onFalse(new SetWristAngle(Rotation2d.fromDegrees(0), wrist, false));
    m_driverController.povDown().onTrue(new InstantCommand(vision::toggleLED));



  }
//Big money yuoung money cash money brash money
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand(String autoToUse) {


      ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(
        autoToUse,
        new PathConstraints(0.5, 0.5)
        );

        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));
        //eventMap.put("intakeDown", new IntakeDown());

        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
          tracker::getOdometry, // Pose2d supplier
          tracker::setOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
          DriveConstants.DRIVE_KINEMATICS, // SwerveDriveKinematics
          new PIDConstants(1.25, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
          new PIDConstants(0.25, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
          drive::setModuleStates, // Module states consumer used to output to the drive subsystem
          eventMap,
          true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          drive // The drive subsystem. Used to properly set the requirements of path following commands
          ); 

          Command fullAuto = autoBuilder.fullAuto(pathGroup);


        return fullAuto;
      }
      /* 
      HashMap<String, Command> eventMap = new HashMap<>();
      eventMap.put("marker1", new PrintCommand("Passed marker 1"));
      //eventMap.put("intakeDown", new IntakeDown());
      */
      
  
      //not PP
        /* 
        FollowPathWithEvents command = new FollowPathWithEvents(
          swerveControllerCommand,
          pathGroup.getMarkers(),
          eventMap
        );
        */
        /* 
        SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
          tracker::getOdometry, // Pose2d supplier
          tracker::setOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
          DriveConstants.DRIVE_KINEMATICS, // SwerveDriveKinematics
          new PIDConstants(1.25, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
          new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
          drive::setModuleStates, // Module states consumer used to output to the drive subsystem
          eventMap,
          true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
          drive // The drive subsystem. Used to properly set the requirements of path following commands
          ); 
          
        
    Command fullAuto = autoBuilder.fullAuto(pathGroup);


        return fullAuto;
        */

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
