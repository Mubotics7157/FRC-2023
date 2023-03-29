// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.plaf.basic.BasicSliderUI.TrackListener;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Tracker;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  //private final SendableChooser<String> autoChooser = new SendableChooser<>();
  //private String currentSelectedAuto;

  Compressor compressor = new Compressor(IntakeConstants.DEVICE_ID_PCM , IntakeConstants.PNEUMATICS_MODULE_TYPE);
  //DigitalInput test1 = new DigitalInput(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    SmartDashboard.putNumber("fallen cone wrist setpoint", 0);
    SmartDashboard.putNumber("Turn kP", .3/5);
/* 
    autoChooser.setDefaultOption("preload + climb","New PL + 2 Comp");
    autoChooser.addOption("preload + 1", "New PL +2");
    autoChooser.addOption("preload + 2 COMP","New PL + 2 Comp");
    autoChooser.addOption("preload + 2 BLUE","New PL + 2 Blue");
    autoChooser.addOption("preload + taxi", "PL + taxi");
    autoChooser.addOption("preload only", "PL only");
    autoChooser.addOption("preload + taxi + cube", "PL + taxi + cube");
    SmartDashboard.putData(autoChooser)

    */

    // = autoChooser.getSelected();   

    m_autonomousCommand = m_robotContainer.getAutonomousCommand("PL + 1 only");


  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    //SmartDashboard.putBoolean("test 1", test1.get());
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    LED.getInstance().setOrangeFade();
    m_autonomousCommand = m_robotContainer.getAutonomousCommand("test");
  }

  @Override
  public void disabledPeriodic() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand("PL + 1 only");
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      //Tracker.getInstance().resetViaVision();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
      compressor.enableDigital();
    
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  @Override
  public void autonomousExit() {
    Drive.getInstance().lockModules();
  }
  
  

}
