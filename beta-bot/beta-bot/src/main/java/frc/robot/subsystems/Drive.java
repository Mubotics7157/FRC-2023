package frc.robot.subsystems;


import java.util.HashMap;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import frc.robot.util.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.util.SwerveModule;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.OpenDoor;
import frc.robot.commands.ScoreConeHigh;
import frc.robot.commands.ScoreConeMid;
import frc.robot.commands.ScoreCubeHigh;
import frc.robot.commands.ScoreCubeHighShoot;
import frc.robot.commands.SetIntakingHeight;
import frc.robot.commands.ShootCone;
import frc.robot.commands.ShootPosition;
import frc.robot.commands.Stow;
import frc.robot.subsystems.SuperStructure.SuperStructureState;

public class Drive extends SubsystemBase {
    private static final SuperStructure superStructure = SuperStructure.getInstance();

    private double driveSpeed = DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY;
    private double driveAngle = DriveConstants.MAX_TELE_ANGULAR_VELOCITY;
    private double tanDeadband = 0.15;
    private double angDeadband = 0.15;
    private Rotation2d softAngle = new Rotation2d();

    private Alliance lastKnownAlliance;

    private static Drive instance = new Drive();
    /* 
    private SwerveModule frontLeft = new SwerveModule(DriveConstants.FRONT_LEFT_DRIVE_PORT,DriveConstants.FRONT_LEFT_TURN_PORT,DriveConstants.FRONT_LEFT_ENCODER_PORT,AltConstants.DriveConstants.FRONT_LEFT_ENCODER_OFFSET, false);
    private SwerveModule frontRight = new SwerveModule(DriveConstants.FRONT_RIGHT_DRIVE_PORT,DriveConstants.FRONT_RIGHT_TURN_PORT,DriveConstants.FRONT_RIGHT_ENCODER_PORT,AltConstants.DriveConstants.FRONT_RIGHT_ENCODER_OFFSET, false);
    private SwerveModule rearLeft = new SwerveModule(DriveConstants.REAR_LEFT_DRIVE_PORT,DriveConstants.REAR_LEFT_TURN_PORT,DriveConstants.REAR_LEFT_ENCODER_PORT,AltConstants.DriveConstants.REAR_LEFT_ENCODER_OFFSET, false);
    private SwerveModule rearRight = new SwerveModule(DriveConstants.REAR_RIGHT_DRIVE_PORT,DriveConstants.REAR_RIGHT_TURN_PORT,DriveConstants.REAR_RIGHT_ENCODER_PORT,AltConstants.DriveConstants.REAR_RIGHT_ENCODER_OFFSET, false);
    */
    private SwerveModule frontLeft = new SwerveModule(DriveConstants.FRONT_LEFT_DRIVE_PORT,DriveConstants.FRONT_LEFT_TURN_PORT,DriveConstants.FRONT_LEFT_ENCODER_PORT,Constants.DriveConstants.FRONT_LEFT_ENCODER_OFFSET, false);
    private SwerveModule frontRight = new SwerveModule(DriveConstants.FRONT_RIGHT_DRIVE_PORT,DriveConstants.FRONT_RIGHT_TURN_PORT,DriveConstants.FRONT_RIGHT_ENCODER_PORT,Constants.DriveConstants.FRONT_RIGHT_ENCODER_OFFSET, false);
    private SwerveModule rearLeft = new SwerveModule(DriveConstants.REAR_LEFT_DRIVE_PORT,DriveConstants.REAR_LEFT_TURN_PORT,DriveConstants.REAR_LEFT_ENCODER_PORT,Constants.DriveConstants.REAR_LEFT_ENCODER_OFFSET, false);
    private SwerveModule rearRight = new SwerveModule(DriveConstants.REAR_RIGHT_DRIVE_PORT,DriveConstants.REAR_RIGHT_TURN_PORT,DriveConstants.REAR_RIGHT_ENCODER_PORT,Constants.DriveConstants.REAR_RIGHT_ENCODER_OFFSET, false);
    
    private WPI_Pigeon2 gyro = new WPI_Pigeon2(DriveConstants.DEVICE_ID_PIGEON,DriveConstants.CANIVORE_NAME);
    private TrapezoidProfile.Constraints rotProfile = new TrapezoidProfile.Constraints(2*Math.PI,Math.PI);
    private ProfiledPIDController rotController = new ProfiledPIDController(.5, 0, 0,rotProfile);

    //HashMap<String, Command> eventMap = new HashMap<>();


    public Drive(){
        rotController.setTolerance(5);
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        gyro.reset();

        SmartDashboard.putNumber("align P", 0.25);
        SmartDashboard.putNumber("strafe P", 0.25);
        SmartDashboard.putNumber("offset strafe", 0);

        PathPlannerServer.startServer(5811);
/* 
    eventMap.put("score", new SequentialCommandGroup(new Stow(superStructure),new ScoreConeHigh(superStructure), new ShootCone(), new WaitCommand(.2)));
    //eventMap.put("score", new SequentialCommandGroup(new ScoreConeHigh(superStructure), new ShootCone(), new WaitCommand(.4), new Stow(superStructure)));
    eventMap.put("unstowed score cone", new SequentialCommandGroup(new Stow(superStructure), new ScoreConeHigh(superStructure), new WaitCommand(.2), new ShootCone()));
    eventMap.put("score-cone-mid", new SequentialCommandGroup(new ScoreConeMid(superStructure), new WaitCommand(.5), new ShootCone(), new WaitCommand(.4), new Stow(superStructure)));
    eventMap.put("intake-cone",new SequentialCommandGroup(new SetIntakingHeight(superStructure, SuperStructureState.FALLEN_CONE)));
    eventMap.put("intake-cube",new SequentialCommandGroup(new SetIntakingHeight(superStructure, SuperStructureState.CUBE_INTAKE)));
    eventMap.put("stow",new Stow(superStructure));
    eventMap.put("cook",new SequentialCommandGroup(new OpenDoor(superStructure, 0.5), new WaitCommand(.25)));
    eventMap.put("uncook", new Stow(superStructure));
    eventMap.put("lock", new InstantCommand(this::lockModules));
    eventMap.put("score-cube-mid", new SequentialCommandGroup(new ScoreCubeHigh(superStructure), new ShootCone(), new WaitCommand(0.3), new Stow(superStructure)));
    eventMap.put("go-to-shoot", new ShootPosition());
    eventMap.put("shoot", new ShootCone());
    eventMap.put("snipe cube high", new ScoreCubeHighShoot(superStructure));
    eventMap.put("score cube high", new SequentialCommandGroup(new Stow(superStructure), new ScoreCubeHigh(superStructure), new ShootCone(), new WaitCommand(.6)));
    eventMap.put("shoot preload", new SequentialCommandGroup(new ShootPosition(),new WaitCommand(.3),new ShootCone()));
    eventMap.put("reset", new InstantCommand(Tracker.getInstance()::resetViaVision));
    //eventMap.put("score-1", new ShootCube());
    //eventMap.put("score-preload", new SequentialCommandGroup(new ScoreConeHigh(superStructure), new WaitCommand(0.75), new ShootCone()));
    //eventMap.put("intake",new frc.robot.commands.Intake(superStructure, true));
    //eventMap.put("stow", new Stow(superStructure));
    //eventMap.put("not-kadoomer", new ParallelCommandGroup(new SetWristAngle(Rotation2d.fromDegrees(-7), wrist, false, false), new RunIntake(intake, IntakeState.OFF)));
    //ooga-wooga
    */
    }

    public static Drive getInstance(){
        if(instance == null){
            return new Drive();
        }
        else
            return instance;
    }

    @Override
    public void periodic() {
        //logData();
        SmartDashboard.putNumber("Drive Speed", getTan());
    }
    
    public void logData(){
         
        SmartDashboard.putNumber("left front", frontLeft.getHeading().getDegrees());
        SmartDashboard.putNumber("left rear", rearLeft.getHeading().getDegrees());
        SmartDashboard.putNumber("right front", frontRight.getHeading().getDegrees());
        SmartDashboard.putNumber("right rear", rearRight.getHeading().getDegrees());

        
        SmartDashboard.putNumber("left front abs", frontLeft.getAbsHeading().getDegrees());
        SmartDashboard.putNumber("left rear abs", rearLeft.getAbsHeading().getDegrees());
        SmartDashboard.putNumber("right front abs", frontRight.getAbsHeading().getDegrees());
        SmartDashboard.putNumber("right rear abs", rearRight.getAbsHeading().getDegrees());
        
        
        SmartDashboard.putNumber("left front velocity", frontLeft.getDriveVelocity());
        /* 
        SmartDashboard.putNumber("left rear velocity", rearLeft.getDriveVelocity());
        SmartDashboard.putNumber("right front velocity", frontRight.getDriveVelocity());
        SmartDashboard.putNumber("right rear velocity", rearRight.getDriveVelocity());
        */
        /* 
        SmartDashboard.putNumber("left front", frontLeft.getState().angle.getDegrees());
        SmartDashboard.putNumber("left rear", rearLeft.getState().angle.getDegrees());
        SmartDashboard.putNumber("right rear", rearRight.getState().angle.getDegrees());
        SmartDashboard.putNumber("front right", frontRight.getState().angle.getDegrees());
        SmartDashboard.putNumber("Gyro Pitch", gyro.getPitch());

        SmartDashboard.putNumber("left front position", frontLeft.getPosition());
*/
    }

    public void setModuleStates(SwerveModuleState[] states){
        double currentTime = Timer.getFPGATimestamp();
        
        //SmartDashboard.putNumber("angle setState", states[0].angle.getDegrees());
        //double currVel = frontLeft.getDriveVelocity();
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        rearLeft.setState(states[2]);
        rearRight.setState(states[3]);

        SmartDashboard.putNumber("front left drive setpoint", states[0].speedMetersPerSecond);
 /* 

        SmartDashboard.putNumber("FL VEL Error", Math.abs(Math.abs(states[0].speedMetersPerSecond)-Math.abs(frontLeft.getDriveVelocity())));
        SmartDashboard.putNumber("FL Velocity", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("FL Turn Error", frontLeft.getHeading().rotateBy(states[0].angle.unaryMinus()).getDegrees());
        SmartDashboard.putNumber("FL Heading", frontLeft.getHeading().getDegrees());
        */
        /* 
        SmartDashboard.putNumber("FL VEL", frontLeft.getDriveVelocity());
        SmartDashboard.putNumber("FL Turn Error", frontLeft.getHeading().rotateBy(states[0].angle.unaryMinus()).getDegrees());
        SmartDashboard.putNumber("FL Accel", (frontLeft.getDriveVelocity()-lastReqVel)/(currentTime-lastTimeStamp));
        SmartDashboard.putNumber("Motor current draw", frontLeft.getCurrentDraw());
        SmartDashboard.putNumber("Front Left Relative Heading", frontLeft.getRelativeHeading().getDegrees());
        SmartDashboard.putNumber("Front Right Relative Heading", frontRight.getRelativeHeading().getDegrees());
        SmartDashboard.putNumber("Rear Left Relative Heading", rearLeft.getRelativeHeading().getDegrees());
        SmartDashboard.putNumber("Rear Right Relative Heading", rearRight.getRelativeHeading().getDegrees());
        SmartDashboard.putNumber("Front Left Abs Heading", frontLeft.getAbsHeading().getDegrees());
        SmartDashboard.putNumber("Front Right Abs Heading", frontRight.getAbsHeading().getDegrees());
        SmartDashboard.putNumber("Rear Left Abs Heading", rearLeft.getAbsHeading().getDegrees());
        SmartDashboard.putNumber("Rear Right Abs Heading", rearRight.getAbsHeading().getDegrees());
    */
        //lastReqVel = currVel;

        //double flError = states[0].angle.rotateBy(frontLeft.getState().angle).getDegrees();
        //SmartDashboard.putNumber("left front error", flError);

        //double frError = states[1].angle.rotateBy(frontRight.getState().angle).getDegrees();
        //SmartDashboard.putNumber("right front error", frError);

        //double rlError = states[2].angle.rotateBy(rearLeft.getState().angle).getDegrees();
        //SmartDashboard.putNumber("left rear error", rlError);

        //double rrError = states[3].angle.rotateBy(rearRight.getState().angle).getDegrees();
        //SmartDashboard.putNumber("right rear error", rrError);
    }
    
    public Rotation2d getDriveHeading(){
        return gyro.getRotation2d();
    }

    public double getDrivePitch(){
        return gyro.getPitch();
    }

    public void resetHeading(){
        Tracker.getInstance().resetHeading();    
    }
    
    public void changeMax(){
        tanDeadband = 0.15;
        angDeadband = 0.15;
        driveSpeed = DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY;
        driveAngle = DriveConstants.MAX_TELE_ANGULAR_VELOCITY;
    }

    public void changeSlow(){
        tanDeadband = 0.20;
        angDeadband = 0.25;
        driveSpeed = DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY / 3;
        driveAngle = DriveConstants.MAX_TELE_ANGULAR_VELOCITY / 3;
    }

    public void changeVerySlow(){
        tanDeadband = 0.20;
        angDeadband = 0.25;
        driveSpeed = DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY / 5;
        driveAngle = DriveConstants.MAX_TELE_ANGULAR_VELOCITY / 5;
    }

    public double getTan(){
        return driveSpeed;
    }

    public double getAng(){
        return driveAngle;
    }

    public double getTanDeadband(){
        return tanDeadband;
    }

    public double getAngDeadband(){
        return angDeadband;
    }
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] modulePositions = {frontLeft.getPosition(),frontRight.getPosition(),rearLeft.getPosition(),rearRight.getPosition()};

        return modulePositions;
    }

    public ProfiledPIDController getRotationController(){
        return rotController;
    }

    public void lockModules(){
        frontLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        frontRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        rearLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        rearRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    }

    public void changeMotorGains(){
        frontLeft.changeDriveKP();
        frontRight.changeDriveKP();
        rearLeft.changeDriveKP();
        rearRight.changeDriveKP();

        //frontLeft.changeTurnKP();
        //frontRight.changeTurnKP();
        //rearLeft.changeTurnKP();
        //rearRight.changeTurnKP();
        
    }

    public PPSwerveControllerCommand followPath(PathPlannerTrajectory traj,boolean startingPath){
            traj = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());

            if(startingPath)
                Tracker.getInstance().setPose(traj.getInitialHolonomicPose());
  
            return new PPSwerveControllerCommand(
                 traj,
                 Tracker.getInstance()::getPose, // Pose supplier
                 DriveConstants.DRIVE_KINEMATICS, // SwerveDriveKinematics
                 new PIDController(2.5
                 , 0, 0),
                 new PIDController(2.5, 0, 0), // Y controller (usually the same values as X controller)
                 new PIDController(1.75, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 this::setModuleStates,  
                 false,
                 this
            );
    }
    /* 
    public FollowPathWithEvents followPathEvents(PathPlannerTrajectory traj, boolean startingPath){

        return new FollowPathWithEvents(
            followPath(traj, startingPath), //path command
            traj.getMarkers(), //trajectory
            eventMap // hashmap for events
            );
    }
    */
        public PPSwerveControllerCommand followPath(Pose2d starting,double distMeters){
            Pose2d offsetPose = starting.plus(new Transform2d(new Translation2d(distMeters, 0),Rotation2d.fromDegrees(0)));
            PathPlannerTrajectory traj = PathPlanner.generatePath(
                new com.pathplanner.lib.PathConstraints(3, 4),
                new PathPoint(starting.getTranslation(),starting.getRotation(),starting.getRotation()), // position, heading
                new PathPoint(offsetPose.getTranslation(), offsetPose.getRotation(),starting.getRotation()) // position, heading
            );
            traj = PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance());

            Tracker.getInstance().setPose(traj.getInitialHolonomicPose());
            return new PPSwerveControllerCommand(
                 traj,
                 Tracker.getInstance()::getPose, // Pose supplier
                 DriveConstants.DRIVE_KINEMATICS, // SwerveDriveKinematics
                 new PIDController(2.5
                 , 0, 0),
                 new PIDController(2.5, 0, 0), // Y controller (usually the same values as X controller)
                 new PIDController(1.75, 0, 0), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                 this::setModuleStates,  
                 false,
                 this
            );
    }

    public Alliance getLastAlliance(){
        return lastKnownAlliance;
    }

    public void setLastAlliance(Alliance alliance){
        lastKnownAlliance = alliance;
    }

    private void stop(){
        frontLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        frontRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        rearLeft.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
        rearRight.setState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    }
}