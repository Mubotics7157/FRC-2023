package frc.robot.subsystems;


import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SwerveModule;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {
    
    private double driveSpeed = DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY;
    private double driveAngle = DriveConstants.MAX_TELE_ANGULAR_VELOCITY;
    private double tanDeadband = 0.1;
    private double angDeadband = 0.15;

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


    public Drive(){
        rotController.setTolerance(5);
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        gyro.reset();

        SmartDashboard.putNumber("align P", 0.25);
        SmartDashboard.putNumber("strafe P", 0.25);
        SmartDashboard.putNumber("offset strafe", 0);

        //PathPlannerServer.startServer(5811);
    }

    public static Drive getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        logData();
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
        tanDeadband = 0.1;
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
}