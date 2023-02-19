package frc.robot.subsystems;


import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SwerveModule;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {
    
    private double driveSpeed = DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY;
    private double driveAngle = DriveConstants.MAX_TELE_ANGULAR_VELOCITY;
    private static Drive instance = new Drive();
    private SwerveModule frontLeft = new SwerveModule(DriveConstants.FRONT_LEFT_DRIVE_PORT,DriveConstants.FRONT_LEFT_TURN_PORT,DriveConstants.FRONT_LEFT_ENCODER_PORT,DriveConstants.FRONT_LEFT_ENCODER_OFFSET, false);
    private SwerveModule frontRight = new SwerveModule(DriveConstants.FRONT_RIGHT_DRIVE_PORT,DriveConstants.FRONT_RIGHT_TURN_PORT,DriveConstants.FRONT_RIGHT_ENCODER_PORT,DriveConstants.FRONT_RIGHT_ENCODER_OFFSET, false);
    private SwerveModule rearLeft = new SwerveModule(DriveConstants.REAR_LEFT_DRIVE_PORT,DriveConstants.REAR_LEFT_TURN_PORT,DriveConstants.REAR_LEFT_ENCODER_PORT,DriveConstants.REAR_LEFT_ENCODER_OFFSET, false);
    private SwerveModule rearRight = new SwerveModule(DriveConstants.REAR_RIGHT_DRIVE_PORT,DriveConstants.REAR_RIGHT_TURN_PORT,DriveConstants.REAR_RIGHT_ENCODER_PORT,DriveConstants.REAR_RIGHT_ENCODER_OFFSET, false);
    private WPI_Pigeon2 gyro = new WPI_Pigeon2(DriveConstants.DEVICE_ID_PIGEON,DriveConstants.CANIVORE_NAME);
    private TrapezoidProfile.Constraints rotProfile = new TrapezoidProfile.Constraints(2*Math.PI,Math.PI);
    private ProfiledPIDController rotController = new ProfiledPIDController(.5, 0, 0,rotProfile);
    private double lastTimeStamp = Timer.getFPGATimestamp();
    private Translation2d lastReqVel = new Translation2d();

    public Drive(){
        rotController.setTolerance(5);
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        gyro.reset();

        SmartDashboard.putNumber("align P", 0.25);
        SmartDashboard.putNumber("strafe P", 0.25);
        SmartDashboard.putNumber("offset strafe", 0);

        PathPlannerServer.startServer(5811);
    }

    public static Drive getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        // SmartDashboard.putNumber("gyro yaw", getDriveHeading().getDegrees());
 
 
        // SmartDashboard.putNumber("left rear adjusted angle",rearLeft.getState().angle.getDegrees() -  rearLeft.getRelativeHeading().getDegrees());
 
 
        // SmartDashboard.putNumber("rotation controller error", rotController.getPositionError());
    }
    
    public void logData(){
        SmartDashboard.putNumber("left front", frontLeft.getState().angle.getDegrees());
        SmartDashboard.putNumber("left rear", rearLeft.getState().angle.getDegrees());
        SmartDashboard.putNumber("right rear", rearRight.getState().angle.getDegrees());
        SmartDashboard.putNumber("front right", frontRight.getState().angle.getDegrees());

    }

    public void setModuleStates(SwerveModuleState[] states){
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        rearLeft.setState(states[2]);
        rearRight.setState(states[3]);

        //SmartDashboard.putNumber("FL VEL Error", Math.abs(states[0].speedMetersPerSecond-frontLeft.getDriveVelocity()));

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
        driveSpeed = DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY;
        driveAngle = DriveConstants.MAX_TELE_ANGULAR_VELOCITY;
    }

    public void changeSlow(){
        driveSpeed = DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY / 2;
        driveAngle = DriveConstants.MAX_TELE_ANGULAR_VELOCITY / 2;
    }

    public void changeVerySlow(){
        driveSpeed = DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY / 4;
        driveAngle = DriveConstants.MAX_TELE_ANGULAR_VELOCITY / 4;
    }

    public double getTan(){
        return driveSpeed;
    }

    public double getAng(){
        return driveAngle;
    }
    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition frontLeftPos = new SwerveModulePosition(frontLeft.getPosition(),frontLeft.getRelativeHeading());
        SwerveModulePosition rearLeftPos = new SwerveModulePosition(rearLeft.getPosition(),rearLeft.getRelativeHeading());
        SwerveModulePosition frontRightPos = new SwerveModulePosition(frontRight.getPosition(),frontRight.getRelativeHeading());
        SwerveModulePosition rearRightPos = new SwerveModulePosition(rearRight.getPosition(),rearRight.getRelativeHeading());

        SwerveModulePosition[] modulePositions = {frontLeftPos,frontRightPos,rearLeftPos,rearRightPos};
        return modulePositions;
    }

    public ProfiledPIDController getRotationController(){
        return rotController;
    }

    public ChassisSpeeds limitTangentialAcceleration(ChassisSpeeds currVelocity){
        double dt = Timer.getFPGATimestamp() - lastTimeStamp;

        double maxDV =  DriveConstants.MAX_DRIVE_TANGENTIAL_ACCEL * dt;

        Translation2d currVel = new Translation2d(currVelocity.vxMetersPerSecond, currVelocity.vyMetersPerSecond);

        Translation2d dV =  currVel.minus(lastReqVel) ;

        if(dV.getNorm() > maxDV){
            Translation2d velLimit = lastReqVel.plus(new Translation2d(maxDV,maxDV));
            currVelocity.vxMetersPerSecond = velLimit.getX();
            currVelocity.vyMetersPerSecond = velLimit.getY();
            SmartDashboard.putString("Limited?", "yes");
        }
        else
            SmartDashboard.putString("Limited?", "no");


        SmartDashboard.putNumber("Current DV", dV.getNorm());
        SmartDashboard.putNumber("maxDV", maxDV);
        SmartDashboard.putNumber("Max X Vel", currVelocity.vxMetersPerSecond);
        SmartDashboard.putNumber("Max Y Vel", currVelocity.vyMetersPerSecond);
        SmartDashboard.putNumber("Last X Vel", lastReqVel.getX());
        SmartDashboard.putNumber("Last Y Vel", lastReqVel.getY());
        SmartDashboard.putNumber("DT", dt);
        SmartDashboard.putNumber("Current Accel", (currVel.minus(lastReqVel).getX())/dt);
        
        lastReqVel = currVel;
        lastTimeStamp = Timer.getFPGATimestamp();

        return currVelocity;

    }

}
