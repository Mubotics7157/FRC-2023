package frc.robot.subsystems;


import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SwerveModule;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class Drive extends SubsystemBase {
    
    private static Drive instance = new Drive();
    private SwerveModule frontLeft = new SwerveModule(DriveConstants.FRONT_LEFT_DRIVE_PORT,DriveConstants.FRONT_LEFT_TURN_PORT,DriveConstants.FRONT_LEFT_ENCODER_PORT,DriveConstants.FRONT_LEFT_ENCODER_OFFSET, false);
    private SwerveModule frontRight = new SwerveModule(DriveConstants.FRONT_RIGHT_DRIVE_PORT,DriveConstants.FRONT_RIGHT_TURN_PORT,DriveConstants.FRONT_RIGHT_ENCODER_PORT,DriveConstants.FRONT_RIGHT_ENCODER_OFFSET, false);
    private SwerveModule rearLeft = new SwerveModule(DriveConstants.REAR_LEFT_DRIVE_PORT,DriveConstants.REAR_LEFT_TURN_PORT,DriveConstants.REAR_LEFT_ENCODER_PORT,DriveConstants.REAR_LEFT_ENCODER_OFFSET, false);
    private SwerveModule rearRight = new SwerveModule(DriveConstants.REAR_RIGHT_DRIVE_PORT,DriveConstants.REAR_RIGHT_TURN_PORT,DriveConstants.REAR_RIGHT_ENCODER_PORT,DriveConstants.REAR_RIGHT_ENCODER_OFFSET, false);
    private WPI_Pigeon2 gyro = new WPI_Pigeon2(DriveConstants.DEVICE_ID_PIGEON, SwerveModuleConstants.SWERVE_CANIVORE_ID);
    private TrapezoidProfile.Constraints rotProfile = new TrapezoidProfile.Constraints(2*Math.PI,Math.PI);
    private ProfiledPIDController rotController = new ProfiledPIDController(.5, 0, 0,rotProfile);

    public Drive(){
        rotController.setTolerance(5);
        rotController.enableContinuousInput(-Math.PI, Math.PI);

        gyro.reset();
    }

    public static Drive getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro Angle", -gyro.getAngle());
        SmartDashboard.putNumber("gyro yaw", Rotation2d.fromDegrees(gyro.getYaw()).getDegrees());

        SmartDashboard.putNumber("left front", frontLeft.getState().angle.getDegrees());

        SmartDashboard.putNumber("left rear", rearLeft.getState().angle.getDegrees());
        SmartDashboard.putNumber("right rear", rearRight.getState().angle.getDegrees());
        SmartDashboard.putNumber("front right", frontRight.getState().angle.getDegrees());

        SmartDashboard.putNumber("left rear adjusted angle",rearLeft.getState().angle.getDegrees() -  rearLeft.getRelativeHeading().getDegrees());


        SmartDashboard.putNumber("rotation controller error", rotController.getPositionError());
    }

    public void setModuleStates(SwerveModuleState[] states){
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        rearLeft.setState(states[2]);
        rearRight.setState(states[3]);

        double flError = states[0].angle.rotateBy(frontLeft.getState().angle).getDegrees();
        SmartDashboard.putNumber("left front error", flError);

        double frError = states[1].angle.rotateBy(frontRight.getState().angle).getDegrees();
        SmartDashboard.putNumber("right front error", frError);

        double rlError = states[2].angle.rotateBy(rearLeft.getState().angle).getDegrees();
        SmartDashboard.putNumber("left rear error", rlError);

        double rrError = states[3].angle.rotateBy(rearRight.getState().angle).getDegrees();
        SmartDashboard.putNumber("right rear error", rrError);
    }
    
    public synchronized Rotation2d getDriveHeading(){
        return gyro.getRotation2d();
    }

    public synchronized void resetHeading(){
        Tracker.getInstance().resetHeading();    
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition frontLeftPos = new SwerveModulePosition(frontLeft.getPosition(),frontLeft.getRelativeHeading());
        SwerveModulePosition rearLeftPos = new SwerveModulePosition(rearLeft.getPosition(),rearLeft.getRelativeHeading());
        SwerveModulePosition frontRightPos = new SwerveModulePosition(frontRight.getPosition(),frontRight.getRelativeHeading());
        SwerveModulePosition rearRightPos = new SwerveModulePosition(rearRight.getPosition(),rearRight.getRelativeHeading());

        SwerveModulePosition[] modulePositions = {frontLeftPos,frontRightPos,rearLeftPos,rearRightPos};
        SmartDashboard.putNumber("front Left relative Position", frontLeftPos.distanceMeters);
        return modulePositions;
    }

    public ProfiledPIDController getRotationController(){
        return rotController;
    }

}
