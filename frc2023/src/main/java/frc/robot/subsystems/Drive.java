package frc.robot.subsystems;


import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SwerveModule;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class Drive extends SubsystemBase {
    
    private static Drive instance = new Drive();
    private SwerveModule frontLeft = DriveConstants.FRONT_LEFT_MODULE;
    private SwerveModule frontRight = DriveConstants.FRONT_RIGHT_MODULE;
    private SwerveModule rearRight = DriveConstants.REAR_RIGHT_MODULE;
    private SwerveModule rearLeft = DriveConstants.REAR_LEFT_MODULE;
    WPI_Pigeon2 gyro =new WPI_Pigeon2(30);


    double maxAngVel = 2 * Math.PI;

    

    public Drive(){
        SmartDashboard.putNumber("module P Gain", 0);
        SmartDashboard.putNumber("moduel D Gain", 0);
    }

    public static Drive getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Gyro Angle", -gyro.getAngle());
        SmartDashboard.putNumber("gyro yaw", gyro.getYaw());

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
        gyro.reset();
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition frontLeftPos = new SwerveModulePosition(frontLeft.getPosition(),frontLeft.getRelativeHeading());
        SwerveModulePosition rearLeftPos = new SwerveModulePosition(rearLeft.getPosition(),rearLeft.getRelativeHeading());
        SwerveModulePosition frontRightPos = new SwerveModulePosition(frontRight.getPosition(),frontRight.getRelativeHeading());
        SwerveModulePosition rearRightPos = new SwerveModulePosition(rearLeft.getPosition(),rearRight.getRelativeHeading());

        SwerveModulePosition[] modulePositions = {frontLeftPos,rearLeftPos,frontRightPos,rearRightPos};

        return modulePositions;
    }
    public void setIndividualModule(SwerveModule module, SwerveModuleState state){
        

    }

    public void setGains(double kP, double kD){
        frontLeft.updateP(kP);
        frontRight.updateP(kP);
        rearLeft.updateP(kP);
        rearRight.updateP(kP);

        frontLeft.updateD(kD);
        frontRight.updateD(kD);
        rearLeft.updateD(kD);
        rearRight.updateD(kD);

    }
}
