package frc.robot.subsystems;


import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    WPI_Pigeon2 gyro =new WPI_Pigeon2(30, SwerveModuleConstants.SWERVE_CANIVORE_ID);


    double maxAngVel = 2 * Math.PI;

    

    public Drive(){

    }

    public static Drive getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        
    }

    public void setModuleStates(SwerveModuleState[] states){
        frontLeft.setState(states[0]);
        frontRight.setState(states[1]);
        rearLeft.setState(states[2]);
        rearRight.setState(states[3]);

    }
    
    public synchronized Rotation2d getDriveHeading(){
        return gyro.getRotation2d();
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition frontLeftPos = new SwerveModulePosition(frontLeft.getPosition(),frontLeft.getRelativeHeading());
        SwerveModulePosition rearLeftPos = new SwerveModulePosition(rearLeft.getPosition(),rearLeft.getRelativeHeading());
        SwerveModulePosition frontRightPos = new SwerveModulePosition(frontRight.getPosition(),frontRight.getRelativeHeading());
        SwerveModulePosition rearRightPos = new SwerveModulePosition(rearLeft.getPosition(),rearRight.getRelativeHeading());

        SwerveModulePosition[] modulePositions = {frontLeftPos,rearLeftPos,frontRightPos,rearRightPos};

        return modulePositions;

    }
}
