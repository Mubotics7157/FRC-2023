
package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Tracker extends SubsystemBase{

    SwerveDriveOdometry odometry = new SwerveDriveOdometry(DriveConstants.DRIVE_KINEMATICS, Drive.getInstance().getDriveHeading(),Drive.getInstance().getModulePositions());
    Field2d m_field;
    private static Tracker instance = new Tracker();




    public static Tracker getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        odometry.update(Drive.getInstance().getDriveHeading(), Drive.getInstance().getModulePositions());
    }

    public synchronized void setOdometry(Pose2d pose){
        odometry.resetPosition(Drive.getInstance().getDriveHeading(), Drive.getInstance().getModulePositions(), pose);
    }

    public synchronized Pose2d getOdometry(){
        return odometry.getPoseMeters();
    }


    public synchronized void resetHeading(){
        odometry.resetPosition(Drive.getInstance().getDriveHeading(), Drive.getInstance().getModulePositions(), new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0)));
    }



}
