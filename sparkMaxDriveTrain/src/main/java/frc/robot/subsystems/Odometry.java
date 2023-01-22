package frc.robot.subsystems;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Odometry extends SubsystemBase{

    private static Odometry instance = new Odometry();
    DifferentialDrivePoseEstimator estimator;
    DifferentialDriveKinematics kinematics;

    public Odometry(){
        kinematics = new DifferentialDriveKinematics(0.8);


        estimator = new DifferentialDrivePoseEstimator(kinematics, Drive.getInstance().getDriveHeading(), Drive.getInstance().getLeftDistance(), Drive.getInstance().getRightDistance(), new Pose2d());
    }

    public static Odometry getInstance(){
        return instance;
    }

    public void updateRobot(){
        estimator.addVisionMeasurement(VisionManager.getInstance().getLimePose(), VisionManager.getInstance().getLimeLatency());
    }

    public Pose2d getPose(){
        estimator.update(Drive.getInstance().getDriveHeading(), Drive.getInstance().getLeftDistance(), Drive.getInstance().getRightDistance());
        return estimator.getEstimatedPosition();
    }

    public void relocalize(){
        estimator.resetPosition(Drive.getInstance().getDriveHeading(), 0, 0, VisionManager.getInstance().getLimePose());
    }

}
