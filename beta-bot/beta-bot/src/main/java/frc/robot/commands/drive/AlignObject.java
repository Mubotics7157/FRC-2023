package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.AltConstants.DriveConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.VisionManager;
import frc.robot.subsystems.VisionManager.VisionState;

public class AlignObject extends CommandBase {

    private PIDController strafeController = new PIDController(1
    ,0,0);
    private Drive drive;
    private VisionManager vision;
    private double deltaSpeed;

    public AlignObject(Drive instance,VisionManager vision){
        drive = instance;
        this.vision = vision;

        strafeController.setTolerance(Units.degreesToRadians(1));
        addRequirements(drive,vision);
    }
    public void driveFromChassis(ChassisSpeeds speeds){
        var states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
        drive.setModuleStates(states);
    }

    @Override
    public void initialize() {
        System.out.println("RUNNING********************");
        //vision.setTargetLLState(VisionState.CUBE);
        strafeController.reset();
        strafeController.setSetpoint(0);
    }
    

    @Override
    public void execute() {
            if(!(strafeController.atSetpoint()) )
                deltaSpeed = strafeController.calculate(vision.getCubeYaw().getRadians());
            else
                strafeController.calculate(0);
            
            driveFromChassis(new ChassisSpeeds(0, deltaSpeed*frc.robot.Constants.DriveConstants.MAX_TANGENTIAL_VELOCITY, 0));
        
    }

    @Override
    public boolean isFinished() {
        return strafeController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        //vision.setTargetLLState(VisionState.TAG);
    }

    
}

