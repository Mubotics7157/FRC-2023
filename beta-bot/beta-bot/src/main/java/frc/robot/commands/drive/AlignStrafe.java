package frc.robot.commands.drive;

import java.util.function.DoubleSupplier;
import java.util.stream.Stream;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.IntakeVision;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Tracker;
import frc.robot.subsystems.VisionManager;
import frc.robot.subsystems.VisionManager.VisionState;

public class AlignStrafe extends CommandBase{
    private Drive drive;
    private Tracker tracker;
    private VisionManager vision;
    private ProfiledPIDController rotController;
    private ProfiledPIDController strController;
    private boolean atGoal;
    private double deltaSpeed;

    private double strSpeed;

    private double offset = 0;


    private DoubleSupplier fwd, str, rot;
    private InterpolatingTreeMap<Double,Double> offsetMap = new InterpolatingTreeMap<>();


    public void driveFromChassis(ChassisSpeeds speeds){
        var states = DriveConstants.DRIVE_KINEMATICS.toSwerveModuleStates(speeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DriveConstants.MAX_TANGENTIAL_VELOCITY);
        drive.setModuleStates(states);
    }

    public AlignStrafe(DoubleSupplier fwd, DoubleSupplier str, DoubleSupplier rot, Drive dInstance, Tracker tInstance, VisionManager vInstance){
        this.fwd = fwd;
        this.str = str;
        this.rot = rot;

        
        drive = dInstance;
        tracker = tInstance;
        vision = vInstance;

        rotController = new ProfiledPIDController(SmartDashboard.getNumber("align P", 0.25), 0, 0, new TrapezoidProfile.Constraints(2*Math.PI , 2*Math.PI));
        strController = new ProfiledPIDController(SmartDashboard.getNumber("strafe P", 0.25), 0, 0, new TrapezoidProfile.Constraints(2, 2));

        addRequirements(drive);
        addRequirements(tracker);
        addRequirements(vision);
    }

    private double modifyInputs(double val, boolean isRot){
        if(isRot){
            if(Math.abs(val)<.15){
                val = 0;
            }
            return val*DriveConstants.MAX_TELE_ANGULAR_VELOCITY;
        }
        else{
            if(Math.abs(val)<.1){
                val = 0;
            }
            return val*DriveConstants.MAX_TELE_TANGENTIAL_VELOCITY;
        }
    }

    @Override
    public void initialize() {
        VisionManager.getInstance().setTargetLLState(VisionState.TAPE);
        atGoal= false;
        //rotController = new ProfiledPIDController(SmartDashboard.getNumber("align P", 0.25), 0, 0, new TrapezoidProfile.Constraints(2*Math.PI , 2*Math.PI));
        //strController = new ProfiledPIDController(SmartDashboard.getNumber("strafe P", 0.25), 0, 0, new TrapezoidProfile.Constraints(2, 2));
    
        rotController.setP(SmartDashboard.getNumber("align P", 0.25));
        strController.setP(SmartDashboard.getNumber("strafe P", 0.25));

        rotController.enableContinuousInput(-Math.PI, Math.PI);
        rotController.setTolerance(Units.degreesToRadians(3));
        rotController.setGoal(Units.degreesToRadians(0));

        strController.enableContinuousInput(-Math.PI, Math.PI);
        strController.setTolerance(Units.degreesToRadians(.4));
        strController.setGoal(Units.degreesToRadians(0));

        //offsetMap.put(-11.57, -3.58); //=========
        
        //offsetMap.put(-11.19, -2.66);
        //offsetMap.put(.237, 1.33);

        // Left Overflow
        offsetMap.put(17.0, 3.98);
        offsetMap.put(10.0, 3.98);
        offsetMap.put(9.0, 3.98);
        offsetMap.put(4.0, 0.0);
        offsetMap.put(0.0, 0.0);
        offsetMap.put(-4.0, 0.0);
        offsetMap.put(-9.0, -5.2);
        offsetMap.put(-10.0, -5.2);
        offsetMap.put(-18.0, -5.2);

        //offsetMap.put(11.77, 4.81); //==-==
        //offsetMap.put(12.0, 5.7);
        //offsetMap.put(7.7, 5.01);

    }

    @Override
    public void execute() {
        offset = offsetMap.get(IntakeVision.getInstance().getOffset());
        double vx =  modifyInputs(fwd.getAsDouble(),false);
        double vy =  modifyInputs(str.getAsDouble(),false);
        double omega = modifyInputs(rot.getAsDouble(), true);
        
        Rotation2d onTarget = Rotation2d.fromDegrees(180);
        double error = onTarget.rotateBy(tracker.getOdometry().getRotation()).getRadians();

        Rotation2d strTarget = Rotation2d.fromDegrees(-offset);
        double strError = strTarget.rotateBy(vision.getTargetYaw()).getRadians();

        if(Math.abs(error) > Units.degreesToRadians(3))
            deltaSpeed = rotController.calculate(error);
        else{
            deltaSpeed =0;
            atGoal = true;
        }

        if(Math.abs(strError) > Units.degreesToRadians(0.4)){
            strSpeed = strController.calculate(strError);
            LED.getInstance().setRedStrobe();
        }
        else{
            strSpeed = 0;
            LED.getInstance().setGreen();
        }

        if(vision.hasTargets())
            driveFromChassis(ChassisSpeeds.fromFieldRelativeSpeeds(vx, -strSpeed*DriveConstants.MAX_TANGENTIAL_VELOCITY, deltaSpeed*DriveConstants.MAX_TELE_ANGULAR_VELOCITY, Tracker.getInstance().getOdometry().getRotation()));
        else
            driveFromChassis(ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, deltaSpeed * DriveConstants.MAX_TELE_ANGULAR_VELOCITY, Tracker.getInstance().getOdometry().getRotation()));
        
        SmartDashboard.putNumber("controller output", deltaSpeed);
        SmartDashboard.putNumber("strafe speed", strSpeed);
        SmartDashboard.putNumber("error", Units.radiansToDegrees(error));
        SmartDashboard.putNumber("strafe error", Units.radiansToDegrees(strError));
        SmartDashboard.putNumber("strafe controller error", Units.radiansToDegrees(strController.getPositionError()));
        SmartDashboard.putBoolean("On target", rotController.atGoal());
        SmartDashboard.putBoolean("strafe on target", strController.atGoal());
        SmartDashboard.putNumber("actual vision offset", offset);
    }

    /* 
    @Override
    public boolean isFinished() {
        return atGoal;
    }
    */
    @Override
    public void end(boolean interrupted) {
        VisionManager.getInstance().setTargetLLState(VisionState.TAG);
        driveFromChassis(new ChassisSpeeds());
        LED.getInstance().setCurrentIntake();
    }
}