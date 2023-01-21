package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.VisionManager;

public class Align extends CommandBase{
    private Drive drive;
    private VisionManager vision;
    private ProfiledPIDController controller;


    public Align(Drive dInstance, VisionManager vInstance){
        drive = dInstance;
        vision = vInstance;
        addRequirements(drive);
        addRequirements(vision);
    }

    @Override
    public void initialize() {
        controller = new ProfiledPIDController(1.5, 0, 0, new TrapezoidProfile.Constraints(2, 1));
        controller.enableContinuousInput(-Math.PI, Math.PI);
        controller.setTolerance(Units.degreesToRadians(3));
        controller.setP(SmartDashboard.getNumber("align kP", 0));
        
    }

    @Override
    public void execute() {
        double error = Rotation2d.fromDegrees(0).rotateBy(vision.getTargetYaw()).getRadians();

        double deltaSpeed = controller.calculate(error);

        drive.setMotors(deltaSpeed, -deltaSpeed);

        SmartDashboard.putNumber("controller output", deltaSpeed);
        SmartDashboard.putNumber("error", Units.radiansToDegrees(error));
    }

    @Override
    public boolean isFinished() {
        return controller.atGoal() || !vision.hasTargets();
    }
    @Override
    public void end(boolean interrupted) {
        drive.setMotors(0, 0);
    }
}
