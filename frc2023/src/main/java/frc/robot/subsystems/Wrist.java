package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
    private WPI_TalonFX wristMotor;
    private DutyCycleEncoder wristEncoder;
    private Rotation2d setpoint;
    private ProfiledPIDController wristController;
    
    public Wrist(){
        wristMotor = new WPI_TalonFX(WristConstants.DEVICE_ID_WRIST);
        wristEncoder = new DutyCycleEncoder(WristConstants.ABS_ENCODER_PORT);
        wristController = new ProfiledPIDController(WristConstants.WRIST_CONTROLLER_KP, WristConstants.WRIST_CONTROLLER_KI, WristConstants.WRIST_CONTROLLER_KD, new TrapezoidProfile.Constraints(2*Math.PI,1.5*Math.PI));

        wristController.enableContinuousInput(-Math.PI, Math.PI);
        wristController.setTolerance(WristConstants.WRIST_CONTROLLER_TOLERANCE_RAD);
        wristEncoder.setDistancePerRotation(2*Math.PI);

        wristMotor.configFactoryDefault();
        wristMotor.configForwardSoftLimitThreshold(WristConstants.SOFT_LIMIT_FORWARD);
        wristMotor.configForwardSoftLimitEnable(true);
        wristMotor.configReverseSoftLimitThreshold(WristConstants.SOFT_LIMIT_REVERSE);
        wristMotor.configReverseSoftLimitEnable(true);

        wristMotor.configPeakOutputForward(.25);
        wristMotor.configPeakOutputReverse(-.25);

    }

    @Override
    public void periodic() {
        wristController.setGoal(setpoint.getRadians());
        double output = wristController.calculate(getRelativeAngle().getRadians(),setpoint.getRadians());
        wristMotor.setVoltage(output); // might need to change to set

    }

    public void jog(double val){
        wristMotor.set(ControlMode.PercentOutput, val);
    }

    private Rotation2d getRelativeAngle(){
        Rotation2d reportedAngle = new Rotation2d(wristEncoder.get()*Math.PI*2);
        return reportedAngle;
    }

    public void setGains(){
        wristController.setPID(SmartDashboard.getNumber("Wrist kP", 0), 0, 0);
    }

    public void setSetpoint(Rotation2d requestedAngle){
        setpoint = requestedAngle;
    }

    private void logData(){
        SmartDashboard.putNumber("Wrist Angle", getRelativeAngle().getDegrees());
        SmartDashboard.putNumber("Wrist Onboard Sensor Position", wristMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Wrist Setpoint", setpoint.getDegrees());
        SmartDashboard.putNumber("Wrist PID setpoint", wristController.getGoal().position);
        SmartDashboard.putNumber("Wrist PID error", Units.radiansToDegrees(wristController.getPositionError()));  

    }

}