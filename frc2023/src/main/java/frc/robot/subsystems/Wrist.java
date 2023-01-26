package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.util.CommonConversions;

public class Wrist extends SubsystemBase {
    private WPI_TalonFX wristMotor;
    private DutyCycleEncoder wristEncoder;
    private Rotation2d setpoint = Rotation2d.fromDegrees(0);
    private ProfiledPIDController wristController;
    private static Wrist instance = new Wrist();
    private SendableChooser<Double> wristChooser;
    private boolean holdAtWantedState;
    
    public Wrist(){
        wristMotor = new WPI_TalonFX(WristConstants.DEVICE_ID_WRIST);
        wristEncoder = new DutyCycleEncoder(WristConstants.ABS_ENCODER_PORT);
        wristController = new ProfiledPIDController(WristConstants.WRIST_CONTROLLER_KP, WristConstants.WRIST_CONTROLLER_KI, WristConstants.WRIST_CONTROLLER_KD, new TrapezoidProfile.Constraints(2*Math.PI,1.5*Math.PI));
  
        wristController.enableContinuousInput(-Math.PI, Math.PI);
        wristController.setTolerance(WristConstants.WRIST_CONTROLLER_TOLERANCE_RAD);
        wristEncoder.setDistancePerRotation(2*Math.PI);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.slot0.kP = .075;
        wristMotor.configAllSettings(config);
        wristMotor.configFactoryDefault();

        wristMotor.configMotionCruiseVelocity(30000);
        wristMotor.configMotionAcceleration(15000);

        holdAtWantedState = false;
       
        //wristMotor.configForwardSoftLimitThreshold(WristConstants.SOFT_LIMIT_FORWARD);
        //wristMotor.configForwardSoftLimitEnable(true);
        //wristMotor.configReverseSoftLimitThreshold(WristConstants.SOFT_LIMIT_REVERSE);
        //wristMotor.configReverseSoftLimitEnable(true);


        wristMotor.setNeutralMode(NeutralMode.Brake);

        wristMotor.configPeakOutputForward(WristConstants.WRIST_PEAK_OUTPUT_FORWARD);
        wristMotor.configPeakOutputReverse(WristConstants.WRIST_PEAK_OUTPUT_REVERSE);

        wristMotor.setSelectedSensorPosition(0);

        wristChooser = new SendableChooser<>();
        wristChooser.addOption("intake fallen cone", 165.0);
        wristChooser.addOption("intake upright cone", 200.0);
        wristChooser.addOption("score high cone", 200.0);
        wristChooser.addOption("score mid cone", 200.0);
        wristChooser.addOption("custom", SmartDashboard.getNumber("wrist setpoint", 0));

        SmartDashboard.putData(wristChooser);

        SmartDashboard.putNumber("elevator setpoint", 0);
        SmartDashboard.putNumber("wrist setpoint", 0);

    }

    public static Wrist getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        if(holdAtWantedState)
            wristMotor.set(ControlMode.Position,CommonConversions.radiansToSteps(setpoint.getRadians(), 60));

        logData();
    }

    public void jog(double val){
        wristMotor.set(ControlMode.PercentOutput, val);
    }

    public void setHolding(boolean hold){
        holdAtWantedState = hold;

        setpoint = new Rotation2d(CommonConversions.stepsToRadians(wristMotor.getSelectedSensorPosition(), 60));
    }

    private Rotation2d getRelativeAngle(){
        Rotation2d reportedAngle = new Rotation2d(wristEncoder.get()*Math.PI*2);
        return reportedAngle;
    }


    public void setSetpoint(Rotation2d requestedAngle){
        setpoint = requestedAngle;
    }

    public void setGains(){
        wristMotor.config_kP(0, .075);
    }


    public double getSelectedAngle(){
        return wristChooser.getSelected();
    }

    public boolean atSetpoint(){
        return Math.abs(Units.radiansToDegrees(CommonConversions.stepsToRadians(wristMotor.getSelectedSensorPosition(), 60)) - Units.radiansToDegrees(setpoint.getRadians())) < 7;
    }

    private void logData(){
        SmartDashboard.putNumber("Wrist Angle", Units.radiansToDegrees(CommonConversions.stepsToRadians(wristMotor.getSelectedSensorPosition(), 60)));
        SmartDashboard.putNumber("Wrist Onboard Sensor Position", wristMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Wrist PID setpoint", setpoint.getDegrees());//wristController.getGoal().position);
        SmartDashboard.putNumber("Wrist PID error", Units.radiansToDegrees(wristController.getPositionError()));  
        SmartDashboard.putNumber("Wrist Falcon Temp", wristMotor.getTemperature());

    }

}
