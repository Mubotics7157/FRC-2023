package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.util.CommonConversions;

public class Wrist extends SubsystemBase {
    private WPI_TalonFX wristMotor;
    private DutyCycleEncoder wristEncoder;
    private Rotation2d setpoint = Rotation2d.fromDegrees(0);
    private static Wrist instance = new Wrist();
    private boolean holdAtWantedState;
    
    public Wrist(){
        wristMotor = new WPI_TalonFX(WristConstants.DEVICE_ID_WRIST);
        wristEncoder = new DutyCycleEncoder(WristConstants.ABS_ENCODER_PORT);
  
        holdAtWantedState = false;

        configWristDefault();
       
    }

    public static Wrist getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        if(holdAtWantedState)
            wristMotor.set(ControlMode.MotionMagic,CommonConversions.radiansToSteps(setpoint.getRadians(), 60));

        logData();
    }

    public void jog(double val){
        wristMotor.set(ControlMode.PercentOutput, val);
    }

    public void setHolding(boolean hold){
        setGains();
        holdAtWantedState = hold;

        setpoint = new Rotation2d(CommonConversions.stepsToRadians(wristMotor.getSelectedSensorPosition(), 60));
    }

    public void setSetpoint(Rotation2d requestedAngle){
        setpoint = requestedAngle;
    }

    public void setGains(){
        wristMotor.config_kP(0, .075);
    }


    public boolean atSetpoint(){
        return Math.abs(Units.radiansToDegrees(CommonConversions.stepsToRadians(wristMotor.getSelectedSensorPosition(), 60)) - Units.radiansToDegrees(setpoint.getRadians())) < 7;
    }

    private void zeroOnboardEncoder(){
        wristMotor.setSelectedSensorPosition(0);
    }

    private void configWristDefault(){
        wristEncoder.setDistancePerRotation(2*Math.PI);

        wristMotor.configFactoryDefault();

        wristMotor.setNeutralMode(NeutralMode.Brake);
        wristMotor.configPeakOutputForward(WristConstants.WRIST_PEAK_OUTPUT_FORWARD);
        wristMotor.configPeakOutputReverse(WristConstants.WRIST_PEAK_OUTPUT_REVERSE);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.slot0.kP = .075;
        config.motionCruiseVelocity = 30000;
        config.motionAcceleration = 15000;
        wristMotor.configAllSettings(config);

        zeroOnboardEncoder();

        //wristMotor.configForwardSoftLimitThreshold(WristConstants.SOFT_LIMIT_FORWARD);
        //wristMotor.configForwardSoftLimitEnable(true);
        //wristMotor.configReverseSoftLimitThreshold(WristConstants.SOFT_LIMIT_REVERSE);
        //wristMotor.configReverseSoftLimitEnable(true);


    }

    private void logData(){
        SmartDashboard.putNumber("Wrist Angle", Units.radiansToDegrees(CommonConversions.stepsToRadians(wristMotor.getSelectedSensorPosition(), 60)));
        SmartDashboard.putNumber("Wrist Onboard Sensor Position", wristMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Wrist Falcon Temp", wristMotor.getTemperature());
        SmartDashboard.putBoolean("Wrist Holding", holdAtWantedState);

    }

}
