package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;
import frc.robot.util.CommonConversions;

public class Wrist extends SubsystemBase {

    public enum WristState{
        OFF,
        STOW,
        JOG,
        SETPOINT
    }

    private WPI_TalonFX wristMotor;
    private DutyCycleEncoder wristEncoder;
    private Rotation2d setpoint = Rotation2d.fromDegrees(0);
    private static Wrist instance = new Wrist();
    private boolean holdAtWantedState;
    private double jogVal;

    private WristState wristState;
    
    public Wrist(){
        jogVal = 0;
        wristMotor = new WPI_TalonFX(WristConstants.DEVICE_ID_WRIST);
        wristEncoder = new DutyCycleEncoder(WristConstants.ABS_ENCODER_PORT);
  
        holdAtWantedState = false;

        configWristDefault();

        wristState = WristState.STOW;
       
    }

    public static Wrist getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        if(holdAtWantedState)
            wristMotor.set(ControlMode.MotionMagic,CommonConversions.radiansToSteps(setpoint.getRadians(), 60));

        logData();

        WristState snapWristState;
        synchronized(this){
            snapWristState = wristState;
        }

        switch(snapWristState){
            case OFF:
                jog(0);
                break;
            case JOG:
                jog(jogVal);
                break;
            case SETPOINT:
                setState();
                break;
            case STOW:
                setState();
                break;
        }
    }

    public void jog(double val){
        jogVal = val;
    }



    public void setHolding(boolean hold){
        setGains();
        holdAtWantedState = hold;

        setpoint = new Rotation2d(CommonConversions.stepsToRadians(wristMotor.getSelectedSensorPosition(), 60));
    }

    public void setSetpoint(Rotation2d requestedAngle){
        setpoint = requestedAngle;
    }

    private void setState(){
        //double armFF  = WristConstants.ARM_FF.calculate(setpoint.getRadians(), Math.PI/2,Math.PI);
        //wristMotor.set(ControlMode.MotionMagic,CommonConversions.radiansToSteps(setpoint.getRadians(), 60),DemandType.ArbitraryFeedForward,armFF);
        //wristMotor.setVoltage(armFF);
        wristMotor.set(ControlMode.MotionMagic,CommonConversions.radiansToSteps(setpoint.getRadians(), 60));
    }

    public void setGains(){
        wristMotor.config_kP(0, WristConstants.WRIST_CONTROLLER_KP);
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
        config.slot0.kP = WristConstants.WRIST_CONTROLLER_KP;
        config.motionCruiseVelocity = 30000;
        config.motionAcceleration = 15000;
        wristMotor.configAllSettings(config);

        zeroOnboardEncoder();

        //wristMotor.configForwardSoftLimitThreshold(WristConstants.SOFT_LIMIT_FORWARD);
        //wristMotor.configForwardSoftLimitEnable(true);
        wristMotor.configForwardSoftLimitThreshold(WristConstants.SOFT_LIMIT_REVERSE);
        wristMotor.configForwardSoftLimitEnable(true);

    }

    public void setWristState(WristState state){
        wristState = state;

        if(state==WristState.STOW)
            setpoint = Rotation2d.fromDegrees(-76);
    }

    private void logData(){
        SmartDashboard.putNumber("Wrist Angle", Units.radiansToDegrees(CommonConversions.stepsToRadians(wristMotor.getSelectedSensorPosition(), 60)));
        SmartDashboard.putNumber("Wrist Onboard Sensor Position", wristMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("Wrist Falcon Temp", wristMotor.getTemperature());
        SmartDashboard.putBoolean("Wrist Holding", holdAtWantedState);

    }

}
