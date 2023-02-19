package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
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
        SETPOINT,
        ZERO
    }

    private WPI_TalonFX wristMotor;
    private DutyCycleEncoder wristEncoder;
    private Rotation2d setpoint = Rotation2d.fromDegrees(0);
    private static Wrist instance = new Wrist();
    private boolean holdAtWantedState;
    private double jogVal;
    private WristState wristState;
    private DigitalInput magSensor;

    
    public Wrist(){
        jogVal = 0;
        wristMotor = new WPI_TalonFX(WristConstants.DEVICE_ID_WRIST);
  
        holdAtWantedState = false;

        magSensor = new DigitalInput(WristConstants.DEVICE_ID_MAG_SENSOR);

        configWristDefault();

        wristState = WristState.STOW;
        
        //SmartDashboard.putNumber("Wrist setpoint", -117);
        SmartDashboard.putNumber("mid score", -135);
       
    }

    public static Wrist getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        if(holdAtWantedState)
            wristMotor.set(ControlMode.MotionMagic,CommonConversions.radiansToSteps(setpoint.getRadians(), 96));

        logData();

        switch(wristState){
            case OFF:
                jog(0);
                break;
            case JOG:
                jog(jogVal);
                wristMotor.set(ControlMode.PercentOutput, jogVal);
                break;
            case SETPOINT:
                setState();
                break;
            case STOW:
                setState();
                break;
            case ZERO:
                zeroRoutine();
                break;
        }
    }

    public void jog(double val){
        jogVal = val;
    }

    public void setSetpoint(Rotation2d requestedAngle){
        setWristState(WristState.SETPOINT);
        setpoint = requestedAngle;
    }

    public boolean zeroRoutine(){
        if(!magSensor.get()){ //assuming !get() means not triggered
            wristMotor.set(ControlMode.PercentOutput, 0.1);
            return false;
        }
        
        else{
            wristMotor.set(ControlMode.PercentOutput, 0);
            zeroOnboardEncoder();
            return true;
        }
        //this returns if the wrist has been zeroed
    }

    private void setState(){
        wristMotor.set(ControlMode.MotionMagic,CommonConversions.radiansToSteps(setpoint.getRadians(), 96));
    }

    public void setGains(){
        wristMotor.config_kP(0, WristConstants.WRIST_CONTROLLER_KP);
    }


    public boolean atSetpoint(){
        return Math.abs(Units.radiansToDegrees(CommonConversions.stepsToRadians(wristMotor.getSelectedSensorPosition(), 96)) - Units.radiansToDegrees(setpoint.getRadians())) < 3;
    }

    public void zeroOnboardEncoder(){
        wristMotor.setSelectedSensorPosition(0);
    }

    private void configWristDefault(){
        //wristEncoder.setDistancePerRotation(2*Math.PI);

        wristMotor.configFactoryDefault();


        wristMotor.setInverted(true);
        wristMotor.setNeutralMode(NeutralMode.Brake);
        wristMotor.configPeakOutputForward(WristConstants.WRIST_PEAK_OUTPUT_FORWARD);
        wristMotor.configPeakOutputReverse(WristConstants.WRIST_PEAK_OUTPUT_REVERSE);
        wristMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10, 10, 1));
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.slot0.kP = WristConstants.WRIST_CONTROLLER_KP;
        config.motionCruiseVelocity = 60000;
        config.motionAcceleration = 60000;
        wristMotor.configAllSettings(config);

        zeroOnboardEncoder();

        //wristMotor.configForwardSoftLimitThreshold(WristConstants.SOFT_LIMIT_FORWARD);
        //wristMotor.configForwardSoftLimitEnable(true);
        //wristMotor.configForwardSoftLimitThreshold(WristConstants.SOFT_LIMIT_REVERSE);
        //wristMotor.configForwardSoftLimitEnable(true);

    }

    public void setWristState(WristState state){
        wristState = state;

        if(state==WristState.STOW)
            setpoint = Rotation2d.fromDegrees(-7);
    }

    private void logData(){
        SmartDashboard.putString("Wrist State", wristState.toString());
        SmartDashboard.putBoolean("Mag Sensor", magSensor.get());
        SmartDashboard.putNumber("Wrist Setpoint", setpoint.getDegrees());
    }

}
