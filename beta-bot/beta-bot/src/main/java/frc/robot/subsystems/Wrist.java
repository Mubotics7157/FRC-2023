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
import frc.robot.Constants;
import frc.robot.AltConstants.IntakeConstants;
import frc.robot.AltConstants.SuperStructureConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.SuperStructure.SuperStructureState;
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
        SmartDashboard.putNumber("custom wrist", -104);
    }

    public static Wrist getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        if(holdAtWantedState)
            wristMotor.set(ControlMode.MotionMagic,CommonConversions.radiansToSteps(setpoint.getRadians(), WristConstants.WRIST_GEARING));

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
                if(Elevator.getInstance().getElevatorHeight()>-20)
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

    public void zeroRoutine(){
        if(magSensor.get() != WristConstants.MAG_DETECTED){
            wristMotor.set(ControlMode.PercentOutput, WristConstants.ZEROING_SPEED);
        }
        
        else{
            wristMotor.set(ControlMode.PercentOutput, 0);
            zeroOnboardEncoder(); 
            setWristState(WristState.STOW);
        }
    }

    private void setState(){
        wristMotor.set(ControlMode.MotionMagic,CommonConversions.radiansToSteps(setpoint.getRadians(), WristConstants.WRIST_GEARING));
        if(magSensor.get()==WristConstants.MAG_DETECTED)
            wristMotor.setSelectedSensorPosition(0);
    }

    public void setGains(){
        wristMotor.config_kP(0, WristConstants.WRIST_CONTROLLER_KP);
    }


    public boolean atSetpoint(){
        return getError() <3;
    }

    private double getError(){
        return Math.abs(Units.radiansToDegrees(CommonConversions.stepsToRadians(wristMotor.getSelectedSensorPosition(), Constants.WristConstants.WRIST_GEARING)) - Units.radiansToDegrees(setpoint.getRadians()));

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
        config.motionAcceleration = 45000;
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
            setpoint = frc.robot.Constants.SuperStructureConstants.WRIST_STOW;
    }

    private void logData(){
        SmartDashboard.putString("Wrist State", wristState.toString());
        SmartDashboard.putBoolean("Mag Sensor", magSensor.get());
        SmartDashboard.putNumber("Wrist Setpoint", setpoint.getDegrees());
        SmartDashboard.putNumber("Wrist Error Deg", getError());
    }

}
