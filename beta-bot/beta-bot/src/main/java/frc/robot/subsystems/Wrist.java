package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
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
    private Rotation2d setpoint;
    private static Wrist instance = new Wrist();
    private double jogVal;
    private WristState wristState;
    private DigitalInput magSensor;

    
    public Wrist(){
        setpoint = Rotation2d.fromDegrees(0);
        jogVal = 0;
        wristState = WristState.STOW;

        wristMotor = new WPI_TalonFX(WristConstants.DEVICE_ID_WRIST);
        magSensor = new DigitalInput(WristConstants.DEVICE_ID_MAG_SENSOR);

        configWristDefault();
    }

    public static Wrist getInstance(){
        return instance;
    }

    @Override
    public void periodic() {

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
                if(Elevator.getInstance().getElevatorHeight()>ElevatorConstants.ELEVATOR_MAX_HEIGHT)
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

    public boolean isZeroed(){
        return magSensor.get() == WristConstants.MAG_DETECTED;
    }

    private void setState(){
        
        if(SuperStructure.getInstance().getState() == SuperStructureState.CONE_HIGH || SuperStructure.getInstance().getState() == SuperStructureState.CUBE_HIGH){
            if(Elevator.getInstance().getElevatorHeight() > WristConstants.ELEVATOR_AVOID_HEIGHT)
                wristMotor.set(ControlMode.MotionMagic,CommonConversions.radiansToSteps(WristConstants.WRIST_STOWING_ANGLE.getRadians(), WristConstants.WRIST_GEARING));
            else
                wristMotor.set(ControlMode.MotionMagic,CommonConversions.radiansToSteps(setpoint.getRadians(), WristConstants.WRIST_GEARING));
        }
        else{

            wristMotor.set(ControlMode.MotionMagic,CommonConversions.radiansToSteps(setpoint.getRadians(), WristConstants.WRIST_GEARING));
        }

        if(wristState == WristState.STOW &&magSensor.get()==WristConstants.MAG_DETECTED)
            wristMotor.setSelectedSensorPosition(0);

        
    }

    public void setGains(){
        wristMotor.config_kP(0, WristConstants.WRIST_CONTROLLER_KP);
    }


    public boolean atSetpoint(){
        return getError() <WristConstants.WRIST_CONTROLLER_TOLERANCE_DEGREES;
    }

    private double getError(){
        return Math.abs(Units.radiansToDegrees(CommonConversions.stepsToRadians(wristMotor.getSelectedSensorPosition(), Constants.WristConstants.WRIST_GEARING)) - Units.radiansToDegrees(setpoint.getRadians()));

    }

    public void zeroOnboardEncoder(){
        wristMotor.setSelectedSensorPosition(0);
    }

    private void configWristDefault(){

        wristMotor.configFactoryDefault();


        wristMotor.setInverted(true);
        wristMotor.setNeutralMode(NeutralMode.Brake);
        
        wristMotor.configPeakOutputForward(WristConstants.WRIST_PEAK_OUTPUT_FORWARD);
        wristMotor.configPeakOutputReverse(WristConstants.WRIST_PEAK_OUTPUT_REVERSE);
        wristMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, WristConstants.WRIST_CURRENT_LIMIT, WristConstants.WRIST_TRIGGER_THRESHOLD_CURRENT, 1));
        wristMotor.configVoltageCompSaturation(WristConstants.WRIST_NOMINAL_VOLTAGE);
        wristMotor.enableVoltageCompensation(true);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.slot0.kP = WristConstants.WRIST_CONTROLLER_KP;
        config.motionCruiseVelocity = WristConstants.WRIST_MOTION_VEL;
        config.motionAcceleration = WristConstants.WRIST_MOTION_ACCEL;
        wristMotor.configAllSettings(config);

        zeroOnboardEncoder();

        wristMotor.configForwardSoftLimitThreshold(WristConstants.SOFT_LIMIT_FORWARD);
        wristMotor.configForwardSoftLimitEnable(true);
        wristMotor.configForwardSoftLimitThreshold(WristConstants.SOFT_LIMIT_REVERSE);
        wristMotor.configForwardSoftLimitEnable(true);

    }

    public void setWristState(WristState state){
        wristState = state;

        if(state==WristState.STOW){
            if(Intake.getInstance().isClosed())
                setpoint = frc.robot.Constants.SuperStructureConstants.WRIST_STOW;
            else
                setpoint = WristConstants.WRIST_STOWING_ANGLE;

        }
    }

    private void logData(){
        SmartDashboard.putString("Wrist State", wristState.toString());
        SmartDashboard.putBoolean("Mag Sensor", magSensor.get());
        SmartDashboard.putNumber("Wrist Setpoint", setpoint.getDegrees());
        SmartDashboard.putNumber("Wrist Error Deg", getError());
        SmartDashboard.putNumber("Wrist Velocity", wristMotor.getSelectedSensorVelocity());
    }

}
