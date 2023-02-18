package frc.robot.subsystems;



import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;


public class Elevator extends SubsystemBase {

    private CANSparkMax elevatorMotor;
    private SparkMaxPIDController elevatorController;
    private RelativeEncoder elevatorEncoder;
    private CANSparkMax elevatorSlave;

    private double setpoint;

    private static Elevator instance = new Elevator();

    private DigitalInput limitSwitch;


    private DutyCycleEncoder absoluteEncoder;

    private ElevatorState state = ElevatorState.OFF;

    private double jogInput = 0;
    
    public enum ElevatorState{
        JOG,
        SETPOINT,
        ZERO,
        STOW,
        OFF
    }

    public Elevator(){
        elevatorMotor = new CANSparkMax(ElevatorConstants.DEVICE_ID_ELEVATOR_MASTER, MotorType.kBrushless);
        elevatorSlave = new CANSparkMax(ElevatorConstants.DEVICE_ID_ELEVATOR_SLAVE, MotorType.kBrushless);

        elevatorController = elevatorMotor.getPIDController();
        elevatorEncoder = elevatorMotor.getEncoder();

        elevatorMotor.restoreFactoryDefaults();
        elevatorSlave.restoreFactoryDefaults();

        limitSwitch = new DigitalInput(ElevatorConstants.DEVICE_ID_ELEVATOR_SWITCH);

        configElevatorMotor();

        setpoint = 0;

        elevatorSlave.follow(elevatorMotor);


        SmartDashboard.putNumber("elevator setpoint", -26);
        zeroElevator();


    }

    public static Elevator getInstance(){
        return instance;
    }

    @Override
    public void periodic() {
        switch(state){
            case OFF:
                setPercentOutput(0);
                break;
            case SETPOINT:
                goToSetpoint();
                break;
            case ZERO:
                zeroRoutine();
                break;
            case JOG:
                setPercentOutput(jogInput);
                break;
            case STOW:
                goToSetpoint();
                break;
            default:
                break;
        }
        logData();
    }

    public void setPercentOutput(double val){
        elevatorMotor.set(val);
    }

    public boolean setElevatorHeight(double setpoint){
        this.setpoint = setpoint;
        setState(ElevatorState.SETPOINT);
        configElevatorMotor();
        elevatorController.setReference(setpoint, ControlType.kSmartMotion);

        return atSetpoint();
    }


    private void goToSetpoint(){
        elevatorController.setReference(setpoint, ControlType.kSmartMotion);   
    }


    public boolean zeroRoutine(){
        if(!limitSwitch.get()){//assuming !get() means not triggered
            elevatorMotor.set(0.1);
            return false;
        }

        else{
            elevatorMotor.set(0);
            zeroElevator();
            return true;
        }
        //this returns if the elevator has been zeroed
    }


    public void setJogInput(double val){
        jogInput = val;
    }

    public void zeroElevator(){
        elevatorEncoder.setPosition(0);
    }

    public double getElevatorHeight(){
        return elevatorEncoder.getPosition();
    }

    private double getSetpoint(){
        return setpoint;
    }

    private double getError(){
        return Math.abs(setpoint - getElevatorHeight());
    }

    public boolean atSetpoint(){
        return Math.abs(setpoint - getElevatorHeight()) < ElevatorConstants.ELEVATOR_HEIGHT_TOLERANCE;
    }

    public boolean atZero(){
        return absoluteEncoder.getAbsolutePosition() == ElevatorConstants.ELEVATOR_ZERO_HEIGHT;
    }

    public void configElevatorPID(boolean useSD){

        if(useSD){
            elevatorController.setP(SmartDashboard.getNumber("Elevator kP", 0));
            elevatorController.setI(SmartDashboard.getNumber("Elevator kI", 0));
            elevatorController.setD(SmartDashboard.getNumber("Elevator kD", 0));
            elevatorController.setFF(SmartDashboard.getNumber("Elevator kF", 0));
        }
    }

    private void configElevatorMotor(){

        //elevatorMotor.setSmartCurrentLimit(20);
        //elevatorSlave.setSmartCurrentLimit(20);

        elevatorMotor.setControlFramePeriodMs(50);
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorSlave.setIdleMode(elevatorMotor.getIdleMode());

        //elevatorMotor.enableSoftLimit(null, false)
        //elevatorEncoder.setPositionConversionFactor(2*Math.PI * ElevatorConstants.ELEVATOR_GEARING);

        elevatorController.setOutputRange(-1, .35, 0);

        elevatorController.setP(.00003);
        elevatorController.setFF(0.0002);

        elevatorController.setSmartMotionMaxVelocity(9500, 0);
        elevatorController.setSmartMotionMaxAccel(10000, 0);

        elevatorController.setSmartMotionMinOutputVelocity(0, 0);
        elevatorController.setSmartMotionAllowedClosedLoopError(0, 0);
    }

    private void configElevatorDownwardConstraints(){
        elevatorController.setSmartMotionMaxVelocity(3500, 0);
        elevatorController.setSmartMotionMaxAccel(2000, 0);
    }

    private void logData(){
        SmartDashboard.putNumber("Elevator Setpoint", setpoint);
        SmartDashboard.putString("Elevator State", state.toString());
        SmartDashboard.putNumber("bruddah", elevatorEncoder.getPosition());
    }

    public void setState(ElevatorState state){
        this.state = state;

        if(state==ElevatorState.STOW){
            configElevatorDownwardConstraints();
            setpoint = 0;
        }
        else
            configElevatorMotor();
    }

}
