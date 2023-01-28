package frc.robot.subsystems;

import java.util.LinkedHashMap;


import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.util.OrangeUtility;


public class Elevator extends SubsystemBase {

    private CANSparkMax elevatorMotor;
    private SparkMaxPIDController elevatorController;
    private RelativeEncoder elevatorEncoder;
    private CANSparkMax elevatorSlave;

    private LinkedHashMap <ElevatorSetpoint,Integer> elevatorHeights = new LinkedHashMap<>();
    private ElevatorSetpoint currentState;
    private double setpoint;

    private static Elevator instance = new Elevator();

    private ShuffleboardTab tab = Shuffleboard.getTab("Elevator");

    private DutyCycleEncoder absoluteEncoder;

    private ElevatorState state = ElevatorState.OFF;

    private double jogInput = 0;
    
    public enum ElevatorState{
        JOG,
        SETPOINT,
        ZERO,
        OFF
    }

    public enum ElevatorSetpoint{
        STOW,
        GROUND_INTAKE,
        CONE_LOW,
        CONE_MID,
        CONE_HIGH,
        CUBE_LOW,
        CUBE_MID,
        CUBE_HIGH
    }

    public Elevator(){
        elevatorMotor = new CANSparkMax(ElevatorConstants.DEVICE_ID_ELEVATOR_MASTER, MotorType.kBrushless);
        elevatorSlave = new CANSparkMax(ElevatorConstants.DEVICE_ID_ELEVATOR_SLAVE, MotorType.kBrushless);
        elevatorController = elevatorMotor.getPIDController();
        elevatorEncoder = elevatorMotor.getEncoder();

        elevatorMotor.restoreFactoryDefaults();
        elevatorSlave.restoreFactoryDefaults();

        configElevatorMotor();
        currentState = ElevatorSetpoint.STOW;

        setpoint = 0;

        elevatorSlave.follow(elevatorMotor);

        //elevatorHeights.put(ElevatorSetpoint.STOW, 0);
        //elevatorHeights.put(ElevatorSetpoint.GROUND_INTAKE, 0);

        OrangeUtility.sleep(1000);
        SmartDashboard.putNumber("elevator setpoint", 0);

        elevatorController.setOutputRange(-.25, .25, 0);

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
                setState(setpoint);
                break;
            case ZERO:
                //TODO: add zeroing routine
                break;
            case JOG:
                setPercentOutput(jogInput);
                break;
            default:
                break;
        }

        logData();
    }

    public void setPercentOutput(double val){
        elevatorMotor.set(val);
    }

    public boolean setState(ElevatorSetpoint wantedState){
        setState(ElevatorState.SETPOINT);
        setpoint = elevatorHeights.get(wantedState);
        elevatorController.setReference(setpoint, ControlType.kPosition);

        return atSetpoint();
    }

    public void setState(ElevatorState state){
        this.state = state;
    }

    public boolean setState(double setpoint){
        setState(ElevatorState.SETPOINT);

        this.setpoint = setpoint;

        elevatorController.setReference(setpoint, ControlType.kPosition);

        return atSetpoint();
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

    private void 
    configElevatorMotor(){

        //elevatorMotor.setSmartCurrentLimit(20);
        //elevatorSlave.setSmartCurrentLimit(20);
        elevatorMotor.setControlFramePeriodMs(50);
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        elevatorSlave.setIdleMode(elevatorMotor.getIdleMode());
        //elevatorMotor.enableSoftLimit(null, false)
        elevatorController.setP(.005);
        elevatorController.setI(0);
        elevatorController.setD(0);
        elevatorController.setFF(0);
        elevatorEncoder.setPositionConversionFactor(2*Math.PI * ElevatorConstants.ELEVATOR_GEARING);

        // elevatorController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        // elevatorController.setSmartMotionMaxAccel(setpoint, 0);
        // elevatorController.setSmartMotionMaxVelocity(setpoint, 0);
    }

    private void logData(){
        //Shuffleboard.getTab("elevator").add("Elevator Setpoint", setpoint);
        //Shuffleboard.getTab("elevator").add("Elevator State", getCurrentState().toString());
        // SmartDashboard.putNumber("Elevator Setpoint", setpoint);
        // SmartDashboard.putString("Elevator State", getCurrentState().toString());
    }

}
