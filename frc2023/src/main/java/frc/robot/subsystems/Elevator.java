package frc.robot.subsystems;

import java.util.LinkedHashMap;

import org.opencv.core.Mat;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Robot.ObjectType;


public class Elevator extends SubsystemBase {

    private CANSparkMax elevatorMotor;
    private SparkMaxPIDController elevatorController;
    private RelativeEncoder elevatorEncoder;

    private LinkedHashMap <ElevatorSetpoint,Integer> elevatorHeights = new LinkedHashMap<>();
    private ElevatorSetpoint currentState;

    private static Elevator instance;

    private int setpoint;

    public enum ElevatorSetpoint{
        GROUND_INTAKE,
        STOW,
        CONE_LOW,
        CONE_MID,
        CONE_HIGH,
        CUBE_LOW,
        CUBE_MID,
        CUBE_HIGH
    }


    public Elevator(){
        elevatorMotor = new CANSparkMax(ElevatorConstants.DEVICE_ID_ELEVATOR, MotorType.kBrushless);
        elevatorController = elevatorMotor.getPIDController();
        elevatorEncoder = elevatorMotor.getEncoder();

        configElevatorMotor();

        instance = new Elevator();

        elevatorHeights.put(ElevatorSetpoint.STOW, 0);
        elevatorHeights.put(ElevatorSetpoint.GROUND_INTAKE, 0);

        currentState = ElevatorSetpoint.STOW;

        setpoint = 0;

    }

    public static Elevator getInstance(){
        return instance;
    }

    public boolean setState(ElevatorSetpoint wantedState){
        setpoint = elevatorHeights.get(wantedState);

        elevatorController.setReference(setpoint, ControlType.kPosition);

        return atSetpoint();
    }

    private boolean atSetpoint(){
        return Math.abs(setpoint - getElevatorHeight()) < ElevatorConstants.ELEVATOR_HEIGHT_TOLERANCE;
    }

    private double getElevatorHeight(){
        return elevatorEncoder.getPosition();
    }

    private void configElevatorPID(boolean useSD){
        elevatorController.setP(SmartDashboard.getNumber("Elevator kP", 0));
        elevatorController.setI(SmartDashboard.getNumber("Elevator kI", 0));
        elevatorController.setD(SmartDashboard.getNumber("Elevator kD", 0));
        elevatorController.setFF(SmartDashboard.getNumber("Elevator kF", 0));
    }

    private void configElevatorMotor(){
        elevatorMotor.setSmartCurrentLimit(30);
        elevatorMotor.setControlFramePeriodMs(50);
        elevatorMotor.setIdleMode(IdleMode.kBrake);
        //elevatorMotor.enableSoftLimit(null, false)
        elevatorController.setP(0);
        elevatorController.setI(0);
        elevatorController.setD(0);
        elevatorController.setFF(0);
        elevatorEncoder.setPositionConversionFactor(2*Math.PI * ElevatorConstants.ELEVATOR_GEARING);
    }

}
