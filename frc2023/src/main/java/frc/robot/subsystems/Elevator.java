package frc.robot.subsystems;

import java.util.LinkedHashMap;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;


public class Elevator extends SubsystemBase {

    private CANSparkMax elevatorMotor;
    private SparkMaxPIDController elevatorController;
    private RelativeEncoder elevatorEncoder;

    private LinkedHashMap <ElevatorSetpoint,Integer> elevatorHeights = new LinkedHashMap<>();
    private ElevatorSetpoint currentState;

    private static Elevator instance;

    public enum ElevatorSetpoint{
        GROUND_INTAKE,
        STOW,
        LOW,
        MID,
        HIGH
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
    }

    public static Elevator getInstance(){
        return instance;
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
