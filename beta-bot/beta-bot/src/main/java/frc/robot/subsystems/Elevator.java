package frc.robot.subsystems;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
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
import frc.robot.Constants.SuperStructureConstants;


public class Elevator extends SubsystemBase {

    private WPI_TalonFX elevatorMotor;
    private WPI_TalonFX elevatorSlave;

    private double setpoint;

    private static Elevator instance = new Elevator();

    private DigitalInput limitSwitch;

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
        elevatorMotor = new WPI_TalonFX(ElevatorConstants.DEVICE_ID_ELEVATOR_MASTER);
        elevatorSlave = new WPI_TalonFX(ElevatorConstants.DEVICE_ID_ELEVATOR_SLAVE);

        elevatorMotor.configFactoryDefault();
        elevatorSlave.configFactoryDefault();

        limitSwitch = new DigitalInput(ElevatorConstants.DEVICE_ID_ELEVATOR_SWITCH);

        configElevatorMotor();

        setpoint = 0;

        elevatorSlave.follow(elevatorMotor);


        //SmartDashboard.putNumber("elevator setpoint", -26);
        SmartDashboard.putNumber("custom elevator", -24.5);
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
        elevatorMotor.set(ControlMode.MotionMagic, setpoint);

        return atSetpoint();
    }


    private void goToSetpoint(){
        elevatorMotor.set(ControlMode.MotionMagic, setpoint * 2048);   
    }


    public void zeroRoutine(){
        if(limitSwitch.get() != ElevatorConstants.MAG_DETECTED){
            elevatorMotor.set(ElevatorConstants.ZEROING_SPEED);
            //go down until mag is hit
        }

        else{
            elevatorMotor.set(0);
            zeroElevator();
            setState(ElevatorState.STOW);
        }
    }


    public void setJogInput(double val){
        jogInput = val;
    }

    public void zeroElevator(){
        elevatorMotor.setSelectedSensorPosition(0);
    }

    public double getElevatorHeight(){
        return elevatorMotor.getSelectedSensorPosition() / 2048;
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

    public void configElevatorPID(boolean useSD){

        if(useSD){
            elevatorMotor.config_kP(0, SmartDashboard.getNumber("Elevator kP", 0));
            elevatorMotor.config_kI(0, SmartDashboard.getNumber("Elevator kI", 0));
            elevatorMotor.config_kD(0, SmartDashboard.getNumber("Elevator kD", 0));
            elevatorMotor.config_kF(0, SmartDashboard.getNumber("Elevator kF", 0));
        }
    }

    private void configElevatorMotor(){

        //elevatorMotor.setSmartCurrentLimit(20);
        //elevatorSlave.setSmartCurrentLimit(20);
        elevatorMotor.setInverted(true);
        elevatorSlave.setInverted(true);

        elevatorMotor.configVoltageCompSaturation(10);
        elevatorSlave.configVoltageCompSaturation(10);

        //elevatorMotor.setStatusFramePeriod(0, 50);
        elevatorMotor.setNeutralMode(NeutralMode.Brake);
        elevatorSlave.setNeutralMode(NeutralMode.Brake);

        //elevatorMotor.enableSoftLimit(null, false)
        //elevatorEncoder.setPositionConversionFactor(2*Math.PI * ElevatorConstants.ELEVATOR_GEARING);

        elevatorMotor.configPeakOutputForward(1);
        elevatorMotor.configPeakOutputReverse(-1);

        elevatorMotor.config_kP(0, .00003);
        elevatorMotor.config_kF(0, 0.0002);

        elevatorMotor.configMotionAcceleration((10500 / 600) * 2048, 0);
        elevatorMotor.configMotionCruiseVelocity((11000 / 600) * 2048, 0);

        //elevatorMotor.setSmartMotionMinOutputVelocity(0, 0);
        elevatorMotor.configAllowableClosedloopError(0, 0);
    }

    private void configElevatorDownwardConstraints(){
        elevatorMotor.configMotionCruiseVelocity((8000 / 600) * 2048, 0);
        elevatorMotor.configMotionAcceleration((8000 / 600) * 2048, 0);
    }

    private void logData(){
        SmartDashboard.putNumber("Elevator Setpoint", setpoint);
        SmartDashboard.putString("Elevator State", state.toString());
        SmartDashboard.putNumber("Elevator Position", elevatorMotor.getSelectedSensorPosition() / 2048);
        SmartDashboard.putNumber("Elevator Height", getElevatorHeight());
    }

    public void setState(ElevatorState state){
        this.state = state;

        if(state==ElevatorState.STOW){
            configElevatorDownwardConstraints();
            setpoint = SuperStructureConstants.ELEVATOR_STOW;
        }
        else
            configElevatorMotor();
    }

}
