package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    public enum IntakeState{
        OFF,
        INTAKE_CUBE,
        OUTTAKE_CUBE,
        INTAKE_CONE,
        OUTTAKE_CONE,
        INTAKE,
        OUTTAKE,
        IDLE
    }

    private CANSparkMax intakeMaster; 
    private CANSparkMax intakeSlave;

    private DoubleSolenoid solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, IntakeConstants.DEVICE_ID_SOLENOID_FORWARD, IntakeConstants.DEVICE_ID_SOLENOID_REVERSE);

    private SparkMaxPIDController intakeController;
    private RelativeEncoder intakeEncoder;
    private IntakeState intakeState;

    private static Intake instance = new Intake();

    public Intake(){
        
        intakeState = IntakeState.OFF;

        intakeMaster = new CANSparkMax(IntakeConstants.DEVICE_ID_INTAKE_MASTER, MotorType.kBrushless);
        intakeSlave = new CANSparkMax(IntakeConstants.DEVICE_ID_INTAKE_SLAVE, MotorType.kBrushless);

        intakeController = intakeMaster.getPIDController();
        intakeEncoder = intakeMaster.getEncoder();

        intakeMaster.restoreFactoryDefaults();
        intakeSlave.restoreFactoryDefaults();

        intakeController.setP(0);
        intakeController.setFF(0);

        //intakeController.setOutputRange(-0.5, 0.5);

        //intakeEncoder.setPosition(0);

        //intakeAngle.setSoftLimit(SoftLimitDirection.kForward, 5000);
        //intakeAngle.setSoftLimit(SoftLimitDirection.kReverse, 0);


        //intakeMaster.setSmartCurrentLimit(20);
        //intakeSlave.setSmartCurrentLimit(20);

        intakeMaster.setInverted(false);
        intakeSlave.setInverted(!intakeMaster.getInverted());

        intakeSlave.follow(intakeMaster);

        intakeMaster.setIdleMode(IdleMode.kBrake);
        intakeSlave.setIdleMode(intakeMaster.getIdleMode());

        //TODO: ask harshal abt further clarification for regular intake current limit
        
    }

    public static Intake getInstance(){
        return instance;
    }


    @Override
    public void periodic() {

        IntakeState snapIntakeState;       
        synchronized(this){
            snapIntakeState = intakeState;
        }
        
        switch(snapIntakeState){
            case OFF:
                currentLimit(false);
                setMotors(0);
                break;
            case INTAKE_CUBE:
                currentLimit(false);
                setMotors(SmartDashboard.getNumber("Intake speed", 0.5));
                toggleIntake(false);
                //value to be determined :P
                break;
            case OUTTAKE_CUBE:
                currentLimit(false);
                setMotors(-SmartDashboard.getNumber("Intake Speed", 0.5));
                toggleIntake(false);
                //value to be detemermined :P
                break;
            case INTAKE_CONE:
                currentLimit(false);
                setMotors(SmartDashboard.getNumber("Intake Speed", 0.5));
                toggleIntake(true);
                break;
            case OUTTAKE_CONE:
                currentLimit(false);
                setMotors(-SmartDashboard.getNumber("Intake Speed", 0.5));
                toggleIntake(true);
                break;
            case INTAKE:
                currentLimit(false);
                setMotors(SmartDashboard.getNumber("Intake Speed", 0.5));
                break;
            case OUTTAKE:
                currentLimit(false);
                setMotors(SmartDashboard.getNumber("Intake Speed", 0.5));
                break;
            case IDLE:
                currentLimit(true);
                setMotors(.1);
                break;
        }
        
    }

    public void setIntakeState(IntakeState state){
        intakeState = state;
    }

    public void setMotors(double speed){
        intakeMaster.set(speed);
        intakeSlave.set(speed);
    }

    public void toggleIntake(boolean forward){
        
        if(forward)
            solenoid.set(Value.kForward);
        else
            solenoid.set(Value.kReverse);
        
    }


    public void currentLimit(boolean enable){
        if(enable){
            intakeMaster.setSmartCurrentLimit(2, 10);
            intakeSlave.setSmartCurrentLimit(2, 10);
        }
        else{
            intakeMaster.setSmartCurrentLimit(50);
            intakeSlave.setSmartCurrentLimit(50);
        }
    }
}
