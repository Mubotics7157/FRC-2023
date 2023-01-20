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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    //9 10   
    private WPI_TalonFX intakeMaster; 
    private WPI_TalonFX intakeSlave;
    private CANSparkMax intakeAngle;
    private SparkMaxPIDController intakeController;
    private RelativeEncoder intakeEncoder;
    private IntakeState intakeState;

    private static Intake instance = new Intake();

    public Intake(){
        
        intakeState = IntakeState.OFF;

        intakeMaster = new WPI_TalonFX(19);
        intakeSlave = new WPI_TalonFX(20);
        intakeAngle = new CANSparkMax(14, MotorType.kBrushless);

        intakeController = intakeAngle.getPIDController();
        intakeEncoder = intakeAngle.getEncoder();

        intakeMaster.configFactoryDefault();
        intakeSlave.configFactoryDefault();
        intakeAngle.restoreFactoryDefaults();

        intakeController.setP(0);
        intakeController.setFF(0);

        intakeController.setOutputRange(-0.5, 0.5);

        intakeEncoder.setPosition(0);

        //intakeAngle.setSoftLimit(SoftLimitDirection.kForward, 5000);
        //intakeAngle.setSoftLimit(SoftLimitDirection.kReverse, 0);


        //intakeMaster.setSmartCurrentLimit(20);
        //intakeSlave.setSmartCurrentLimit(20);

        intakeMaster.setInverted(true);
        intakeSlave.setInverted(false);
        intakeAngle.setInverted(false);

        intakeMaster.setNeutralMode(NeutralMode.Brake);
        intakeSlave.setNeutralMode(NeutralMode.Brake);
        intakeAngle.setIdleMode(IdleMode.kBrake);

        intakeSlave.follow(intakeMaster);

        //TODO: ask harshal abt further clarification for regular intake current limit

        //intakeSlave.follow(intakeMaster);
        
        
    }

    public static Intake getInstance(){
        return instance;
    }


    @Override
    public void periodic() {
        /*
        switch(intakeState){
            case OFF:
                currentLimit(false);
                setMotors(0);
                break;
            case INTAKE_CUBE:
                currentLimit(false);
                setMotors(SmartDashboard.getNumber("Intake speed", 0.5));
                setAngle(SmartDashboard.getNumber("Intake Angle Degrees", 0));
                //value to be determined :P
                break;
            case OUTTAKE_CUBE:
                currentLimit(false);
                setMotors(-SmartDashboard.getNumber("Intake Speed", 0.5));
                setAngle(SmartDashboard.getNumber("Intake Angle Degrees", 0));
                //value to be detemermined :P
                break;
            case INTAKE_CONE:
                currentLimit(false);
                setMotors(SmartDashboard.getNumber("Intake Speed", 0.5));
                setAngle(SmartDashboard.getNumber("Intake Angle Degrees", 0));
                break;
            case OUTTAKE_CONE:
                currentLimit(false);
                setMotors(-SmartDashboard.getNumber("Intake Speed", 0.5));
                setAngle(SmartDashboard.getNumber("Intake Angle Degrees", 0));
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

        */
    }

    public void setIntakeState(IntakeState state){
        intakeState = state;
    }

    public void setMotors(double speed){
        intakeMaster.set(speed);
    }

    public void setAngle(double angle){
        double value = angle * 20;
        intakeController.setReference(value, com.revrobotics.CANSparkMax.ControlType.kPosition);
    }

    public void jogJawMotor(double val){
        intakeAngle.set(val);
    }

    public void currentLimit(boolean enable){
        if(enable){
            intakeMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 2, 10, .25));
            intakeSlave.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 2, 10, .25));
            intakeAngle.setSmartCurrentLimit(2, 20);
        }
        else{
            intakeMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 2, 10, .25));
            intakeSlave .configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 2, 10, .25));
            intakeAngle.setSmartCurrentLimit(40);
        }
    }

    public double getIntakeAngle(){
        return intakeEncoder.getPosition() / 20;
    }
}
