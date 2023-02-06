package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
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

    private DoubleSolenoid solenoid = new DoubleSolenoid(IntakeConstants.DEVICE_ID_REV_PH,PneumaticsModuleType.REVPH, IntakeConstants.DEVICE_ID_SOLENOID_FORWARD, IntakeConstants.DEVICE_ID_SOLENOID_REVERSE);

    private SparkMaxPIDController intakeController;
    private RelativeEncoder intakeEncoder;
    private IntakeState intakeState;

    private AnalogInput tof;


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

        //intakeAngle.setSoftLimit(SoftLimitDirection.kForward, 5000);
        //intakeAngle.setSoftLimit(SoftLimitDirection.kReverse, 0);


        //intakeMaster.setSmartCurrentLimit(20);
        //intakeSlave.setSmartCurrentLimit(20);

        intakeMaster.setInverted(true);
        intakeSlave.setInverted(!intakeMaster.getInverted());

        intakeSlave.follow(intakeMaster);

        intakeMaster.setIdleMode(IdleMode.kBrake);
        intakeSlave.setIdleMode(intakeMaster.getIdleMode());

        tof = new AnalogInput(0);


        SmartDashboard.putNumber("Intake speed", 0);
        SmartDashboard.putNumber("ratio", 1);
        
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
                setMotors(0, 0);
                break;
            case INTAKE_CUBE:
                currentLimit(false);
                setMotors(SmartDashboard.getNumber("Intake speed", 0.5), SmartDashboard.getNumber("ratio", 0));
                toggleIntake(false);
                //value to be determined :P
                break;
            case OUTTAKE_CUBE:
                currentLimit(false);
                setMotors(-SmartDashboard.getNumber("Intake Speed", 0.5), SmartDashboard.getNumber("ratio", 0));
                toggleIntake(false);
                //value to be detemermined :P
                break;
            case INTAKE_CONE:
                currentLimit(false);
                setMotors(SmartDashboard.getNumber("Intake Speed", 0.5), SmartDashboard.getNumber("ratio", 0));
                toggleIntake(true);
                break;
            case OUTTAKE_CONE:
                currentLimit(false);
                setMotors(-SmartDashboard.getNumber("Intake Speed", 0.5), SmartDashboard.getNumber("ratio", 0));
                toggleIntake(true);
                break;
            case INTAKE:
                currentLimit(false);
                setMotors(SmartDashboard.getNumber("Intake Speed", 0.5), SmartDashboard.getNumber("ratio", 0));
                break;
            case OUTTAKE:
                currentLimit(false);
                setMotors(-SmartDashboard.getNumber("Intake Speed", 0.5), SmartDashboard.getNumber("ratio", 0));
                break;
            case IDLE:
                currentLimit(true);
                setMotors(.15, 1);
                break;
        }
        
    }

    public void setIntakeState(IntakeState state){
        intakeState = state;
    }

    public void setMotors(double speed, double ratio){
        intakeMaster.set(speed * ratio);
        intakeSlave.set(speed);
    }

    public void toggleIntake(boolean forward){
        
        if(forward)
            solenoid.set(Value.kForward);
        else
            solenoid.set(Value.kReverse);
        
    }

    public void closeJaws(){
        solenoid.set(Value.kReverse);
    }

    public void openJaws(){
        solenoid.set(Value.kForward);
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

    public double getObjDistance(){
        return tof.getAverageValue();
    }

}
