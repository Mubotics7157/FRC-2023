package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Ultrasonic;
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

    private DoubleSolenoid solenoid; 
    private SparkMaxPIDController intakeController;
    private RelativeEncoder intakeEncoder;
    private IntakeState intakeState;

    //private Ultrasonic tof;
    private  MedianFilter filter;

    private static Intake instance = new Intake();

    public Intake(){
        solenoid = new DoubleSolenoid(IntakeConstants.DEVICE_ID_REV_PH,PneumaticsModuleType.REVPH, IntakeConstants.DEVICE_ID_SOLENOID_FORWARD, IntakeConstants.DEVICE_ID_SOLENOID_REVERSE);

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

        //tof = new Ultrasonic(IntakeConstants.ULTRASONIC_PING_PORT,IntakeConstants.ULTRASONIC_RESPONSE_PORT);
        filter = new MedianFilter(IntakeConstants.FILTER_SAMPLE_WINDOW);


        SmartDashboard.putNumber("Intake speed", 0.5);
        
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
        
        SmartDashboard.putString("intake state", snapIntakeState.toString());
        
        switch(snapIntakeState){
            case OFF:
                setMotors(0);
                break;
            case INTAKE_CUBE:
                setMotors(IntakeConstants.CUBE_INTAKE_SPEED);
                //toggleIntake(false);
                //value to be determined :P
                break;
            case OUTTAKE_CUBE:
                setMotors(IntakeConstants.CUBE_OUTTAKE_SPEED);
                toggleIntake(false);
                //value to be detemermined :P
                break;
            case INTAKE_CONE:
                setMotors(IntakeConstants.CONE_INTAKE_SPEED);
                toggleIntake(true);
                break;
            case OUTTAKE_CONE:
                setMotors(IntakeConstants.CONE_OUTTAKE_SPEED);
                toggleIntake(true);
                break;
            case INTAKE:
                setMotors(SmartDashboard.getNumber("Intake Speed", 0.5));
                break;
            case OUTTAKE:
                setMotors(-SmartDashboard.getNumber("Intake Speed", 0.5));
                break;
            case IDLE:
                setMotors(IntakeConstants.IDLE_SPEED);
                break;
        }
        
    }

    public void setIntakeState(IntakeState state){
        if(state==IntakeState.IDLE)
            currentLimit(true);
        else
            currentLimit(false);
        intakeState = state;
    }

    public void setMotors(double speed){
        intakeMaster.set(speed);
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

    public boolean isClosed(){
        if(solenoid.get() == Value.kForward){
            return false;
        }
        else if(solenoid.get() == Value.kReverse){
            return true;
        }
        else
            return false;
    }

    public void currentLimit(boolean enable){
        if(enable){
            intakeMaster.setSmartCurrentLimit(2, 10);
        }
        else{
            intakeMaster.setSmartCurrentLimit(50);
        }
    }

    public double getObjDistance(){
        //return filter.calculate(tof.getRangeInches());
        return 0;
    }

}
