package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SuperStructureConstants;

public class Intake extends SubsystemBase {

    public enum IntakeState{
        OFF,
        INTAKE_CUBE,
        OUTTAKE_CUBE_MID,
        OUTTAKE_CUBE_HIGH,
        INTAKE_CONE,
        OUTTAKE_CONE,
        INTAKE,
        OUTTAKE,
        IDLE,
        CONE_SNIPER,
        CUSTOM
    }

    private CANSparkMax intakeMaster; 
    private CANSparkMax intakeSlave;

    private DoubleSolenoid solenoid; 

    private SparkMaxPIDController topController;
    private SparkMaxPIDController botController;

    private RelativeEncoder topEncoder;
    private RelativeEncoder botEncoder;
    private IntakeState intakeState;

    //private Ultrasonic tof;
    private  MedianFilter filter;

    private static Intake instance = new Intake();

    private InterpolatingTreeMap<Double,Double> distanceMap = new InterpolatingTreeMap<>();

    public Intake(){
        solenoid = new DoubleSolenoid(IntakeConstants.DEVICE_ID_PCM, IntakeConstants.PNEUMATICS_MODULE_TYPE, IntakeConstants.DEVICE_ID_SOLENOID_FORWARD, IntakeConstants.DEVICE_ID_SOLENOID_REVERSE);

        intakeState = IntakeState.OFF;

        intakeMaster = new CANSparkMax(IntakeConstants.DEVICE_ID_INTAKE_MASTER, MotorType.kBrushless);
        intakeSlave = new CANSparkMax(IntakeConstants.DEVICE_ID_INTAKE_SLAVE, MotorType.kBrushless);

        topController = intakeMaster.getPIDController();
        topEncoder = intakeMaster.getEncoder();

        botController = intakeSlave.getPIDController();
        botEncoder = intakeSlave.getEncoder();

        intakeMaster.restoreFactoryDefaults();
        intakeSlave.restoreFactoryDefaults();

        topController.setP(IntakeConstants.TOP_ROLLER_KP);
        topController.setFF(IntakeConstants.TOP_ROLLER_KF);

        botController.setP(IntakeConstants.TOP_ROLLER_KP);
        botController.setFF(IntakeConstants.TOP_ROLLER_KF);

        intakeMaster.setInverted(IntakeConstants.INVERT_MASTER);
        intakeSlave.setInverted(!intakeMaster.getInverted());
        intakeSlave.follow(intakeMaster);
        intakeMaster.setIdleMode(IdleMode.kBrake);
        intakeSlave.setIdleMode(intakeMaster.getIdleMode());

        //tof = new Ultrasonic(IntakeConstants.ULTRASONIC_PING_PORT,IntakeConstants.ULTRASONIC_RESPONSE_PORT);
        filter = new MedianFilter(IntakeConstants.FILTER_SAMPLE_WINDOW);

        intakeMaster.setSmartCurrentLimit(100);

        intakeMaster.enableVoltageCompensation(10);
        intakeSlave.enableVoltageCompensation(10);

        SmartDashboard.putNumber("custom intake", -1000);  
    }

    public static Intake getInstance(){
        return instance;
    }

    public void initMap(){
        distanceMap.put(0.0, 0.0);
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
                setSpeed(IntakeConstants.CUBE_INTAKE_SPEED);
                toggleIntake(false);
                break;
            case OUTTAKE_CUBE_MID:
                setSpeed(IntakeConstants.CUBE_OUTTAKE_MID);
                //toggleIntake(false);
                break;
            case OUTTAKE_CUBE_HIGH:
                setSpeed(IntakeConstants.CUBE_OUTTAKE_HIGH);
                break;
            case INTAKE_CONE:
                setSpeed(IntakeConstants.CONE_INTAKE_SPEED);
                //setSpeed(2000);
                toggleIntake(true);
                break;
            case OUTTAKE_CONE:
                setSpeed(-3000);
                //toggleIntake(true);
                break;
            case INTAKE:
                setSpeed(IntakeConstants.CONE_INTAKE_SPEED);
                break;
            case OUTTAKE:
                setSpeed(-IntakeConstants.CONE_INTAKE_SPEED);
                //setSpeed(-3000);
                break;
            case IDLE:
                setSpeed(IntakeConstants.IDLE_SPEED);
                break;
            case CONE_SNIPER:
                setSpeed(IntakeConstants.CONE_SNIPER_SPEED);
                break;
            case CUSTOM:
                setSpeed(SmartDashboard.getNumber("custom intake", -1000));
                break;
        }
        
    }


    public void setIntakeState(IntakeState state){
        if(state != IntakeState.IDLE)//state==IntakeState.OUTTAKE_CONE || state==IntakeState.OUTTAKE_CUBE_MID || state==IntakeState.OUTTAKE_CUBE_HIGH || state==IntakeState.INTAKE_CONE || state==IntakeState.INTAKE_CUBE || state == IntakeState.CUSTOM)
            currentLimit(false);
        else
            currentLimit(true);
            
        intakeState = state;
    }

    public void setMotors(double speed){
        intakeMaster.set(speed);
    }

    private void setSpeed(double speedRPM){
        topController.setReference(speedRPM, com.revrobotics.CANSparkMax.ControlType.kVelocity);
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
            return true;
        }
        else if(solenoid.get() == Value.kReverse){
            return false;
        }
        else
            return false;
    }

    public void currentLimit(boolean enable){
        if(enable)
            intakeMaster.setSmartCurrentLimit(20, 30);
        
        else
            intakeMaster.setSmartCurrentLimit(70);
         
    }

    public double getObjDistance(){
        //return filter.calculate(tof.getRangeInches());
        return 0;
    }

}
