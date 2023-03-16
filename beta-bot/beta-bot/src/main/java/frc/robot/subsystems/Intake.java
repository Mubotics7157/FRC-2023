package frc.robot.subsystems;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.util.CommonConversions;

public class Intake extends SubsystemBase {

    public enum IntakeState{
        OFF,
        INTAKE_CUBE,
        OUTTAKE_CUBE_MID,
        OUTTAKE_CUBE_HIGH,
        OUTTAKE_CUBE_HYBRID,
        OUTTAKE_CUBE_HIGH_SHOOT,
        OUTTAKE_CUBE_MID_SHOOT,
        INTAKE_CONE,
        OUTTAKE_CONE,
        INTAKE_CONE_SEAGUL,
        INTAKE,
        OUTTAKE,
        IDLE,
        CONE_SNIPER,
        CUSTOM
    }

    private TalonFX intakeMaster; 
    private TalonFX intakeSlave;

    private TalonFXConfiguration masterConfig;
    private TalonFXConfiguration slaveConfig;

    private DoubleSolenoid solenoid; 

    private IntakeState intakeState;

    //private Ultrasonic tof;


    private static Intake instance = new Intake();

    private InterpolatingTreeMap<Double,Double> distanceMap = new InterpolatingTreeMap<>();

    public Intake(){
        solenoid = new DoubleSolenoid(IntakeConstants.DEVICE_ID_PCM, IntakeConstants.PNEUMATICS_MODULE_TYPE, IntakeConstants.DEVICE_ID_SOLENOID_FORWARD, IntakeConstants.DEVICE_ID_SOLENOID_REVERSE);

        intakeState = IntakeState.OFF;

        intakeMaster = new TalonFX(IntakeConstants.DEVICE_ID_INTAKE_MASTER);
        intakeSlave = new TalonFX(IntakeConstants.DEVICE_ID_INTAKE_SLAVE);

        intakeMaster.configFactoryDefault();
        intakeSlave.configFactoryDefault();

        masterConfig.slot0.kP = IntakeConstants.TOP_ROLLER_KP;
        masterConfig.slot0.kF = IntakeConstants.TOP_ROLLER_KF;

        slaveConfig.slot0.kP = IntakeConstants.TOP_ROLLER_KP;
        slaveConfig.slot0.kF = IntakeConstants.TOP_ROLLER_KF;

        intakeMaster.configAllSettings(masterConfig);
        intakeSlave.configAllSettings(slaveConfig);

        intakeMaster.setInverted(IntakeConstants.INVERT_MASTER);
        intakeSlave.setInverted(!intakeMaster.getInverted());
       
        intakeMaster.setNeutralMode(NeutralMode.Brake);
        intakeSlave.setNeutralMode(NeutralMode.Brake);

        intakeMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 70, 70, 1)); 
        intakeSlave.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 70, 70, 1));

        intakeMaster.configVoltageCompSaturation(10);
        intakeSlave.configVoltageCompSaturation(10);

       intakeMaster.enableVoltageCompensation(true);
       intakeSlave.enableVoltageCompensation(true);


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


        
        SmartDashboard.putString("intake state", intakeState.toString());
        
        switch(intakeState){
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
            case OUTTAKE_CUBE_HYBRID:
                setSpeed(IntakeConstants.CUBE_OUTTAKE_HYBRID);
                break;
            case OUTTAKE_CUBE_MID_SHOOT:
                setSpeed(IntakeConstants.CUBE_OUTTAKE_MID_SHOOT);
                //toggleIntake(false);
                break;
            case OUTTAKE_CUBE_HIGH_SHOOT:
                if(DriverStation.isAutonomous())
                    setSpeed(IntakeConstants.CUBE_OUTTAKE_HIGH_SHOOT-150);
                else
                    setSpeed(IntakeConstants.CUBE_OUTTAKE_HIGH_SHOOT);
                break;
            case INTAKE_CONE:
                setSpeed(IntakeConstants.CONE_INTAKE_SPEED);
                //setSpeed(2000);
                toggleIntake(true);
                break;
            case OUTTAKE_CONE:
                if(DriverStation.isTeleop())
                    setSpeed(.35*-3000);
                else
                    setSpeed(.4*-3000);
                //toggleIntake(true);
                break;
            case INTAKE_CONE_SEAGUL:
                setSpeed(.375 * 5700);
                break;
            case INTAKE:
                setSpeed(IntakeConstants.CONE_INTAKE_SPEED);
                break;
            case OUTTAKE:
                if(DriverStation.isTeleop())
                    setSpeed(.35*-IntakeConstants.CONE_INTAKE_SPEED);
                else
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
        intakeMaster.set(ControlMode.PercentOutput,speed);
        intakeSlave.set(ControlMode.PercentOutput,speed);
    }

    private void setSpeed(double speedRPM){
        intakeMaster.set(ControlMode.Velocity, CommonConversions.RPMToStepsPerDecisec(speedRPM));
        intakeSlave.set(ControlMode.Velocity, CommonConversions.RPMToStepsPerDecisec(speedRPM));

        //topController.setReference(0, ControlType.kVelocity);
    }

    public void toggleIntake(boolean forward){
        
        if(forward)
            solenoid.set(Value.kForward);
        else
            solenoid.set(Value.kReverse);
        
    }

    public void closeJaws(){
        solenoid.set(Value.kForward);
    }

    public void openJaws(){
        solenoid.set(Value.kReverse);
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
        if(enable){
            intakeMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 40, 1)); 
            intakeSlave.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 40, 1));
        }
        
        else{
            intakeMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 70, 80, 1)); 
            intakeSlave.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 70, 80, 1));
        }
         
    }

    public double getObjDistance(){
        //return filter.calculate(tof.getRangeInches());
        return 0;
    }

}
