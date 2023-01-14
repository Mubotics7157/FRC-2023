package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.util.CommonConversions;


public class Elevator extends SubsystemBase {

    private WPI_TalonFX elevatorMotor;
    private TalonFXConfiguration config;
    private static Elevator instance = new Elevator();
    private boolean runPID = false;

    private double setpoint = 0;
    
    public Elevator(){
        //encoder.setDistancePerRotation();
        config = new TalonFXConfiguration();
        elevatorMotor = new WPI_TalonFX(ElevatorConstants.DEVICE_ID_ELEVATOR);

        config.slot0.kP = ElevatorConstants.ELEVATOR_KP; 
        config.slot0.kD = 0;
        config.slot0.kF = 0;
        config.motionCruiseVelocity =  CommonConversions.radPerSecToStepsPerDecisec(2*Math.PI,6);
        config.motionAcceleration = CommonConversions.radPerSecSquaredToStepsPerDecisecSquared(Math.PI,6);

        elevatorMotor.configAllSettings(config);

        elevatorMotor.configPeakOutputForward(WristConstants.WRIST_PEAK_OUTPUT_FORWARD);
        elevatorMotor.configPeakOutputReverse(ElevatorConstants.WRIST_PEAK_OUTPUT_REVERSE);

        elevatorMotor.setInverted(true);

        //elevatorMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10, 20, 1));
        elevatorMotor.setNeutralMode(NeutralMode.Brake);
        elevatorMotor.setSelectedSensorPosition(0);

        elevatorMotor.configReverseSoftLimitThreshold(0);
        elevatorMotor.configReverseSoftLimitEnable(true);

        SmartDashboard.putNumber("elevator setpoint", 0);
    }

    @Override
    public void periodic() {
        if(runPID)
            elevatorMotor.set(ControlMode.Position,setpoint);
            //elevatorMotor.set(ControlMode.Position,CommonConversions.inchesToSteps(setpoint, 1.625, 6));
    
        SmartDashboard.putNumber("elevator onboard encoder", elevatorMotor.getSelectedSensorPosition());
    }

    public static Elevator getInstance(){
        return instance;
    }

    public void setOutput(double val){
        elevatorMotor.set(ControlMode.PercentOutput, val);
    }


    public void setSetpoint(double heightIn){
        setpoint = heightIn;
        //setpoint = CommonConversions.inchesToSteps(heightIn, 1.625, 6);
    }

    public void setGains(double kP){
        elevatorMotor.config_kP(0, kP);
    }

    public void setPositionHold(boolean hold){
        runPID = hold;
    }

    public double getHeight(){
        return elevatorMotor.getSelectedSensorPosition();
    }

    public void setSetpoint(){

    }

    /*
    public double getElevatorHeightIn(){

    }
    */

    
}
