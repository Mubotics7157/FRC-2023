package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.CommonConversions;


public class Elevator extends SubsystemBase {

    private WPI_TalonFX elevatorMotor;
    private TalonFXConfiguration config;
    private static Elevator instance = new Elevator();

    private double setpoint = 0;
    
    public Elevator(){

        config = new TalonFXConfiguration();
        elevatorMotor = new WPI_TalonFX(29);

        config.slot0.kP = 0;
        config.slot0.kD = 0;
        config.slot0.kF = 0;
        config.motionCruiseVelocity =  CommonConversions.radPerSecToStepsPerDecisec(2*Math.PI,6);
        config.motionAcceleration = CommonConversions.radPerSecSquaredToStepsPerDecisecSquared(Math.PI,6);

        elevatorMotor.configAllSettings(config);

        elevatorMotor.configPeakOutputForward(.75);
        elevatorMotor.configPeakOutputReverse(-.75);

        elevatorMotor.setInverted(true);

        //elevatorMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10, 20, 1));
        elevatorMotor.setNeutralMode(NeutralMode.Brake);
    }

    @Override
    public void periodic() {
        //elevatorMotor.set(ControlMode.Position,setpoint);
    }

    public static Elevator getInstance(){
        return instance;
    }

    public void setOutput(double val){
        elevatorMotor.set(ControlMode.PercentOutput, val);
    }

    public void idle(){
        elevatorMotor.set(ControlMode.PercentOutput,.05);
    }

    public void setSetpoint(double heightIn){
        setpoint = heightIn;
    }

}
