package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Elevator {

    private WPI_TalonFX elevatorMotor;
    private static Elevator instance = new Elevator();
    
    public Elevator(){

        elevatorMotor = new WPI_TalonFX(0);

        elevatorMotor.configPeakOutputForward(.25);
        elevatorMotor.configPeakOutputReverse(-.25);

        elevatorMotor.setInverted(true);

        elevatorMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 10, 20, 1));
        elevatorMotor.setNeutralMode(NeutralMode.Brake);
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


}
