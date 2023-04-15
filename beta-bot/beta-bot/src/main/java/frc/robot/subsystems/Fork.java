package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ForksConstants;

public class Fork extends SubsystemBase{

private WPI_TalonFX forkMotor;
private boolean initialDeploy;

private static Fork instance = new Fork();
    public Fork (){
        forkMotor = new WPI_TalonFX(ForksConstants.DEVICE_ID_FORKS);

        forkMotor.configFactoryDefault();
        forkMotor.setNeutralMode(NeutralMode.Brake);
        forkMotor.setInverted(false);
        forkMotor.config_kP(0, ForksConstants.FORK_KP);
        //forkMotor.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 20, 1));
        initialDeploy = false;

        forkMotor.setSelectedSensorPosition(0);

    }

    public static Fork getInstance(){
        return instance;
    }

    public void set(double speed){
        forkMotor.set(ControlMode.PercentOutput, speed);
    }

    public boolean setSetpoint(double setpoint){
        forkMotor.set(ControlMode.Position,setpoint);
        return Math.abs(setpoint-forkMotor.getSelectedSensorPosition()) < 30000;
    }

    public void setInitialDeployState(boolean state){
        initialDeploy = state;
    }


    public boolean getInitialDeployState(){
        return initialDeploy;
    }
}