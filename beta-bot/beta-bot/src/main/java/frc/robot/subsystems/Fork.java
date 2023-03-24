package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ForksConstants;

public class Fork extends SubsystemBase{
    
WPI_TalonFX forkMotor;

private static Fork instance = new Fork();
    public Fork (){
        forkMotor = new WPI_TalonFX(ForksConstants.DEVICE_ID_FORKS);

        forkMotor.configFactoryDefault();
        forkMotor.setNeutralMode(NeutralMode.Brake);
        forkMotor.setInverted(false);
        
    }

    public static Fork getInstance(){
        return instance;
    }

    public void set(double speed){
        forkMotor.set(ControlMode.PercentOutput, speed);
    }
}
