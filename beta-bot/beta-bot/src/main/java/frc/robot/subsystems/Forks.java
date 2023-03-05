package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Forks extends SubsystemBase {
    private WPI_TalonFX forks;

    public Forks(){
        forks = new WPI_TalonFX(35);
        forks.configFactoryDefault();
        forks.configPeakOutputForward(.5);
        forks.configPeakOutputReverse(-.5);
        forks.setNeutralMode(NeutralMode.Brake);
        //forks.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 30, 40, 1));
    }

    public void setOutput(double output){
        forks.set(output);
    }
    
}
