package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    //9 10   
    private WPI_TalonFX intakeMaster; 
    private WPI_TalonFX intakeSlave;
    
    private static Intake instance = new Intake();

    public Intake(){
        
        
        intakeMaster = new WPI_TalonFX(9);
        intakeSlave = new WPI_TalonFX(10);

        intakeMaster.configFactoryDefault();
        intakeSlave.configFactoryDefault();


        //intakeMaster.setSmartCurrentLimit(20);
        //intakeSlave.setSmartCurrentLimit(20);

        intakeMaster.setInverted(false);
        intakeSlave.setInverted(false);

        intakeMaster.setNeutralMode(NeutralMode.Brake);
        intakeSlave.setNeutralMode(NeutralMode.Brake);

        //TODO: ask harshal abt further clarification for regular intake current limit

        //intakeSlave.follow(intakeMaster);
        
    }

    public static Intake getInstance(){
        return instance;
    }

    public void setMotors(double val){
        intakeMaster.set(val);
        intakeSlave.set(val);
    }

    public void currentLimit(boolean enable){
        if(enable)
            intakeMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 2, 10, .25));
        else
            intakeMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 2, 10, .25));
    }
}
