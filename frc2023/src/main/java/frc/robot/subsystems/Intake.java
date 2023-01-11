package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    //9 10   
    private CANSparkMax intakeMaster; 
    private CANSparkMax intakeSlave;
    
    private static Intake instance = new Intake();

    public Intake(){
        
        
        intakeMaster = new CANSparkMax(9, MotorType.kBrushless);
        intakeSlave = new CANSparkMax(10, MotorType.kBrushless);

        intakeSlave.restoreFactoryDefaults();
        intakeMaster.restoreFactoryDefaults();


        //intakeMaster.setSmartCurrentLimit(20);
        //intakeSlave.setSmartCurrentLimit(20);

        intakeMaster.setInverted(false);
        intakeSlave.setInverted(false);

        intakeMaster.setIdleMode(IdleMode.kBrake);
        intakeSlave.setIdleMode(IdleMode.kBrake);


        //intakeSlave.follow(intakeMaster);
        
    }

    public static Intake getInstance(){
        return instance;
    }

    public void setMotors(double val){
        intakeMaster.set(val);
        intakeSlave.set(val);
    }
}
