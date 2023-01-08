package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
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

        intakeMaster.setSmartCurrentLimit(40);
        intakeSlave.setSmartCurrentLimit(40);

        intakeSlave.follow(intakeMaster);
    }

    public static Intake getInstance(){
        return instance;
    }

    public void setMotors(double val){
        intakeMaster.set(val);
    }
}
