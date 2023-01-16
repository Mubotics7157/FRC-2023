package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    //9 10   
    private WPI_TalonFX intakeMaster; 
    private WPI_TalonFX intakeSlave;
    private CANSparkMax intakeBot;
    private static Intake instance = new Intake();

    public Intake(){
        
        
        intakeMaster = new WPI_TalonFX(19);
        intakeSlave = new WPI_TalonFX(20);
        intakeBot = new CANSparkMax(14, MotorType.kBrushless);

        intakeMaster.configFactoryDefault();
        intakeSlave.configFactoryDefault();
        intakeBot.restoreFactoryDefaults();


        //intakeMaster.setSmartCurrentLimit(20);
        //intakeSlave.setSmartCurrentLimit(20);

        intakeMaster.setInverted(true);
        intakeSlave.setInverted(false);
        intakeBot.setInverted(false);

        intakeMaster.setNeutralMode(NeutralMode.Brake);
        intakeSlave.setNeutralMode(NeutralMode.Brake);
        intakeBot.setIdleMode(IdleMode.kBrake);

        //TODO: ask harshal abt further clarification for regular intake current limit

        //intakeSlave.follow(intakeMaster);
        
        
    }

    public static Intake getInstance(){
        return instance;
    }

    public void setMotors(String objectType, double val){
       

        if(objectType == "cones"){
            intakeMaster.set(val);
            intakeBot.set(val);
        }
        else if(objectType == "cubes"){
            intakeMaster.set(val);
            intakeSlave.set(val);
        }
        else{
            intakeMaster.set(val);
            intakeSlave.set(val);
            intakeBot.set(val);
        }
    }

    public void currentLimit(boolean enable){
        if(enable){
            intakeMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 2, 10, .25));
            intakeSlave.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 2, 10, .25));
            intakeBot.setSmartCurrentLimit(2, 20);
        }
        else{
            intakeMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 2, 10, .25));
            intakeSlave .configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 2, 10, .25));
            intakeBot.setSmartCurrentLimit(40);
        }
    }
}
