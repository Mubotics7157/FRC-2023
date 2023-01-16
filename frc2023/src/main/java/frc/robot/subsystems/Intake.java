package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    //9 10   
    private WPI_TalonFX intakeMaster; 
    private WPI_TalonFX intakeSlave;
    private CANSparkMax intakeAngle;
    private SparkMaxPIDController intakeController;
    private RelativeEncoder intakeEncoder;
    private static Intake instance = new Intake();

    public Intake(){
        
        
        intakeMaster = new WPI_TalonFX(19);
        intakeSlave = new WPI_TalonFX(20);
        intakeAngle = new CANSparkMax(14, MotorType.kBrushless);

        intakeController = intakeAngle.getPIDController();
        intakeEncoder = intakeAngle.getEncoder();

        intakeMaster.configFactoryDefault();
        intakeSlave.configFactoryDefault();
        intakeAngle.restoreFactoryDefaults();

        intakeController.setP(0);
        intakeController.setFF(0);

        intakeController.setOutputRange(-0.5, 0.5);

        intakeEncoder.setPosition(0);
        //intakeAngle.setSoftLimit(SoftLimitDirection.kForward, 5000);
        //intakeAngle.setSoftLimit(SoftLimitDirection.kReverse, 0);


        //intakeMaster.setSmartCurrentLimit(20);
        //intakeSlave.setSmartCurrentLimit(20);

        intakeMaster.setInverted(true);
        intakeSlave.setInverted(false);
        intakeAngle.setInverted(false);

        intakeMaster.setNeutralMode(NeutralMode.Brake);
        intakeSlave.setNeutralMode(NeutralMode.Brake);
        intakeAngle.setIdleMode(IdleMode.kBrake);

        //TODO: ask harshal abt further clarification for regular intake current limit

        //intakeSlave.follow(intakeMaster);
        
        
    }

    public static Intake getInstance(){
        return instance;
    }

    public void setMotors(String objectType, double val, double angleVal){
       

        if(objectType == "cones"){
            intakeMaster.set(val);
            intakeAngle.set(val);
            //intakeAngle.set(angleVal);
        }
        else if(objectType == "cubes"){
            intakeMaster.set(val);
            intakeSlave.set(val);
            //intakeAngle.set(angleVal);
        }
        else{
            intakeMaster.set(val);
            intakeSlave.set(val);
            //intakeAngle.set(angleVal);
        }
    }

    public void setAngle(double value){
        intakeAngle.set(value);
    }

    public void currentLimit(boolean enable){
        if(enable){
            intakeMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 2, 10, .25));
            intakeSlave.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 2, 10, .25));
            intakeAngle.setSmartCurrentLimit(2, 20);
        }
        else{
            intakeMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 2, 10, .25));
            intakeSlave .configStatorCurrentLimit(new StatorCurrentLimitConfiguration(false, 2, 10, .25));
            intakeAngle.setSmartCurrentLimit(40);
        }
    }

    public double getIntakeAngle(){
        return intakeEncoder.getPosition() / 20;
    }
}
