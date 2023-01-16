package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    //9 10   
    private WPI_TalonFX intakeMaster; 
    private WPI_TalonFX intakeSlave;
    private CANSparkMax intakeAngle;
    private SparkMaxPIDController intakeController;
    private RelativeEncoder intakeEncoder;
    private static Intake instance = new Intake();
    private SparkMaxPIDController jawController;
    private RelativeEncoder jawEncoder;
    private Rotation2d intakeAngleSetpoint;


    public enum ObjectType{
        CUBE,
        CONE,
        SIDEWAYS_CONE
    }

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

        intakeAngle.setSmartCurrentLimit(20);

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

        jawController = intakeAngle.getPIDController();
        jawEncoder = intakeAngle.getEncoder();
        jawEncoder.setPositionConversionFactor(2*Math.PI/20);

        jawController.setP(0);
        
        
    }

    public static Intake getInstance(){
        return instance;
    }

    public void intakeGamePiece(ObjectType type){
       
        switch(type){
            case CONE:
            break;

            case CUBE:
            break;

            case SIDEWAYS_CONE:

            break;

            default:
            break;

        }

    }

    public void setAngle(Rotation2d angle){
        intakeAngleSetpoint = angle;
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

    private void jogIntakeAngle(double val){
        intakeAngle.set(val);
    }

    public void runIntake(double val){
        intakeMaster.set(ControlMode.PercentOutput,val);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("intake encoder", jawEncoder.getPosition());
        jawController.setReference(intakeAngleSetpoint.getRadians(), ControlType.kPosition);
    }

}
