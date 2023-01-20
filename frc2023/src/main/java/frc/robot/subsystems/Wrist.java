package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.WristConstants;

public class Wrist extends SubsystemBase {
    private CANSparkMax wristMaster;
    private CANSparkMax wristSlave;
    private SparkMaxPIDController wristController;
    private RelativeEncoder wristEncoder;

    private static Wrist instance = new Wrist();

    private double setpoint;



    public Wrist(){
        wristMaster = new CANSparkMax(WristConstants.DEVICE_ID_WRIST_MASTER, MotorType.kBrushless);
        wristSlave = new CANSparkMax(WristConstants.DEVICE_ID_WRIST_SLAVE, MotorType.kBrushless);

        wristMaster.setIdleMode(IdleMode.kBrake);
        wristSlave.setIdleMode(IdleMode.kBrake);

        wristEncoder = wristMaster.getEncoder();
        wristController = wristMaster.getPIDController();

        wristSlave.follow(wristMaster);

        setpoint = 0;
    }

    @Override
    public void periodic() {
        logData();
    }

    public static Wrist getInstance(){
        return instance;
    }

    public void setPercentOutput(double val){
        wristMaster.set(val);
    }

    public void setSetpoint(double setpoint){
        this.setpoint = setpoint;
    }

    public void goToPosition(){
        wristController.setReference(setpoint, ControlType.kPosition);

    }

    private double getPosition(){
        return wristEncoder.getPosition();
    }

    private void logData(){
        SmartDashboard.putNumber("wrist position", getPosition());
    }


}
