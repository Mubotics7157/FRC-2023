package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {
    private CANSparkMax leftMaster;
    private CANSparkMax leftSlave;
    private CANSparkMax rightMaster;
    private CANSparkMax rightSlave;

    private Pigeon2 gyro;

    public Drive(){
        leftMaster = new CANSparkMax(DriveConstants.DEVICE_ID_LEFT_MASTER, MotorType.kBrushless);
        leftSlave = new CANSparkMax(DriveConstants.DEVICE_ID_LEFT_SLAVE, MotorType.kBrushless);
        rightMaster = new CANSparkMax(DriveConstants.DEVICE_ID_RIGHT_MASTER, MotorType.kBrushless);
        rightSlave = new CANSparkMax(DriveConstants.DEVICE_ID_RIGHT_SLAVE, MotorType.kBrushless);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);

        leftMaster.setInverted(true);
        leftSlave.setInverted(leftMaster.getInverted());

        rightMaster.setInverted(false);
        rightSlave.setInverted(rightMaster.getInverted());

        gyro = new Pigeon2(DriveConstants.DEVICE_ID_GYRO);
    }

    public void setMotors(double left, double right ){
        leftMaster.set(left);
        rightMaster.set(right);
    }

    public void tankDrive(){
        double leftOutput = Math.abs(RobotContainer.m_driverController.getLeftY()) > .2 ? RobotContainer.m_driverController.getLeftY() :0;
        double rightOutput = Math.abs(RobotContainer.m_driverController.getRightY()) > .2 ? RobotContainer.m_driverController.getRightY() :0;
        leftMaster.set(leftOutput);
        rightMaster.set(rightOutput);
    }


    
}
