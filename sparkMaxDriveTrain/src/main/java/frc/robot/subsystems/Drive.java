package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {
    private CANSparkMax leftMaster;
    private CANSparkMax leftSlave;
    private CANSparkMax rightMaster;
    private CANSparkMax rightSlave;

    public Drive(){
        leftMaster = new CANSparkMax(DriveConstants.DEVICE_ID_LEFT_MASTER, MotorType.kBrushless);
        leftSlave = new CANSparkMax(DriveConstants.DEVICE_ID_LEFT_SLAVE, MotorType.kBrushless);
        rightMaster = new CANSparkMax(DriveConstants.DEVICE_ID_RIGHT_MASTER, MotorType.kBrushless);
        rightSlave = new CANSparkMax(DriveConstants.DEVICE_ID_RIGHT_SLAVE, MotorType.kBrushless);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
    }

    public void setMotors(double left, double right ){
        leftMaster.set(left);
        rightMaster.set(right);
    }

    
}
