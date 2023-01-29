package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase{
    private CANSparkMax leftMaster;
    private CANSparkMax leftSlave;
    private CANSparkMax rightMaster;
    private CANSparkMax rightSlave;

    private static Drive instance = new Drive();

    public static Drive getInstance(){return instance;}

    public Drive(){
        leftMaster = new CANSparkMax(0, MotorType.kBrushless);
        leftSlave = new CANSparkMax(0, MotorType.kBrushless);
        rightMaster = new CANSparkMax(0, MotorType.kBrushless);
        rightSlave = new CANSparkMax(0, MotorType.kBrushless);

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
    }

    public void drive(double l, double r){
        leftMaster.set(l);
        rightMaster.set(r);
    }

}