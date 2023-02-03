package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
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
        leftMaster = new CANSparkMax(40, MotorType.kBrushless);
        leftSlave = new CANSparkMax(41, MotorType.kBrushless);
        rightMaster = new CANSparkMax(43, MotorType.kBrushless);
        rightSlave = new CANSparkMax(42, MotorType.kBrushless);

        rightMaster.setInverted(true);
        rightSlave.setInverted(rightMaster.getInverted());

        leftMaster.setInverted(false);
        leftSlave.setInverted(leftMaster.getInverted());

        rightMaster.setIdleMode(IdleMode.kBrake);
        rightSlave.setIdleMode(rightMaster.getIdleMode());
        leftMaster.setIdleMode(rightMaster.getIdleMode());
        leftSlave.setIdleMode(rightMaster.getIdleMode());

        leftSlave.follow(leftMaster);
        rightSlave.follow(rightMaster);
    }

    public void drive(double l, double r){
        if(Math.abs(l) < 0.1)
            l = 0;
        if(Math.abs(r) < 0.1)
            r = 0;
            
        leftMaster.set(l);
        rightMaster.set(r);
    }

}