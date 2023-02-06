package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Wrist.WristState;

public class ZeroWrist extends CommandBase{
    private Wrist wrist;
    private boolean hasZeroed;

    public ZeroWrist(Wrist instance){
        wrist = instance;
        hasZeroed = false;

        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        wrist.setWristState(WristState.ZERO);
        hasZeroed = wrist.zeroRoutine();
    }

    @Override
    public boolean isFinished() {
        return wrist.zeroRoutine();
    }

    @Override
    public void end(boolean interrupted) {
        wrist.setWristState(WristState.SETPOINT);
    }
}
