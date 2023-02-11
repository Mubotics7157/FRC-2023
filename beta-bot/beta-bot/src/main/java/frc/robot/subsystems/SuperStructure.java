package frc.robot.subsystems;

import java.util.HashMap;
import java.util.TreeMap;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake.IntakeState;

public class SuperStructure extends CommandBase {
    private Intake intake = Intake.getInstance();
    private Elevator elevator = Elevator.getInstance();
    private Wrist wrist = Wrist.getInstance();
    
    private SuperStructureState scoringState = SuperStructureState.STOWED;
    private SetpointState setpointState = SetpointState.OFF;

    public enum SuperStructureState{
        CONE_HIGH,
        CUBE_HIGH,
        CONE_MID,
        CUBE_MID,
        CUBE_INTAKE,
        CONE_INTAKE,
        FALLEN_CONE,
        STOWED,
        ZERO
    }

    public enum SetpointState{
        AT_GOAL,
        MOVING,
        OFF
    }

    public void goToPosition(double elevatorSetpoint, Rotation2d wristSetpoint){
        elevator.setElevatorHeight(elevatorSetpoint);
        wrist.setSetpoint(wristSetpoint);

    }

    public void intakeCone(double elevatorSetpoint, Rotation2d wristSetpoint, int intakeSetpoint){
        elevator.setElevatorHeight(elevatorSetpoint);
        wrist.setSetpoint(wristSetpoint);
        intake.setIntakeState(IntakeState.INTAKE_CONE);

    }

    public void intakeCube(double elevatorSetpoint, Rotation2d wristSetpoint, int intakeSetpoint){
        elevator.setElevatorHeight(elevatorSetpoint);
        wrist.setSetpoint(wristSetpoint);
        intake.setIntakeState(IntakeState.INTAKE_CUBE);

    }

    public void shoot(int intakeSetpoint, boolean cone){
        if(cone)
            intake.setIntakeState(IntakeState.OUTTAKE_CONE);
        else
            intake.setIntakeState(IntakeState.OUTTAKE_CUBE);
    }





    
}
