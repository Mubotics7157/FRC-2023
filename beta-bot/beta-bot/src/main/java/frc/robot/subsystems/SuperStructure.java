package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Wrist.WristState;

public class SuperStructure extends SubsystemBase {
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

    @Override
    public void periodic() {
        switch(scoringState){
            case CONE_HIGH:
                break;
            case CONE_MID:
                break;
            case STOWED:
                stowAll();
                break;
            case CONE_INTAKE:
                break;
            case CUBE_INTAKE:
                break;
            case FALLEN_CONE:
                break;
        }
    }

    public void goToPosition(double elevatorSetpoint, Rotation2d wristSetpoint){
        elevator.setElevatorHeight(elevatorSetpoint);
        wrist.setSetpoint(wristSetpoint);

    }

    public void intakeCone(double elevatorSetpoint, Rotation2d wristSetpoint, int intakeSetpoint){
        elevator.setElevatorHeight(elevatorSetpoint);
        wrist.setSetpoint(wristSetpoint);

        if(wrist.atSetpoint())
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

    public void stowAll(){
        wrist.setWristState(WristState.STOW);
        elevator.setState(ElevatorState.STOW);
        intake.setIntakeState(IntakeState.OFF);

    }

    public void idleIntake(){
        intake.setIntakeState(IntakeState.IDLE);
    }

    public boolean atSetpoint(){
        return wrist.atSetpoint() && elevator.atSetpoint();
    }

    public SuperStructureState getState(){
        return scoringState;
    }

    public void setState(SuperStructureState state){
        scoringState = state;
    }
}
