package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SuperStructureConstants;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Wrist.WristState;

public class SuperStructure extends SubsystemBase {

    private static SuperStructure instance = new SuperStructure();
    private Intake intake = Intake.getInstance();
    private Elevator elevator = Elevator.getInstance();
    private Wrist wrist = Wrist.getInstance();
    private LED led = LED.getInstance();
    
    private SuperStructureState scoringState = SuperStructureState.STOWED;

    public enum SuperStructureState{
        CONE_HIGH,
        CONE_MID,
        CUBE_HIGH,
        CUBE_MID,
        CUBE_INTAKE,
        CONE_INTAKE,
        FALLEN_CONE,
        OPEN_DOOR,
        STOWED,
        ZERO
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("robot at setpoint", atSetpoint());
        SmartDashboard.putString("SuperStructure state", scoringState.toString());
        //if(scoringState!=SuperStructureState.STOWED && !atSetpoint())
            //idleIntake();
    }

    public static SuperStructure getInstance(){
        return instance;
    }
    public void goToPosition(double elevatorSetpoint, Rotation2d wristSetpoint){
        elevator.setElevatorHeight(elevatorSetpoint);
        wrist.setSetpoint(wristSetpoint);

    }

    public void intakeCone(double elevatorSetpoint, Rotation2d wristSetpoint){
        elevator.setElevatorHeight(elevatorSetpoint);
        wrist.setSetpoint(wristSetpoint);

        //intake.setIntakeState(IntakeState.INTAKE_CONE);

    }

    public void intakeCube(double elevatorSetpoint, Rotation2d wristSetpoint){
        elevator.setElevatorHeight(elevatorSetpoint);
        wrist.setSetpoint(wristSetpoint);
        intake.setIntakeState(IntakeState.INTAKE_CUBE);

    }

    public void shoot(boolean cone){
        if(cone)
            intake.setIntakeState(IntakeState.OUTTAKE_CONE);
        else
            intake.setIntakeState(IntakeState.OUTTAKE_CUBE);
    }

    public void stowAll(){
        wrist.setWristState(WristState.STOW);
        elevator.setState(ElevatorState.STOW);
        //intake.setIntakeState(IntakeState.OFF);
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

        switch(state){
            case CONE_HIGH:
                led.setStrobe();
                break;
            case CUBE_INTAKE:
                led.setPurpleStrobe();
                break;
            case CONE_INTAKE:
                led.setYellowStrobe();
                break;
            case FALLEN_CONE:
                led.setYellowStrobe();
                break;
            case STOWED:
                led.setCurrentIntake();
                break;
            default:
                led.setStrobe();
                break;
        }

                switch(scoringState){
            case CONE_HIGH:
                goToPosition(SuperStructureConstants.ELEVATOR_CONE_HIGH, SuperStructureConstants.WRIST_CONE_HIGH);
                break;
            case CONE_MID:
                goToPosition(SuperStructureConstants.ELEVATOR_CONE_MID, SuperStructureConstants.WRIST_CONE_MID);
                break;
            case STOWED:
                stowAll();
                led.setCurrentIntake();
                break;
            case CONE_INTAKE:
                intakeCone(SuperStructureConstants.ELEVATOR_INTAKE_CONE_FALLEN, SuperStructureConstants.WRIST_INTAKE_CONE_UPRIGHT);
                break;
            case CUBE_INTAKE:
                intakeCube(SuperStructureConstants.ELEVATOR_INTAKE_CONE_FALLEN, SuperStructureConstants.WRIST_INTAKE_CONE_FALLEN);
                break;
            case FALLEN_CONE:
                intakeCone(SuperStructureConstants.ELEVATOR_INTAKE_CONE_FALLEN, Rotation2d.fromDegrees(-93));//SuperStructureConstants.WRIST_INTAKE_CONE_FALLEN);
                led.setYellowStrobe();
                break;
            case OPEN_DOOR:
                goToPosition(SuperStructureConstants.ELEVATOR_INTAKE_CONE_FALLEN, SuperStructureConstants.WRIST_INTAKE_CONE_FALLEN);
                break;
            default:
                break;
        }

    }
}
