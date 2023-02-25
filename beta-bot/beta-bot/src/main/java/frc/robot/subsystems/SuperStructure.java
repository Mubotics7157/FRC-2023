package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

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
    //private LED led = LED.getInstance();
    private LED led;
    
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
        SEAGUL,
        CUSTOM,
        CONE_SNIPER,
        ZERO
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("robot at setpoint", atSetpoint());
        SmartDashboard.putString("SuperStructure state", scoringState.toString());

        if(scoringState!=SuperStructureState.FALLEN_CONE && scoringState!=SuperStructureState.CONE_INTAKE && scoringState!=SuperStructureState.CUBE_INTAKE && !atSetpoint() && intake.isClosed() && scoringState!=SuperStructureState.SEAGUL)
            intake.setIntakeState(IntakeState.IDLE);

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

        intake.setIntakeState(IntakeState.INTAKE_CONE);

    }

    public void intakeCube(double elevatorSetpoint, Rotation2d wristSetpoint){
        elevator.setElevatorHeight(elevatorSetpoint);
        wrist.setSetpoint(wristSetpoint);
        intake.setIntakeState(IntakeState.INTAKE_CUBE);

    }

    public void shoot(){
        if(scoringState == SuperStructureState.CONE_HIGH || scoringState == SuperStructureState.CONE_MID)
            intake.setIntakeState(IntakeState.OUTTAKE_CONE);
        else if(scoringState == SuperStructureState.CUBE_MID)
            intake.setIntakeState(IntakeState.OUTTAKE_CUBE_MID);
        else if(scoringState == SuperStructureState.CUBE_HIGH)
            intake.setIntakeState(IntakeState.OUTTAKE_CUBE_HIGH);
        else if(scoringState == SuperStructureState.CONE_SNIPER)
            intake.setIntakeState(IntakeState.CONE_SNIPER);
        else
            intake.setIntakeState(IntakeState.CUSTOM);
    }

    public void stowAll(){
        elevator.setState(ElevatorState.STOW);
        intake.setIntakeState(IntakeState.OFF);
        wrist.setWristState(WristState.STOW);
    }

    public void zeroAll(){
        //elevator.setState(ElevatorState.ZERO);
        wrist.setWristState(WristState.ZERO);
        //intake.setIntakeState(IntakeState.OFF);
    }

    public boolean isZeroed(){
        return wrist.isZeroed();
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

        //setLedMode(scoringState);
        
        
        switch(scoringState){
            case CONE_HIGH:
                goToPosition(SuperStructureConstants.ELEVATOR_CONE_HIGH, SuperStructureConstants.WRIST_CONE_HIGH);
                Drive.getInstance().changeSlow();
                break;
            case CONE_MID:
                goToPosition(SuperStructureConstants.ELEVATOR_CONE_MID, SuperStructureConstants.WRIST_CONE_MID);
                Drive.getInstance().changeSlow();
                break;
            case CUBE_HIGH:
                goToPosition(SuperStructureConstants.ELEVATOR_CUBE_HIGH, SuperStructureConstants.WRIST_CUBE_HIGH);
                Drive.getInstance().changeSlow();
                break;
            case STOWED:
                Drive.getInstance().changeMax();
                stowAll();
                break;
            case CONE_INTAKE:
                intakeCone(SuperStructureConstants.ELEVATOR_INTAKE_CONE_UPRIGHT, SuperStructureConstants.WRIST_INTAKE_CONE_UPRIGHT);
                break;
            case CUBE_INTAKE:
                intakeCube(SuperStructureConstants.ELEVATOR_INTAKE_CONE_FALLEN, SuperStructureConstants.WRIST_INTAKE_CONE_FALLEN);
                break;
            case FALLEN_CONE:
                intakeCone(SuperStructureConstants.ELEVATOR_INTAKE_CONE_FALLEN, SuperStructureConstants.WRIST_INTAKE_CONE_FALLEN);//SuperStructureConstants.WRIST_INTAKE_CONE_FALLEN);
                break;
            case OPEN_DOOR:
                goToPosition(SuperStructureConstants.ELEVATOR_INTAKE_CONE_FALLEN, SuperStructureConstants.WRIST_INTAKE_CONE_FALLEN);
                break;
            case CUSTOM:
                goToPosition(SmartDashboard.getNumber("custom elevator", 0), Rotation2d.fromDegrees(SmartDashboard.getNumber("custom wrist", -55)));
                Drive.getInstance().changeSlow();
                break;
            case SEAGUL:
                //goToPosition(0, Rotation2d.fromDegrees(-20));
                intakeCone(0, Rotation2d.fromDegrees(-20));
                Drive.getInstance().changeSlow();
                break;
            case ZERO:
                zeroAll();
                break;
            case CONE_SNIPER:
                goToPosition(0, Rotation2d.fromDegrees(-50));
            default:
                break;
        }

    }

    private void setLedMode(SuperStructureState state){
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
    }
}
