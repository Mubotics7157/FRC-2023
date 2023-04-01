package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
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
    private LED led = LED.getInstance();
    //private LED led;
    private boolean scoreHigh;
    private boolean idleIntake = true;
    
    private SuperStructureState scoringState = SuperStructureState.STOWED;
    private ScoringPosition scoringPosition = ScoringPosition.HIGH;

    private Rotation2d wristAdj;
    private double elevAdj;
    
    public enum SuperStructureState{
        CONE_HIGH,
        CONE_MID,
        CUBE_HIGH,
        CUBE_MID_SHOOT,
        CUBE_HIGH_SHOOT,
        CUBE_MID,
        CUBE_HYBRID,
        CUBE_INTAKE,
        CONE_INTAKE,
        FALLEN_CONE,
        PORTAL,
        OPEN_DOOR,
        STOWED,
        SEAGUL,
        CUSTOM,
        CONE_SNIPER,
        ZERO,
        CLIMB
    }

    public enum ScoringPosition{
        HIGH,
        MID,
        HYBRID
    }

    public SuperStructure(){
        SmartDashboard.putNumber("custom wrist adjustment", 0);
        SmartDashboard.putNumber("custom elevator adjustment", 0);
        //led = LED.getInstance();

    }

    @Override
    public void periodic() {
        //SmartDashboard.putBoolean("robot at setpoint", atSetpoint());
        //SmartDashboard.putString("SuperStructure state", scoringState.toString());
        //SmartDashboard.putString("Scoring Position", scoringPosition.toString());

        if(idleIntake && Intake.getInstance().getState() != IntakeState.IDLE)
            intake.setIntakeState(IntakeState.IDLE);

    }

    public static SuperStructure getInstance(){
        return instance;
    }
    public void goToPosition(double elevatorSetpoint, Rotation2d wristSetpoint){
        wrist.setSetpoint(wristSetpoint);
        elevator.setElevatorHeight(elevatorSetpoint);
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

    public void intake(double elevatorSetpoint, Rotation2d wristSetpoint, IntakeState intakeState){
        elevator.setElevatorHeight(elevatorSetpoint);
        wrist.setSetpoint(wristSetpoint);
        intake.setIntakeState(intakeState);
    }

    public void shoot(){
        idleIntake = false;
        
        if(scoringState == SuperStructureState.CONE_HIGH || scoringState == SuperStructureState.CONE_MID)
            intake.setIntakeState(IntakeState.OUTTAKE_CONE);

        else if(scoringState == SuperStructureState.CUBE_MID)
            intake.setIntakeState(IntakeState.OUTTAKE_CUBE_MID);

        else if(scoringState == SuperStructureState.CUBE_HIGH)
            intake.setIntakeState(IntakeState.OUTTAKE_CUBE_HIGH);

        else if(scoringState == SuperStructureState.CONE_SNIPER)
            intake.setIntakeState(IntakeState.CONE_SNIPER);

        else if(scoringState == SuperStructureState.CUBE_HIGH_SHOOT)
            intake.setIntakeState(IntakeState.OUTTAKE_CUBE_HIGH_SHOOT);

        else if(scoringState == SuperStructureState.CUBE_MID_SHOOT)
            intake.setIntakeState(IntakeState.OUTTAKE_CUBE_MID_SHOOT);

        else if(scoringState == SuperStructureState.CUBE_HYBRID)
            intake.setIntakeState(IntakeState.OUTTAKE_CUBE_HYBRID);
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
    wristAdj = Rotation2d.fromDegrees(-6).plus(Rotation2d.fromDegrees(SmartDashboard.getNumber("custom wrist adjustment", 0)));
    elevAdj = SmartDashboard.getNumber("custom elevator adjustment", 0);

    if(state==SuperStructureState.CUBE_INTAKE || state==SuperStructureState.CONE_INTAKE|| state ==SuperStructureState.FALLEN_CONE)
        idleIntake = false;
    else if((state==SuperStructureState.CONE_HIGH || state==SuperStructureState.CONE_MID || scoringState == SuperStructureState.FALLEN_CONE || scoringState == SuperStructureState.CONE_INTAKE || scoringState == SuperStructureState.PORTAL || scoringState == SuperStructureState.SEAGUL || state == SuperStructureState.CONE_SNIPER) || !atSetpoint())
        idleIntake = true;
    else
        idleIntake = false;
    
    
    
        scoringState = state;
        setLedMode(scoringState);

        switch(scoringState){
            case CONE_HIGH:
                goToPosition(SuperStructureConstants.ELEVATOR_CONE_HIGH + elevAdj, SuperStructureConstants.WRIST_CONE_HIGH.plus(wristAdj));
                Drive.getInstance().changeSlow();
                break;
            case CONE_MID:
                goToPosition(SuperStructureConstants.ELEVATOR_CONE_MID + elevAdj, SuperStructureConstants.WRIST_CONE_MID.plus(wristAdj));
                Drive.getInstance().changeSlow();
                break;
            case CUBE_HIGH:
                goToPosition(SuperStructureConstants.ELEVATOR_CUBE_HIGH + elevAdj, SuperStructureConstants.WRIST_CUBE_HIGH.plus(wristAdj));
                Drive.getInstance().changeSlow();
                break;
            case CUBE_MID:
                goToPosition(SuperStructureConstants.ELEVATOR_CUBE_MID + elevAdj, SuperStructureConstants.WRIST_CUBE_MID.plus(wristAdj));
                Drive.getInstance().changeSlow();
                break;
            case CUBE_HIGH_SHOOT:
                goToPosition(SuperStructureConstants.ELEVATOR_CUBE_SHOOT + elevAdj, SuperStructureConstants.WRIST_CUBE_SHOOT.plus(wristAdj));
                Drive.getInstance().changeSlow();
                break;
            case CUBE_MID_SHOOT:
                goToPosition(SuperStructureConstants.ELEVATOR_CUBE_SHOOT + elevAdj, SuperStructureConstants.WRIST_CUBE_SHOOT.plus(wristAdj));
                Drive.getInstance().changeSlow();
                break;
            case CUBE_HYBRID:
                goToPosition(SuperStructureConstants.ELEVATOR_CUBE_HYBRID + elevAdj, SuperStructureConstants.WRIST_CUBE_HYRBID.plus(wristAdj));
                Drive.getInstance().changeMax();
                break;
            case STOWED:
                Drive.getInstance().changeMax();
                stowAll();
                break;
            case CONE_INTAKE:
                Drive.getInstance().changeMax();
                intakeCone(SuperStructureConstants.ELEVATOR_INTAKE_CONE_UPRIGHT + elevAdj, SuperStructureConstants.WRIST_INTAKE_CONE_UPRIGHT.plus(wristAdj));
                break;
            case CUBE_INTAKE:
                Drive.getInstance().changeMax();    
                intakeCube(SuperStructureConstants.ELEVATOR_INTAKE_CUBE + elevAdj, SuperStructureConstants.WRIST_INTAKE_CUBE.plus(wristAdj));
                break;
            case FALLEN_CONE:
                Drive.getInstance().changeMax();
                intakeCone(SuperStructureConstants.ELEVATOR_INTAKE_CONE_FALLEN + elevAdj, SuperStructureConstants.WRIST_INTAKE_CONE_FALLEN.plus(wristAdj));//SuperStructureConstants.WRIST_INTAKE_CONE_FALLEN);
                break;
            case OPEN_DOOR:
                goToPosition(SuperStructureConstants.ELEVATOR_INTAKE_CONE_FALLEN + elevAdj, SuperStructureConstants.WRIST_INTAKE_CONE_FALLEN.plus(wristAdj));
                break;
            case CUSTOM:
                goToPosition(SmartDashboard.getNumber("custom elevator", 0), Rotation2d.fromDegrees(SmartDashboard.getNumber("custom wrist", -55)));
                Drive.getInstance().changeSlow();
                break;
            case SEAGUL:
                //goToPosition(0, Rotation2d.fromDegrees(-20));
                intake(SuperStructureConstants.ELEVATOR_INTAKE_SEAGUL + elevAdj, SuperStructureConstants.WRIST_INTAKE_SEAGUL.plus(wristAdj), IntakeState.INTAKE_CONE_SEAGUL);
                Drive.getInstance().changeSlow();
                break;
            case PORTAL:
                intake(SuperStructureConstants.ELEVATOR_INTAKE_PORTAL, SuperStructureConstants.WRIST_INTAKE_PORTAL.plus(wristAdj), IntakeState.INTAKE_CONE);
                Drive.getInstance().changeVerySlow();
                break;
            case ZERO:
                zeroAll();
                break;
            case CONE_SNIPER:
                Drive.getInstance().changeMax();
                goToPosition(SuperStructureConstants.ELEVATOR_CONE_SNIPER + elevAdj, SuperStructureConstants.WRIST_CONE_SNIPER.plus(wristAdj));
                break;
            case CLIMB:
                Intake.getInstance().closeJaws();
                goToPosition(0, Rotation2d.fromDegrees(0));
                break;
            default:
                break;
        }

    }

    public void emergencySetpointReset(){
        SmartDashboard.putNumber("custom wrist adjustment", 0);
        SmartDashboard.putNumber("custom elevator adjustment", 0);
    }

    private void setLedMode(SuperStructureState state){
        if(led.getLastError() != ErrorCode.FirmVersionCouldNotBeRetrieved){

        switch(state){
            case CONE_HIGH:
                led.setGreenStrobe();
                break;
            case CONE_MID:
                led.setGreenStrobe();
                break;
            case CUBE_HIGH:
                led.setGreenStrobe();
                break;
            case CUBE_MID:
                led.setGreenStrobe();
                break;
            case CUBE_MID_SHOOT:
                led.setGreenStrobe();
                break;
            case CUBE_HYBRID:
                led.setGreenStrobe();
                break;
            case CUBE_INTAKE:
                led.setPurple();
                break;
            case CONE_INTAKE:
                led.setYellow();
                break;
            case FALLEN_CONE:
                led.setYellow();
                break;
            case STOWED:
                led.setCurrentIntake();
                break;
            case SEAGUL:
                led.setYellowStrobe();
                break;
            case CONE_SNIPER:
                led.setRedStrobe();
                break;
            case CLIMB:
                led.setRainbow();
                break;
            default:
                led.setYellow();
                break;
        }
    }
    }

    public void setScorePosition(ScoringPosition position){
        scoringPosition = position;
    }

    public ScoringPosition getScoringPosition(){
        return scoringPosition;
    }

    public void setIdleIntake(boolean idle){
        idleIntake = idle;

    }

    public void enableIdling(){
        idleIntake = true;
    }
}
