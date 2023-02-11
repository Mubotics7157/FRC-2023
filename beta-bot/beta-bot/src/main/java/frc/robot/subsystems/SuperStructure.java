package frc.robot.subsystems;

import java.util.TreeMap;

public class SuperStructure {
    private Intake intake = Intake.getInstance();
    private Elevator elevator = Elevator.getInstance();
    private Wrist wrist = Wrist.getInstance();
    
    private SuperStructureState scoringState = SuperStructureState.STOWED;

    public enum SuperStructureState{
        CONE_HIGH,
        CUBE_HIGH,
        CONE_MID,
        CUBE_MID,
        CUBE_INTAKE,
        CONE_INTAKE,
        FALLEN_CONE,
        STOWED
    }

    
}
