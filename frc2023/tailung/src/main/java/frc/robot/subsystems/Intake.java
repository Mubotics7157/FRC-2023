package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.util.CommonConversions;

public class Intake extends SubsystemBase {

    public enum IntakeState {
        OFF,
        INTAKE_CUBE,
        OUTTAKE_CUBE_MID,
        OUTTAKE_CUBE_HIGH,
        OUTTAKE_CUBE_HYBRID,
        OUTTAKE_CUBE_HIGH_SHOOT,
        OUTTAKE_CUBE_MID_SHOOT,
        INTAKE_CONE,
        OUTTAKE_CONE,
        INTAKE_CONE_SEAGULL,
        INTAKE,
        OUTTAKE,
        IDLE,
        CONE_SNIPER,
        CUSTOM
    }

    private TalonFX intakeMaster;
    private TalonFX intakeSlave;

    private TalonFXConfiguration masterConfig;
    private TalonFXConfiguration slaveConfig;

    private DoubleSolenoid solenoid;

    private IntakeState intakeState;
    private Value commandedSolenoidState;


    private static Intake instance = new Intake();

    private InterpolatingTreeMap<Double, Double> distanceMap = new InterpolatingTreeMap<>();

    private double intakeAdj = 0;
    private double coneIntakeAdjustment = 0;

    public Intake() {
        solenoid = new DoubleSolenoid(IntakeConstants.DEVICE_ID_PCM, IntakeConstants.PNEUMATICS_MODULE_TYPE,
                IntakeConstants.DEVICE_ID_SOLENOID_FORWARD, IntakeConstants.DEVICE_ID_SOLENOID_REVERSE);
        commandedSolenoidState = Value.kForward;

        intakeState = IntakeState.OFF;

        intakeMaster = new TalonFX(IntakeConstants.DEVICE_ID_INTAKE_MASTER);
        intakeSlave = new TalonFX(IntakeConstants.DEVICE_ID_INTAKE_SLAVE);

        intakeMaster.configFactoryDefault();
        intakeSlave.configFactoryDefault();

        masterConfig = new TalonFXConfiguration();
        slaveConfig = new TalonFXConfiguration();

        masterConfig.slot0.kP = IntakeConstants.TOP_ROLLER_KP;
        masterConfig.slot0.kF = IntakeConstants.TOP_ROLLER_KF;

        slaveConfig.slot0.kP = IntakeConstants.TOP_ROLLER_KP;
        slaveConfig.slot0.kF = IntakeConstants.TOP_ROLLER_KF;

        intakeMaster.configAllSettings(masterConfig);
        intakeSlave.configAllSettings(slaveConfig);

        intakeMaster.setInverted(IntakeConstants.INVERT_MASTER);
        intakeSlave.setInverted(intakeMaster.getInverted());

        intakeMaster.setNeutralMode(NeutralMode.Brake);
        intakeSlave.setNeutralMode(NeutralMode.Brake);

        intakeMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 70, 70, 1));
        intakeSlave.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 70, 70, 1));

        intakeMaster.configVoltageCompSaturation(10);
        intakeSlave.configVoltageCompSaturation(10);

        intakeMaster.enableVoltageCompensation(true);
        intakeSlave.enableVoltageCompensation(true);

        SmartDashboard.putNumber("custom intake", -1000);
        SmartDashboard.putNumber("intake adjustment", 0);
        SmartDashboard.putNumber("cone intake adjustment", 0);
    }

    public static Intake getInstance() {
        return instance;
    }

    public void initMap() {
        distanceMap.put(0.0, 0.0);
    }

    @Override
    public void periodic() {
        intakeAdj = SmartDashboard.getNumber("intake adjustment", 0);
        coneIntakeAdjustment = SmartDashboard.getNumber("cone intake adjustment", 0);

        SmartDashboard.putString("intake state", intakeState.toString());

        switch (intakeState) {
            case OFF:
                setMotors(0);
                break;
            case INTAKE_CUBE:
                setSpeed(IntakeConstants.CUBE_INTAKE_SPEED);
                toggleIntake(false);
                break;
            case OUTTAKE_CUBE_MID:
                setSpeed(IntakeConstants.CUBE_OUTTAKE_MID);
                break;
            case OUTTAKE_CUBE_HIGH:
                if(DriverStation.isTeleop())
                    setSpeed(IntakeConstants.CUBE_OUTTAKE_HIGH);
                else
                    setSpeed(-700);
                break;
            case OUTTAKE_CUBE_HYBRID:
                setSpeed(IntakeConstants.CUBE_OUTTAKE_HYBRID);
                break;
            case OUTTAKE_CUBE_MID_SHOOT:
                setSpeed(IntakeConstants.CUBE_OUTTAKE_MID_SHOOT);
                break;
            case OUTTAKE_CUBE_HIGH_SHOOT:
                if (DriverStation.isAutonomous())
                    setSpeed(IntakeConstants.CUBE_OUTTAKE_HIGH_SHOOT - 150);
                else
                    setSpeed(IntakeConstants.CUBE_OUTTAKE_HIGH_SHOOT);
                break;
            case INTAKE_CONE:
                setSpeed(IntakeConstants.CONE_INTAKE_SPEED + coneIntakeAdjustment);
                toggleIntake(true);
                break;
            case OUTTAKE_CONE:
                if (DriverStation.isTeleop())
                    setSpeed(.35 * -3000);
                else
                    setSpeed(.4 * -3000);
                break;
            case INTAKE_CONE_SEAGULL:
                setSpeed(.375 * 5700);
                break;
            case INTAKE:
                setSpeed(IntakeConstants.CONE_INTAKE_SPEED);
                break;
            case OUTTAKE:
                if (DriverStation.isTeleop())
                    setSpeed(.35 * -IntakeConstants.CONE_INTAKE_SPEED);
                else
                    setSpeed(-IntakeConstants.CONE_INTAKE_SPEED);

                break;
            case IDLE:
                setSpeed(IntakeConstants.IDLE_SPEED);
                break;
            case CONE_SNIPER:
            if(DriverStation.isTeleop())
                setSpeed(IntakeConstants.CONE_SNIPER_SPEED);
            else
                setSpeed(IntakeConstants.CONE_SNIPER_SPEED + 500);

                break;
            case CUSTOM:
                setSpeed(SmartDashboard.getNumber("custom intake", -1000));
                break;
        }

    }

    public void setIntakeState(IntakeState state) {
        if (state != IntakeState.IDLE)
            currentLimit(false);
        else
            currentLimit(true);

        intakeState = state;
    }

    public void setMotors(double speed) {
        intakeMaster.set(ControlMode.PercentOutput, speed);
        intakeSlave.set(ControlMode.PercentOutput, speed);
    }

    private void setSpeed(double speedRPM) {
        speedRPM += intakeAdj;
        intakeMaster.set(ControlMode.Velocity, CommonConversions.RPMToStepsPerDecisec(speedRPM));
        intakeSlave.set(ControlMode.Velocity, CommonConversions.RPMToStepsPerDecisec(speedRPM));
    }

    public void toggleIntake(boolean forward) {

        if (forward)
            commandedSolenoidState = Value.kForward;
        else
            commandedSolenoidState = Value.kReverse;

        solenoid.set(commandedSolenoidState);
    }

    public void toggleJaws(){
        if(solenoid.get() == Value.kForward){
            solenoid.set(Value.kReverse);
        }
        else{
            solenoid.set(Value.kForward);
        }
    }

    public void closeJaws() {
        solenoid.set(Value.kForward);
    }

    public void openJaws() {
        solenoid.set(Value.kReverse);
    }

    public boolean isClosed() {
        return commandedSolenoidState == Value.kForward;
    }

    public IntakeState getState() {
        return intakeState;
    }

    public void adjustmentReset(){
        SmartDashboard.putNumber("intake adjustment", 0);
        SmartDashboard.putNumber("cone intake adjustment", 0);
        intakeAdj = 0;
        coneIntakeAdjustment = 0;
    }

    public void currentLimit(boolean enable) {
        if (enable) {
            intakeMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 40, 1));
            intakeSlave.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 20, 40, 1));
        }

        else {
            intakeMaster.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 70, 80, 1));
            intakeSlave.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, 70, 80, 1));
        }

    }

    public double getObjDistance() {
        // return filter.calculate(tof.getRangeInches());
        return 0;
    }

}
