package frc.robot.util;

import com.ctre.phoenixpro.BaseStatusSignalValue;
import com.ctre.phoenixpro.StatusCode;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.TalonFXConfiguration;
import com.ctre.phoenixpro.configs.TorqueCurrentConfigs;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class ProSwerveModule {
    private TalonFX driveMotor;
    private TalonFX steerMotor;
    private CANcoder cancoder;

    private StatusSignalValue<Double> drivePosition;
    private StatusSignalValue<Double> driveVelocity;
    private StatusSignalValue<Double> steerPosition;
    private StatusSignalValue<Double> steerVelocity;
    private BaseStatusSignalValue[] signals;
    private double driveRotationsPerMeter = 0;

    private PositionVoltage angleSetter = new PositionVoltage(0, true, 0, 0, true);
    private VelocityTorqueCurrentFOC velocitySetter = new VelocityTorqueCurrentFOC(0);
    private StatusCode initializationStatus = StatusCode.StatusCodeNotInitialized;

    private SwerveModulePosition internalState = new SwerveModulePosition();

    public ProSwerveModule(int drivePort,int turnPort,int encoderPort,double encoderOffset, boolean invertDrive) {
        driveMotor = new TalonFX(drivePort, SwerveModuleConstants.SWERVE_CANIVORE_ID);
        steerMotor = new TalonFX(turnPort,SwerveModuleConstants.SWERVE_CANIVORE_ID);

        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
        Slot0Configs driveConfigs = new Slot0Configs();
        driveConfigs.kP =3;
        talonConfigs.Slot0 = driveConfigs;
        talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        if(invertDrive)
            talonConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        else 
            talonConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        for(int i=0;i<5;i++){
            initializationStatus = driveMotor.getConfigurator().apply(talonConfigs);
            if(initializationStatus.isOK())
                break;
            else if(!initializationStatus.isOK())
                System.out.println("Failed to Configure CAN ID" + drivePort);
            
        }
        driveMotor.setInverted(invertDrive);

        talonConfigs.TorqueCurrent = new TorqueCurrentConfigs();
        Slot0Configs turnConfig = new Slot0Configs();
        turnConfig.kP = 50;
        turnConfig.kD = 0.1;
        talonConfigs.Slot0 = turnConfig;
        talonConfigs.Feedback.FeedbackRemoteSensorID = encoderPort;
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        talonConfigs.Feedback.RotorToSensorRatio = SwerveModuleConstants.TURN_GEAR_RATIO;
        talonConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        talonConfigs.ClosedLoopGeneral.ContinuousWrap = true;
        for(int i=0;i<5;i++){
            initializationStatus = steerMotor.getConfigurator().apply(talonConfigs);
            if(initializationStatus.isOK())
                break;
            else if(!initializationStatus.isOK())
                System.out.println("Failed to Configure CAN ID" + turnPort);
            
        }

        cancoder = new CANcoder(encoderPort, SwerveModuleConstants.SWERVE_CANIVORE_ID);

        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = encoderOffset / 360;
        for(int i=0;i<5;i++){
            initializationStatus = cancoder.getConfigurator().apply(config);
            if(initializationStatus.isOK())
                break;
            else if(!initializationStatus.isOK())
                System.out.println("Failed to Configure CAN ID" + encoderPort);
            
        }

        drivePosition = driveMotor.getPosition();
        driveVelocity = driveMotor.getVelocity();
        steerPosition = cancoder.getPosition();
        steerVelocity = cancoder.getVelocity();

        signals = new BaseStatusSignalValue[4];
        signals[0] = drivePosition;
        signals[1] = driveVelocity;
        signals[2] = steerPosition;
        signals[3] = steerVelocity;

        double rotationsPerWheelRotation = 6.75;
        double metersPerWheelRotation = Math.PI * DriveConstants.WHEEL_DIAMETER_METERS;
        driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;
    }

    public SwerveModulePosition getPosition() {
        drivePosition.refresh();
        driveVelocity.refresh();
        steerPosition.refresh();
        steerVelocity.refresh();

        //TODO: implement latency comp
        double drive_rot =
                drivePosition.getValue();
        double angle_rot =
                steerPosition.getValue();

        internalState.distanceMeters = drive_rot / driveRotationsPerMeter;
        internalState.angle = Rotation2d.fromRotations(angle_rot);

        return internalState;
    }

    public void setState(SwerveModuleState state) {
        var optimized = CTREUtils.optimize(state, internalState.angle);

        double angleToSetDeg = optimized.angle.getRotations();
        steerMotor.setControl(angleSetter.withPosition(angleToSetDeg));
        double velocityToSet = optimized.speedMetersPerSecond * driveRotationsPerMeter;
        driveMotor.setControl(velocitySetter.withVelocity(velocityToSet));
    }

    public BaseStatusSignalValue[] getSignals() {
        return signals;
    }

    public double getDriveVelocity(){
        driveVelocity.refresh();
        return (driveVelocity.getValue() / SwerveModuleConstants.DRIVE_GEAR_RATIO) * (Math.PI * DriveConstants.WHEEL_DIAMETER_METERS);
    }

    public void changeTurnKP(){
        Slot0Configs turnConfig = new Slot0Configs();
        turnConfig.kP = SmartDashboard.getNumber("Turn kP", 1);
        steerMotor.getConfigurator().refresh(turnConfig);
    }

    public void changeDriveKP(){
        Slot0Configs driveConfig = new Slot0Configs();
        driveConfig.kP = SmartDashboard.getNumber("Drive kP", 1);
        driveMotor.getConfigurator().refresh(driveConfig);
    }

    public Rotation2d getAbsHeading(){
        return Rotation2d.fromDegrees(cancoder.getAbsolutePosition().getValue());
    }

    public Rotation2d getHeading(){
        return Rotation2d.fromRotations(steerMotor.getRotorPosition().getValue());
    }
}
