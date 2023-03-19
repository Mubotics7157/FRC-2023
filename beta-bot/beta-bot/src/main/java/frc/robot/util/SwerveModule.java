package frc.robot.util;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
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
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AltConstants.DriveConstants;
import frc.robot.AltConstants.SwerveModuleConstants;

public class SwerveModule {
    private TalonFX m_driveMotor;
    private TalonFX m_steerMotor;
    private WPI_CANCoder m_cancoder;

    private StatusSignalValue<Double> m_drivePosition;
    private StatusSignalValue<Double> m_driveVelocity;
    private StatusSignalValue<Double> m_steerPosition;
    private StatusSignalValue<Double> m_steerVelocity;
    private BaseStatusSignalValue[] m_signals;
    private double m_driveRotationsPerMeter = 0;

    private PositionVoltage m_angleSetter = new PositionVoltage(0, true, 0, 0, true);
    private VelocityTorqueCurrentFOC m_velocitySetter = new VelocityTorqueCurrentFOC(0);

    private SwerveModulePosition m_internalState = new SwerveModulePosition();

    public SwerveModule(int drivePort,int turnPort,int encoderPort,double encoderOffset, boolean invertDrive) {
        m_driveMotor = new TalonFX(drivePort, "swerve");
        m_steerMotor = new TalonFX(turnPort, "swerve");

        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
        Slot0Configs driveConfigs = new Slot0Configs();
        driveConfigs.kP =3;
        talonConfigs.Slot0 = driveConfigs;
        talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        if(invertDrive)
            talonConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        else 
            talonConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        m_driveMotor.getConfigurator().apply(talonConfigs);
        m_driveMotor.setInverted(invertDrive);

        /* Undo changes for torqueCurrent */
        talonConfigs.TorqueCurrent = new TorqueCurrentConfigs();
        Slot0Configs turnConfig = new Slot0Configs();
        turnConfig.kP = 50;
        turnConfig.kD = 0.1;
        talonConfigs.Slot0 = turnConfig;
        // Modify configuration to use remote CANcoder fused
        //talonConfigs.Feedback.FeedbackRemoteSensorID = encoderPort;
        talonConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        //talonConfigs.Feedback.RotorToSensorRatio = SwerveModuleConstants.TURN_GEAR_RATIO;
        talonConfigs.Feedback.SensorToMechanismRatio = SwerveModuleConstants.TURN_GEAR_RATIO;
        talonConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        talonConfigs.ClosedLoopGeneral.ContinuousWrap = true;
        //talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //talonConfigs.ClosedLoopGeneral.ContinuousWrap = false; // Enable continuous wrap for swerve modules
        m_steerMotor.getConfigurator().apply(talonConfigs);

        m_cancoder = new WPI_CANCoder(encoderPort, SwerveModuleConstants.SWERVE_CANIVORE_ID);

        m_cancoder.configFactoryDefault();
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.magnetOffsetDegrees = encoderOffset;
        m_cancoder.configAllSettings(config,50);

        m_drivePosition = m_driveMotor.getPosition();
        m_driveVelocity = m_driveMotor.getVelocity();
        m_steerPosition = m_steerMotor.getPosition();
        m_steerVelocity = m_steerMotor.getVelocity();

        m_signals = new BaseStatusSignalValue[4];
        m_signals[0] = m_drivePosition;
        m_signals[1] = m_driveVelocity;
        m_signals[2] = m_steerPosition;
        m_signals[3] = m_steerVelocity;

        /* Calculate the ratio of drive motor rotation to meter on ground */
        double rotationsPerWheelRotation = 6.75;
        double metersPerWheelRotation = Math.PI * DriveConstants.WHEEL_DIAMETER_METERS;
        m_driveRotationsPerMeter = rotationsPerWheelRotation / metersPerWheelRotation;

        m_steerMotor.setRotorPosition(4000);
        OrangeUtility.sleep(2000);
    }

    public SwerveModulePosition getPosition() {
        /* Refresh all signals */
        m_drivePosition.refresh();
        m_driveVelocity.refresh();
        m_steerPosition.refresh();
        m_steerVelocity.refresh();

        /* Now latency-compensate our signals */
        double drive_rot =
                m_drivePosition.getValue();
                        //+ (m_driveVelocity.getValue() * m_drivePosition.getTimestamp().getLatency());
        double angle_rot =
                m_steerPosition.getValue();
                        //+ (m_steerVelocity.getValue() * m_steerPosition.getTimestamp().getLatency());

        /* And push them into a SwerveModuleState object to return */
        m_internalState.distanceMeters = drive_rot / m_driveRotationsPerMeter;
        /* Angle is already in terms of steer rotations */
        m_internalState.angle = Rotation2d.fromRotations(angle_rot);

        return m_internalState;
    }

    public void apply(SwerveModuleState state) {
        var optimized = CTREUtils.optimize(state, m_internalState.angle);

        double angleToSetDeg = optimized.angle.getRotations();
        //SmartDashboard.putNumber("angle to set", Units.rotationsToDegrees(angleToSetDeg));
        m_steerMotor.setControl(m_angleSetter.withPosition(angleToSetDeg));
        double velocityToSet = optimized.speedMetersPerSecond * m_driveRotationsPerMeter;
        m_driveMotor.setControl(m_velocitySetter.withVelocity(velocityToSet));
    }

    public BaseStatusSignalValue[] getSignals() {
        return m_signals;
    }

    public double getDriveVelocity(){
        m_driveVelocity.refresh();
        return (m_driveVelocity.getValue() / SwerveModuleConstants.DRIVE_GEAR_RATIO) * (Math.PI * DriveConstants.WHEEL_DIAMETER_METERS);
    }

    public void changeTurnKP(){
        Slot0Configs turnConfig = new Slot0Configs();
        turnConfig.kP = SmartDashboard.getNumber("Turn kP", 1);
        m_steerMotor.getConfigurator().refresh(turnConfig);
    }

    public void changeDriveKP(){
        Slot0Configs driveConfig = new Slot0Configs();
        driveConfig.kP = SmartDashboard.getNumber("Drive kP", 1);
        m_driveMotor.getConfigurator().refresh(driveConfig);
    }

    public Rotation2d getAbsHeading(){
        return Rotation2d.fromDegrees(m_cancoder.getAbsolutePosition());
    }

    public void reZeroTurnMotors(){
        
        m_steerMotor.setRotorPosition(4000);
    }
}
