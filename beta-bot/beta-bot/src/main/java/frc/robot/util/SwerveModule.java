package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.ctre.phoenixpro.configs.CurrentLimitsConfigs;
import com.ctre.phoenixpro.configs.MotorOutputConfigs;
import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.controls.NeutralOut;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.TorqueCurrentFOC;
import com.ctre.phoenixpro.controls.VelocityDutyCycle;
import com.ctre.phoenixpro.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenixpro.controls.VelocityVoltage;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule {
   private TalonFX driveMotor;
   private TalonFX turnMotor;
   private WPI_CANCoder absEncoder;
   private CurrentLimitsConfigs currentLimitsConfigs;

    public SwerveModule(int drivePort, int turnPort, int encoderPort, double angleOffset, boolean isInverted){
        driveMotor = new TalonFX(drivePort, SwerveModuleConstants.SWERVE_CANIVORE_ID);
        turnMotor = new TalonFX(turnPort, SwerveModuleConstants.SWERVE_CANIVORE_ID);

        driveMotor.getConfigurator().apply(new com.ctre.phoenixpro.configs.TalonFXConfiguration());
        turnMotor.getConfigurator().apply(new com.ctre.phoenixpro.configs.TalonFXConfiguration());
    
        //turnMotor.getConfigurator().apply(new com.ctre.phoenixpro.configs.TalonFXConfiguration());
        //driveMotor.getConfigurator().apply(new com.ctre.phoenixpro.configs.TalonFXConfiguration());

        currentLimitsConfigs = new CurrentLimitsConfigs();
        currentLimitsConfigs.StatorCurrentLimit = 15;
        currentLimitsConfigs.StatorCurrentLimitEnable = true;
    
        var driveConfig = new com.ctre.phoenixpro.configs.TalonFXConfiguration();
        driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        driveConfig.Feedback.SensorToMechanismRatio = -1;
        
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0.kP = SwerveModuleConstants.driveKP;
        //driveConfig.Slot0.kS = SwerveModuleConstants.driveKS;
        //driveConfig.Slot0.kV = CommonConversions.metersPerSecToRotationsPerSec(SwerveModuleConstants.driveKV, DriveConstants.WHEEL_DIAMETER_METERS, SwerveModuleConstants.DRIVE_GEAR_RATIO);
        //driveConfig.Slot0.kD = CommonConversions.metersPerSecToRotationsPerSec(SwerveModuleConstants.driveKA, DriveConstants.WHEEL_DIAMETER_METERS, SwerveModuleConstants.DRIVE_GEAR_RATIO);
        driveMotor.getConfigurator().apply(driveConfig);
        driveMotor.setInverted(isInverted);
        //driveMotor.getConfigurator().apply(currentLimitsConfigs);
        //driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 15, 0));
        
        var turnConfig = new com.ctre.phoenixpro.configs.TalonFXConfiguration();
        turnConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.Slot0.kP = 1.5;
        turnConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turnMotor.getConfigurator().apply(turnConfig);
        turnMotor.setRotorPosition(0);
        //turnMotor.setInverted(true);
        //turnMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 15, 0));

        absEncoder = new WPI_CANCoder(encoderPort, SwerveModuleConstants.SWERVE_CANIVORE_ID);
    

        absEncoder.configFactoryDefault();
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.magnetOffsetDegrees = angleOffset;
        absEncoder.configAllSettings(config,50);

        OrangeUtility.sleep(2000);
        System.out.println(getAbsHeading());
        turnMotor.setRotorPosition(getAbsHeading().getDegrees()/(360/(SwerveModuleConstants.TURN_GEAR_RATIO)));
       
    }

    public void setState(SwerveModuleState state){
        SwerveModuleState optimizedState = CTREUtils.optimize(state, getHeading());

        setVelocity(optimizedState.speedMetersPerSecond,.2); 
        setTurnRad(optimizedState.angle);
    }

    private void setVelocity(double driveSetpoint, double dt){
        double driveFFVolts = SwerveModuleConstants.DRIVE_FEEDFORWARD.calculate(driveSetpoint);
        //assuming the gains are remeasured for the proper units it will be used! :P
        SmartDashboard.putNumber("Drive setpoint", CommonConversions.metersPerSecToRotationsPerSec(
            driveSetpoint, DriveConstants.WHEEL_DIAMETER_METERS, SwerveModuleConstants.DRIVE_GEAR_RATIO));
        if(driveSetpoint==0){
            driveMotor.set(0);
        } 
        else 
            driveMotor.setControl(new VelocityVoltage(
                CommonConversions.metersPerSecToRotationsPerSec(
                    driveSetpoint, DriveConstants.WHEEL_DIAMETER_METERS, SwerveModuleConstants.DRIVE_GEAR_RATIO),
                    false, driveFFVolts, 0, false));
            //velocity, enable foc, FF, slot index, override neutral mode
            //driveMotor.setControl(CommonConversions.metersPerSecToStepsPerDecisec(driveSetpoint, DriveConstants.WHEEL_DIAMETER_METERS),true,DemandType.ArbitraryFeedForward,driveFFVolts/12);
            //driveMotor.set(ControlMode.Velocity, CommonConversions.metersPerSecToStepsPerDecisec(driveSetpoint, DriveConstants.WHEEL_DIAMETER_METERS),DemandType.ArbitraryFeedForward,driveFFVolts/12);
    }


    private void setTurnRad(Rotation2d turnSetpointRad){
        //turnMotor.setRotorPosition(turnSetpointRad.getDegrees()/(360/(2048*SwerveModuleConstants.TURN_GEAR_RATIO)));
        var request = new PositionVoltage(0).withSlot(0);
        turnMotor.setControl(request.withPosition(turnSetpointRad.getDegrees()/(360/(SwerveModuleConstants.TURN_GEAR_RATIO))));
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), getHeading()); 
    }

    public Rotation2d getAbsHeading(){
        return Rotation2d.fromDegrees(absEncoder.getAbsolutePosition());
    }

    public Rotation2d getHeading(){
        return Rotation2d.fromDegrees(turnMotor.getRotorPosition().getValue()*(360/(SwerveModuleConstants.TURN_GEAR_RATIO)));
    }

    public double getDriveVelocity(){
        //getRotorVelocity().getValue() or getVelocity()?
        return CommonConversions.rotationsPersecToMetersPerSec(driveMotor.getVelocity().getValue(), DriveConstants.WHEEL_DIAMETER_METERS, SwerveModuleConstants.DRIVE_GEAR_RATIO);
    }

    public double getPosition(){
        return CommonConversions.rotationsToMeters(driveMotor.getRotorPosition().getValue(), DriveConstants.WHEEL_DIAMETER_METERS, SwerveModuleConstants.DRIVE_GEAR_RATIO);
    }

    public Rotation2d getRelativeHeading(){
        return Rotation2d.fromDegrees(turnMotor.getRotorPosition().getValue()*(360/SwerveModuleConstants.TURN_GEAR_RATIO));
    }

    public void flip(double angle){
        absEncoder.configMagnetOffset(absEncoder.configGetMagnetOffset()+angle);
    }
    public void overrideMotors(){
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public double getCurrentDraw(){
        return driveMotor.getSupplyCurrent().getValue();
    }

    public void changeTurnKP(){
        var slotConfig = new Slot0Configs();
        slotConfig.kP = SmartDashboard.getNumber("Turn kP", .3/5);
        turnMotor.getConfigurator().apply(slotConfig);
    }

}
