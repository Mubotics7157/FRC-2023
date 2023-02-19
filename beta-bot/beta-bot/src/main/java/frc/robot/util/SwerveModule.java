package frc.robot.util;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveModuleConstants;

public class SwerveModule {
   private WPI_TalonFX turnMotor;
   private WPI_TalonFX driveMotor;
   private WPI_CANCoder absEncoder;

    public SwerveModule(int drivePort, int turnPort, int encoderPort, double angleOffset, boolean isInverted){
        driveMotor = new WPI_TalonFX(drivePort, SwerveModuleConstants.SWERVE_CANIVORE_ID);
        turnMotor = new WPI_TalonFX(turnPort, SwerveModuleConstants.SWERVE_CANIVORE_ID);

        turnMotor.configFactoryDefault();
        driveMotor.configFactoryDefault();


        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveMotor.configAllSettings(driveConfig);
        driveMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,SwerveModuleConstants.TIMEOUT_MS);
        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.setInverted(isInverted);
        driveMotor.configVoltageCompSaturation(12);
        driveMotor.enableVoltageCompensation(true);
        driveMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 15, 0));
        driveMotor.setSensorPhase(false);
        driveMotor.config_kP(0, SwerveModuleConstants.driveKP);
        
        turnMotor.setSelectedSensorPosition(0);
        turnMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,0,SwerveModuleConstants.TIMEOUT_MS);
        turnMotor.setNeutralMode(NeutralMode.Brake);
        turnMotor.setInverted(true);
        turnMotor.config_kP(0, .15);
        turnMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 15, 15, 0));

        absEncoder = new WPI_CANCoder(encoderPort, SwerveModuleConstants.SWERVE_CANIVORE_ID);
    

        absEncoder.configFactoryDefault();
        CANCoderConfiguration config = new CANCoderConfiguration();
        config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.magnetOffsetDegrees = angleOffset;
        absEncoder.configAllSettings(config,50);

        OrangeUtility.sleep(1000);
        System.out.println(getAbsHeading());
        turnMotor.setSelectedSensorPosition(getAbsHeading().getDegrees()/(360/(2048*SwerveModuleConstants.TURN_GEAR_RATIO)));
    }

    public void setState(SwerveModuleState state){
        SwerveModuleState optimizedState = CTREUtils.optimize(state, getHeading());

        setVelocity(optimizedState.speedMetersPerSecond,.2); 
        setTurnRad(optimizedState.angle);
    }

    private void setVelocity(double driveSetpoint, double dt){
        double driveFFVolts = SwerveModuleConstants.DRIVE_FEEDFORWARD.calculate(driveSetpoint);

        if(driveSetpoint==0){
            driveMotor.set(ControlMode.PercentOutput, 0);
        } 
        else 
            driveMotor.set(ControlMode.Velocity, CommonConversions.metersPerSecToStepsPerDecisec(driveSetpoint, DriveConstants.WHEEL_DIAMETER_METERS),DemandType.ArbitraryFeedForward,driveFFVolts/12);
    }


    private void setTurnRad(Rotation2d turnSetpointRad){
        turnMotor.set(ControlMode.Position, turnSetpointRad.getDegrees()/(360/(2048*SwerveModuleConstants.TURN_GEAR_RATIO)));
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), getHeading()); 
    }

    private Rotation2d getAbsHeading(){
        return Rotation2d.fromDegrees(absEncoder.getAbsolutePosition());
    }

    public Rotation2d getHeading(){
        return Rotation2d.fromDegrees(turnMotor.getSelectedSensorPosition()*(360/(2048*SwerveModuleConstants.TURN_GEAR_RATIO)));
    }

    public double getDriveVelocity(){
        return CommonConversions.stepsPerDecisecToMetersPerSec(driveMotor.getSelectedSensorVelocity());
    }

    public double getPosition(){
        return CommonConversions.stepsToMeters(driveMotor.getSelectedSensorPosition());
    }

    public Rotation2d getRelativeHeading(){
        return Rotation2d.fromDegrees(turnMotor.getSelectedSensorPosition()*(360/(2048*SwerveModuleConstants.TURN_GEAR_RATIO)));
    }

    public void flip(double angle){
        absEncoder.configMagnetOffset(absEncoder.configGetMagnetOffset()+angle);
        
    }
    public void overrideMotors(){
        driveMotor.set(ControlMode.PercentOutput,0);
        turnMotor.set(ControlMode.PercentOutput,0);
    }

}