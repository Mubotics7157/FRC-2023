package frc.robot.util;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class CommonConversions {
  
  public static double stepsToMeters(double steps){
    return steps*((DriveConstants.WHEEL_DIAMETER_METERS * Math.PI) / (2048*Constants.SwerveModuleConstants.DRIVE_GEAR_RATIO));
  }

   public static double stepsToInches(double steps,double wheelDiameter,double gearRatio){
    //1.625
    return steps*((wheelDiameter* Math.PI) / (2048*gearRatio));
  }

   public static double inchesToSteps(double inches,double wheelDiameter,double gearRatio){
    return (inches/(wheelDiameter*Math.PI)) * 2048*Constants.SwerveModuleConstants.DRIVE_GEAR_RATIO;
  }

  /**
   * Converts from encoder units per 100 milliseconds to meters per second.
   * @param stepsPerDecisec steps per decisecond
   * @return meters per second
   */
  public static double stepsPerDecisecToMetersPerSec(double stepsPerDecisec) {
    return stepsToMeters(stepsPerDecisec * 10);
  }

    /**
   * Converts from meters to encoder units.
   * @param meters meters
   * @return encoder units
   */
  public static double metersToSteps(double meters,double wheelDiameter) {
    return (meters / (wheelDiameter*Math.PI) ) *2048*Constants.SwerveModuleConstants.DRIVE_GEAR_RATIO;
  }

    /**
   * Converts from meters per second to encoder units per 100 milliseconds.
   * @param metersPerSec meters per second
   * @return encoder units per decisecond
   */
  public static double metersPerSecToStepsPerDecisec(double metersPerSec,double wheelDiameter) {
    return metersToSteps(metersPerSec,wheelDiameter) * .1d;
  }

  public static double RPMToStepsPerDecisec(double velRPM){
     return velRPM * 2048/600;
  }

  public static double stepsPerDecisecToRPM(double nativeVel){
    return nativeVel/2048*600;
  }


  public static double radiansToSteps(double rad,double gearing){
    double radPerStep = (((2*Math.PI)/gearing)) / 2048;
    return rad/radPerStep;
  }

  public static double stepsToRadians(double steps,double gearing){
    double radPerStep = 2*Math.PI / (gearing * 2048);
    return steps*radPerStep;
  }

  public static double radPerSecToStepsPerDecisec(double radPerSec,double gearing){
    return radiansToSteps(radPerSec,gearing) *.1;
  }

  public static double radPerSecSquaredToStepsPerDecisecSquared(double radPerSec,double gearing){
    return radiansToSteps(radPerSec,gearing) *.01;
  }

  public static double stepsToDegrees(double steps,double gearing){
    double degPerStep = 360 / (gearing * 2048);
    return steps*degPerStep;
  }

  public static double degreeToSteps(double steps,double gearing){
    double degPerStep = 360 / (gearing * 2048);
    return steps/degPerStep;
  }

  public static double metersPerSecToRotationsPerSec(double metersPerSec, double wheelDiameter, double gearing){
    //double stepsPerDecisec = metersPerSecToStepsPerDecisec(steps, gearing);
    //based off of mps = (rps / gearing) * circum
    double RPS = (metersPerSec / (wheelDiameter * Math.PI)) * gearing;
    return RPS;
  }

  public static double RotationsPersecToMetersPerSec(double RotationsPerSec, double wheelDiameter, double gearing){
    double mps = (RotationsPerSec / gearing) * (wheelDiameter * Math.PI);
    return mps;
  }

  public static double RotationsToMeters(double rotations, double wheelDiameter, double gearing){
    double meters = (rotations / gearing) * (wheelDiameter * Math.PI);
    return meters;
  }
}
