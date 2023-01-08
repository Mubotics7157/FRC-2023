package frc.robot.util;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class CommonConversions {
  
     public static double stepsToMeters(double steps){
      return steps*((DriveConstants.WHEEL_DIAMETER_METERS * Math.PI) / (2048*Constants.SwerveModuleConstants.DRIVE_GEAR_RATIO));
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
  public static double metersToSteps(double meters) {
    return (meters / 0.1016 / Math.PI) *2048*Constants.SwerveModuleConstants.DRIVE_GEAR_RATIO;
  }

    /**
   * Converts from meters per second to encoder units per 100 milliseconds.
   * @param metersPerSec meters per second
   * @return encoder units per decisecond
   */
  public static double metersPerSecToStepsPerDecisec(double metersPerSec) {
    return metersToSteps(metersPerSec) * .1d;
  }

  public static double RPMToStepsPerDecisec(double velRPM){
     return velRPM * 2048/600;
  }

  public static double stepsPerDecisecToRPM(double nativeVel){
    return nativeVel/2048*600;
  }


  public static double radiansToSteps(double rad){
    double radPerStep = (((2*Math.PI)/12.8)) / 2048;
    return rad/radPerStep;
  }

  public static double stepsToRadians(double steps){
    double radPerStep = (((2*Math.PI)/12.8)) / 2048;
    return steps*radPerStep;
  }

  public static double radPerSecToStepsPerDecisec(double radPerSec){
    return radiansToSteps(radPerSec) *.1;
  }

  public static double radPerSecSquaredToStepsPerDecisecSquared(double radPerSec){
    return radiansToSteps(radPerSec) *.01;
  }

}
