package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.MythicalMath;
import frc.robot.settings.ReefOffsetEnums;
import static frc.robot.settings.Constants.SensorConstants.*;

import org.opencv.core.Mat;

import frc.robot.settings.SensorNameEnums;
public class DistanceSensors  extends SubsystemBase{

    
  private TimeOfFlight farLeft;
  private TimeOfFlight middleLeft;
  private TimeOfFlight middleRight;
  private TimeOfFlight farRight;
  private TimeOfFlight elevatorSensor;

  private int loopsFLValid;
  private int loopsMLValid;
  private int loopsFRValid;
  private int loopsMRValid;
  private double previousFL = 0;
  private double previousML = 0;
  private double previousFR = 0;
  private double previousMR = 0;
  private final int loopsNeededForValid = 2;

  private double loopsSensed;

  public double distanceOfFrontDistancer;
  
  public DistanceSensors(){
    loopsSensed = 0;
    //TODO these must be configured from the roborio
    elevatorSensor = new TimeOfFlight(ELEVATOR_SENSOR_ID);
    farLeft = new TimeOfFlight(FAR_LEFT_DIST_SENSOR_ID);
    middleLeft = new TimeOfFlight(MIDDLE_LEFT_DIST_SENSOR_ID);
    middleRight = new TimeOfFlight(MIDDLE_RIGHT_DIST_SENSOR_ID);
    farRight = new TimeOfFlight(FAR_RIGHT_DIST_SENSOR_ID);
    SmartDashboard.putNumber("SENSOR/farLeftSampleTime", farLeft.getSampleTime());
    farLeft.setRangingMode(RangingMode.Short, 24);
    middleLeft.setRangingMode(RangingMode.Short, 24);
    middleRight.setRangingMode(RangingMode.Short, 24);
    farRight.setRangingMode(RangingMode.Short, 24);
  }
/**
 * a method that returns the distance sensed by the given sensor, as shown below. If a sensor senses nothing within it's range, It returns 0.0 <br>
 * Left To Right, as Viewed from behind the line of sensors <p>
 * Elevator Sensor ID: Elevator  <p>
 * Far Left ID: FarLeft <p>
 * Middle Left ID: MiddleLeft <p>
 * Middle Right ID: MiddleRight <p>
 * Far Right ID: FarRight <p>
 * @param sensorName the ID of the sensor, as shown above
 * @return the distance sensed by given sensor
 */
  public double getDistance(SensorNameEnums sensorName) {
    switch (sensorName) {
      case Elevator:
        return elevatorSensor.getRange();
      case FarLeft:
        return farLeft.getRange();
      case MiddleLeft:
        return middleLeft.getRange();
      case MiddleRight:
        return middleRight.getRange();
      case FarRight:
        return farRight.getRange();
      default:
      System.out.println("attempted to grab data from invalid sensor ID");
        return 2491;
    }
  }

  @Override
  public void periodic() {
      updateForReefApproach();
      updateRobotState();
      updateValidity();
  }
/**
 * this method should be called periodically to update the FLValid, FRValid, MLValid, and MRValid numbers based on wether or not the sensor
 * is believed to be staring off into space or not. These numbers are used to verify if sensor readings are valid or just blinking static, and also used in the getValidRange method
 */
  private void updateValidity() {
    if(Math.abs(farLeft.getRange()-previousFL)>50) {
      loopsFLValid = 0;
    } else {
      loopsFLValid++;
    }

    if(Math.abs(middleLeft.getRange()-previousML)>50) {
      loopsMLValid = 0;
    } else {
      loopsMLValid++;
    }

    if(Math.abs(farRight.getRange()-previousFR)>50) {
      loopsFRValid = 0;
    } else {
      loopsFRValid++;
    }

    if(Math.abs(middleRight.getRange()-previousMR)>50) {
      loopsMRValid = 0;
    } else {
      loopsMRValid++;
    }

    previousFL = farLeft.getRange();
    previousFR = farRight.getRange();
    previousML = middleLeft.getRange();
    previousMR = middleRight.getRange();
  }
/**
 * 
 * @param sensorName the name of the sensor that you want to check for
 * @return if the sensor is blinking rapidly between values, then returns 0. If not, it returns the sensors range
 */
  public double getValidRange(SensorNameEnums sensorName) {
    switch (sensorName) {
      case FarLeft:
        if(loopsFLValid>loopsNeededForValid) {
          return farLeft.getRange();
        } else {
          return 0;
        }
      case MiddleLeft:
        if(loopsMLValid>loopsNeededForValid) {
          return middleLeft.getRange();
        } else {
          return 0;
        }
      case FarRight: {
        if(loopsFRValid>loopsNeededForValid) {
          return farRight.getRange();
        } else {
          return 0;
        }
      }
      case MiddleRight: {
        if(loopsMRValid>loopsNeededForValid) {
          return middleRight.getRange();
        } else {
          return 0;
        }
      }
      
    }
        return 0;
  }

  private void updateForReefApproach() {
    if(middleRight.getRange() == 0) {
      loopsSensed = 0;
    } else {
      loopsSensed++;
    }
    if(loopsSensed>10) {
      distanceOfFrontDistancer = middleRight.getRange();
    } else {
      distanceOfFrontDistancer = 0;
    }
  } 
/**
 * figures out the state of the robot, in terms of it being aligned infront of the reef. can return any state from ReefOffsetEnums
 * @param sFL true if the far left sensor senses the reef in front of it
 * @param sL true if the middle left sensor sense the reef in front of it
 * @param sR true if the middle right sensor sense the reef in front of it
 * @param sFR true if the far right sensor sense the reef in front of it
 * @return what state of alignemnt in front of the reef our robot is, based on the inputted sensor readings
 */
  private ReefOffsetEnums calcOffset(boolean sFL, boolean sL, boolean sR, boolean sFR){
    int value = 0;
    if (sFL) {
      value |= 0b1000;
    }
    if (sL) {
      value |= 0b0100;
    }
    if (sR) {
      value |= 0b0010;
    }
    if (sFR) {
      value |= 0b0001;
    }
    switch (value) {
      
      case 0b0001:
      case 0b0011: 
      return ReefOffsetEnums.TOO_FAR_LEFT;
      case 0b0111: 
         return ReefOffsetEnums.ALIGNED_LEFT;
      case 0b1111:
      case 0b1001: 
        return ReefOffsetEnums.CENTERED;
      case 0b1110: 
        return ReefOffsetEnums.ALIGNED_RIGHT;
      case 0b1000:
      case 0b1100: 
      return ReefOffsetEnums.TOO_FAR_RIGHT;
       case 0b0000: 
       return ReefOffsetEnums.NOT_SENSED;
       default: 
        return ReefOffsetEnums.UNKNOWN;
    }
  }

  private void updateRobotState() {
  //post ranges for each sensor to SmartDashboard
    SmartDashboard.putNumber("SENSOR/RANGE/elevator", elevatorSensor.getRange());
    SmartDashboard.putNumber("SENSOR/RANGE/far left", farLeft.getRange());
    SmartDashboard.putNumber("SENSOR/RANGE/middle left", middleLeft.getRange());
    SmartDashboard.putNumber("SENSOR/RANGE/middle right", middleRight.getRange());
    SmartDashboard.putNumber("SENSOR/RANGE/far right", farRight.getRange());

    SmartDashboard.putNumber("SENSOR/VALIDRANGE/far left", getValidRange(SensorNameEnums.FarLeft));
    SmartDashboard.putNumber("SENSOR/VALIDRANGE/middle left", getValidRange(SensorNameEnums.MiddleLeft));
    SmartDashboard.putNumber("SENSOR/VALIDRANGE/middle right", getValidRange(SensorNameEnums.MiddleRight));
    SmartDashboard.putNumber("SENSOR/VALIDRANGE/far right", getValidRange(SensorNameEnums.FarRight));
    //update Robot State with sensor readings
    RobotState.getInstance().farLeftSensorTriggered = 
      farLeft.getRange()<RANGE_TO_SEE_REEF_ANGLED_AND_SPACED_SENSORS //sensor senses that we are close enough to the reef to deliver a coral
      && farLeft.getRange()>0 //the sensor does not read (which it sometimes reads when there is nothing within it's sensing range)
      && loopsFLValid>loopsNeededForValid; //the sensor has given a consistent, smoothly changing reading for at least x loops of the code, meaning it isn't just blinking static
    RobotState.getInstance().middleLeftSensorTriggered = 
      middleLeft.getRange()<RANGE_TO_SEE_REEF_FLAT_SENSORS
      && middleLeft.getRange()>0
      && loopsMLValid>loopsNeededForValid;
    RobotState.getInstance().middleRightSensorTriggered =
      middleRight.getRange()<RANGE_TO_SEE_REEF_FLAT_SENSORS
      && middleRight.getRange()>0
      && loopsFRValid>loopsNeededForValid;
    RobotState.getInstance().farRightSensorTriggered =
      farRight.getRange()<RANGE_TO_SEE_REEF_FLAT_SENSORS
      && farRight.getRange()>0
      && loopsMRValid>loopsNeededForValid;
    RobotState.getInstance().reefOffset = calcOffset(
      RobotState.getInstance().farLeftSensorTriggered, 
      RobotState.getInstance().middleLeftSensorTriggered,
      RobotState.getInstance().middleRightSensorTriggered,
      RobotState.getInstance().farRightSensorTriggered
    );
  //for testing purposes
    SmartDashboard.putBoolean("TRIGGERED/FarLeft", RobotState.getInstance().farLeftSensorTriggered);
    SmartDashboard.putBoolean("TRIGGERED/FarRight", RobotState.getInstance().farRightSensorTriggered);
    SmartDashboard.putBoolean("TRIGGERED/MiddleLeft", RobotState.getInstance().middleLeftSensorTriggered);
    SmartDashboard.putBoolean("TRIGGERED/MiddleRight", RobotState.getInstance().middleRightSensorTriggered);
  }
}
