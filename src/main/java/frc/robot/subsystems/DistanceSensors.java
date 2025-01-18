package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.settings.Constants.SensorConstants.*;

public class DistanceSensors  extends SubsystemBase{

    
  private TimeOfFlight farLeft;
  private TimeOfFlight middleLeft;
  private TimeOfFlight middleRight;
  private TimeOfFlight farRight;
  private TimeOfFlight elevatorSensor;
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
  }
/**
 * a method that returns the distance sensed by the given sensor, as shown below. If a sensor senses nothing within it's range, It returns 0.0 <br>
 * Left To Right, as Viewed from behind the line of sensors <p>
 * Elevator Sensor ID: 0 <p>
 * Far Left ID: 1 <p>
 * Middle Left ID: 2 <p>
 * Middle Right ID: 3 <p>
 * Far Right ID: 4 <p>
 * @param sensorNumber the ID of the sensor, as shown above
 * @return the distance sensed by given sensor
 */
  public double getDistance(int sensorNumber) {
    switch (sensorNumber) {
      case 0:
        return elevatorSensor.getRange();
      case 1:
        return farLeft.getRange();
      case 2:
        return middleLeft.getRange();
      case 3:
        return middleRight.getRange();
      case 4:
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
  }

  private void updateForReefApproach() {
    if(middleLeft.getRange() == 0) {
      loopsSensed = 0;
    } else {
      loopsSensed++;
    }
    if(loopsSensed>10) {
      distanceOfFrontDistancer = middleLeft.getRange();
    } else {
      distanceOfFrontDistancer = 0;
    }
  } 

  private void updateRobotState() {
  //post ranges for each sensor to SmartDashboard
    SmartDashboard.putNumber("SENSOR/RANGE/elevator", elevatorSensor.getRange());
    SmartDashboard.putNumber("SENSOR/RANGE/far left", farLeft.getRange());
    SmartDashboard.putNumber("SENSOR/RANGE/middle left", middleLeft.getRange());
    SmartDashboard.putNumber("SENSOR/RANGE/middle right", middleRight.getRange());
    SmartDashboard.putNumber("SENSOR/RANGE/far right", farRight.getRange());
  //update Robot State with sensor readings
    RobotState.getInstance().farLeftSensorTriggered = farLeft.getRange()<RANGE_TO_SEE_REEF && farLeft.getRange()>0;
    RobotState.getInstance().middleLeftSensorTriggered = middleLeft.getRange()<RANGE_TO_SEE_REEF && middleLeft.getRange()>0 ;
    RobotState.getInstance().middleRightSensorTriggered = middleRight.getRange()<RANGE_TO_SEE_REEF && middleRight.getRange()>0;
    RobotState.getInstance().farRightSensorTriggered = farRight.getRange()<RANGE_TO_SEE_REEF && farRight.getRange()>0;
  //for testing purposes
    SmartDashboard.putBoolean("RSensorT", middleRight.getRange()<RANGE_TO_SEE_REEF & middleRight.getRange()>0);
    SmartDashboard.putBoolean("FRSensorT", farRight.getRange()<RANGE_TO_SEE_REEF & farRight.getRange()>0);
  }
}
