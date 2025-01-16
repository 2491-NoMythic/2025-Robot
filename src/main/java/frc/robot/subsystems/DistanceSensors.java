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

  private TimeOfFlight frontDistancer;

  public double distanceOfFrontDistancer;

  @Override
  public void periodic() {
      updateDistanceSensors();
  }

  private void lineupInit() {
    //TODO these must be configured from the roborio
    /** farLeft = new TimeOfFlight(FAR_LEFT_DIST_SENSOR_ID);
    middleLeft = new TimeOfFlight(MIDDLE_LEFT_DIST_SENSOR_ID);
    middleRight = new TimeOfFlight(MIDDLE_RIGHT_DIST_SENSOR_ID);
    farRight = new TimeOfFlight(FAR_RIGHT_DIST_SENSOR_ID);
    **/
    frontDistancer = new TimeOfFlight(FRONT_DISTANCER_ID);
  }


    private void updateDistanceSensors() {

        distanceOfFrontDistancer = frontDistancer.getRange();
  
/** 
        SmartDashboard.putNumber("Sensor", farLeft.getRange());
    SmartDashboard.putBoolean("FLSensorT", farLeft.getRange()<RANGE_TO_SEE_REEF & farLeft.getRange()>0);
    SmartDashboard.putBoolean("LSensorT", middleLeft.getRange()<RANGE_TO_SEE_REEF & middleLeft.getRange()>0);
    SmartDashboard.putBoolean("RSensorT", middleRight.getRange()<RANGE_TO_SEE_REEF & middleRight.getRange()>0);
    SmartDashboard.putBoolean("FRSensorT", farRight.getRange()<RANGE_TO_SEE_REEF & farRight.getRange()>0);
    RobotState.getInstance().farLeftSensorTriggered = farLeft.getRange()<RANGE_TO_SEE_REEF && farLeft.getRange()>0;
    RobotState.getInstance().middleLeftSensorTriggered = middleLeft.getRange()<RANGE_TO_SEE_REEF && middleLeft.getRange()>0 ;
    RobotState.getInstance().middleRightSensorTriggered = middleRight.getRange()<RANGE_TO_SEE_REEF && middleRight.getRange()>0;
    RobotState.getInstance().farRightSensorTriggered = farRight.getRange()<RANGE_TO_SEE_REEF && farRight.getRange()>0;
    */

  }

    
}
