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
  private double loopsSensed;

  private TimeOfFlight frontDistancer;
  public double distanceOfFrontDistancer;


  
  public DistanceSensors(){
    lineupInit();
    loopsSensed = 0;
  }


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
      SmartDashboard.putNumber("looops sensed", loopsSensed);
      if(frontDistancer.getRange() == 0) {
        loopsSensed = 0;
      } else {
        loopsSensed++;
      }
      if(loopsSensed>10) {
        distanceOfFrontDistancer = frontDistancer.getRange();
      } else {
        distanceOfFrontDistancer = 0;
      }

  }

    
}
