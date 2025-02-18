package frc.robot.subsystems;

import javax.print.attribute.standard.MediaSize.Other;
import frc.robot.settings.ReefOffsetEnums;
import frc.robot.settings.ReefSideEnum;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.settings.ElevatorEnums;

public class RobotState {
  private static RobotState instance;
  public boolean LimelightsUpdated;
  public boolean lightsReset;
  public double odometerOrientation;
  public boolean farLeftSensorTriggered;
  public boolean middleLeftSensorTriggered;
  public boolean middleRightSensorTriggered;
  public boolean farRightSensorTriggered;
  public boolean hasAlgae;
  public boolean climbed;
  public ElevatorEnums deliveringCoralHeight;
  public ReefOffsetEnums reefOffset;
  public boolean funnelSensorTrig;
  public boolean coralEndeffSensorTrig;
  public ReefSideEnum closestReefSide;
  public boolean deliveringLeft;
  public boolean goForAlgae;
  public boolean reefLineupRunning;
  public boolean bargeLineUp;
  public boolean inIntakeZone;
  public boolean elevatorIsHigh;
  public boolean coralLineupRunning;
  public boolean coralAligned;
  public boolean elevatorZeroSet;

  public RobotState(){
    //sets any values that aren't periodically updated by a subsystem to a value, so that they won't return null if called before they are updated
    deliveringCoralHeight = ElevatorEnums.Reef1;
    elevatorZeroSet = false;
  }

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }
  /**
   * @return true when the coralEndEffector or funnelIntake detects a coral
   */
  public boolean isCoralSeen() {
    return funnelSensorTrig||coralEndeffSensorTrig||coralLineupRunning;
  }
  }

