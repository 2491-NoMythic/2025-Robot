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
  public boolean coralSeen;
  public boolean coralGone;
  public boolean hasAlgae;
  public ElevatorEnums deliveringCoralHeight;
  public ReefOffsetEnums reefOffset;
  public boolean funnelSensorTrig;
  public boolean coralEndeffSensorTrig;
  public ReefSideEnum closestReefSide;

  private RobotState() {
  }

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }
  }

