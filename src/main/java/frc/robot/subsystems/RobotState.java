package frc.robot.subsystems;

import javax.print.attribute.standard.MediaSize.Other;

public class RobotState {
  private static RobotState instance;
  public boolean LimelightsUpdated;
  public boolean lightsReset;
  public double odometerOrientation;
  public boolean farLeftSensorTriggered;
  public boolean middleLeftSensorTriggered;
  public boolean middleRightSensorTriggered;
  public boolean farRightSensorTriggered;

  private RobotState() {
  }

  public static RobotState getInstance() {
    if (instance == null) {
      instance = new RobotState();
    }
    return instance;
  }

  public enum ReefOffset {
    TOO_FAR_LEFT,
    ALIGNED_LEFT,
    CENTERED,
    ALIGNED_RIGHT,
    TOO_FAR_RIGHT,
    NOT_SENSED,
    UNKNOWN
  }

  static public ReefOffset calcOffset(boolean sFL, /*boolean sL, boolean sR,*/ boolean sFR){
    int value = 0;
    if (sFL) {
      value |= 0b1000;
    }
    // if (sL) {
    //   value |= 0b0100;
    // }
    // if (sR) {
    //   value |= 0b0010;
    // }
    if (sFR) {
      value |= 0b0001;
    }

    switch (value) {
      
       case 0b0001:
      // case 0b0011: 
       return ReefOffset.TOO_FAR_LEFT;
      // case 0b0111: 
      //   return ReefOffset.ALIGNED_LEFT;
      // case 0b1111:
      case 1001: 
        return ReefOffset.CENTERED;
      // case 0b1110: 
      //   return ReefOffset.ALIGNED_RIGHT;
       case 0b1000:
      // case 0b1100: 
       return ReefOffset.TOO_FAR_RIGHT;
       case 0b0000: 
       return ReefOffset.NOT_SENSED;
       default: 
        return ReefOffset.UNKNOWN;
    }
  }
  }

