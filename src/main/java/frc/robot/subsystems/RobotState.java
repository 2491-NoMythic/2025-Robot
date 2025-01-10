package frc.robot.subsystems;

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

  static public ReefOffset calcOffset(boolean s1, boolean s2, boolean s3, boolean s4){
    int value = 0;
    if (s1) {
      value |= 0b0001;
    }
    if (s2) {
      value |= 0b0010;
    }
    switch (value) {
      case 0b0000: return ReefOffset.NOT_SENSED;
    }
    if(s1 && s2 && s3 && s4){
      return ReefOffset.CENTERED;
    }
    //
    if(!s1 && !s2){
      return ReefOffset.TOO_FAR_LEFT;
    }

    return ReefOffset.UNKNOWN;
  }
}
