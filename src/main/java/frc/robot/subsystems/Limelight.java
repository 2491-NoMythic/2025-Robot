package frc.robot.subsystems;

import static frc.robot.settings.Constants.Vision.*;

import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.LogInputs.LimelightInputs;
import frc.robot.helpers.MythicalMath;
import java.io.IOException;
import org.littletonrobotics.junction.Logger;
import java.util.ArrayList;
import java.util.List;

public class Limelight {

  private static Limelight limelight;
/** DO NOT edit anywhere other than periodic() of Limelight class */
public LimelightInputs limelightA = new LimelightInputs(APRILTAG_LIMELIGHTA_NAME);
/** DO NOT edit anywhere other than periodic() of Limelight class */
public LimelightInputs limelightB = new LimelightInputs(APRILTAG_LIMELIGHTB_NAME);
/** DO NOT edit anywhere other than periodic() of Limelight class */
  public LimelightInputs limelightC = new LimelightInputs(APRILTAG_LIMELIGHTC_NAME);

  public static final AprilTagFieldLayout FIELD_LAYOUT;

  static {
    try {
      FIELD_LAYOUT =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
      FIELD_LAYOUT.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  private Limelight() {
    SmartDashboard.putBoolean("Vision/Left/valid", false);
    SmartDashboard.putBoolean("Vision/Left/trusted", false);
    SmartDashboard.putBoolean("Vision/Right/valid", false);
    SmartDashboard.putBoolean("Vision/Right/trusted", false);
    SmartDashboard.putBoolean("Vision/Extra/valid", false);
    SmartDashboard.putBoolean("Vision/Extra/trusted", false);

    // logs active selected path
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  public static Limelight getInstance() {
    if (limelight == null) {
      limelight = new Limelight();
    }
    return limelight;
  }

  /**
   * Gets the most recent limelight pose estimate, given that a trustworthy estimate is available.
   * Uses the provided odometryPose for additional filtering.
   *
   * <p>Trusted poses must:
   *
   * <ul>
   *   <li>Be within field bounds.
   *   <li>Have an average tag distance within [MAX_TAG_DISTANCE] from the robot.
   *   <li>Be within [ALLOWABLE_POSE_DIFFERENCE] from the given odometryPose.
   * </ul>
   *
   * @param odometryPose The current odometry pose estimate
   * @return A valid and trustworthy pose. Null if no valid pose. Poses are prioritized by lowest
   *     tagDistance.
   */
  public Pair<Pose2d, LimelightInputs> getTrustedPose() {
    Pose2d poseA = limelightA.megaTag2Pose2d;
    Pose2d poseB = limelightB.megaTag2Pose2d;
    Pose2d poseC = limelightC.megaTag2Pose2d;
    // we aren't using isTrustworthy here becuase as LL readings have gotten more reliable, we care
    // less about tag distance
    Boolean poseATrust = false;
    Boolean poseBTrust = false;
    Boolean poseCTrust = false;
    if(poseA != null && limelightA.isConnected && limelightA.getClosestTagDistCameraSpace()<MAX_TAG_DISTANCE) {poseATrust = isInField(limelightA);}
    if(poseB != null && limelightB.isConnected && limelightB.getClosestTagDistCameraSpace()<MAX_TAG_DISTANCE) {poseBTrust = isInField(limelightB);}
    if(poseC != null && limelightC.isConnected && limelightC.getClosestTagDistCameraSpace()<MAX_TAG_DISTANCE) {poseCTrust = isInField(limelightC);}

    logTrustToSmartDashboard(poseATrust, limelightA, "Left");
    logTrustToSmartDashboard(poseBTrust, limelightB, "Right");
    logTrustToSmartDashboard(poseCTrust, limelightC, "Extra");

    // if the limelight positions will be merged, let SmartDashboard know!
    boolean mergingPoses = false;
    if (poseATrust && poseBTrust || poseBTrust && poseCTrust || poseATrust && poseCTrust) {
      mergingPoses = true;
    }
    SmartDashboard.putBoolean("LL poses merged", mergingPoses);
    List<LimelightInputs> limelightNames = new ArrayList<>();
    if(poseATrust) {limelightNames.add(limelightA); }
    if(poseBTrust) {limelightNames.add(limelightB); }
    if(poseCTrust) {limelightNames.add(limelightC); }
    if(limelightNames.size() == 0) {
      return null;
    }else if(limelightNames.size() == 1) {
      return new Pair<>(limelightNames.get(0).megaTag2Pose2d, limelightNames.get(0));
    } else {
      return new Pair<>(mergedPose(limelightNames), limelightNames.get(0));
    }
  }
  /**
   * returns a new limelight pose that has the gyroscope rotation of pose1, with the FOMs used to
   * calculate a new pose that proportionally averages the two given positions
   *
   * @param pose1
   * @param pose2
   * @param LL1FOM
   * @param LL2FOM
   * @return
   */
  public Pose2d mergedPose(List<LimelightInputs> limelightNames) {

    Pose2d pose1 = limelightNames.get(0).megaTag2Pose2d;
    Pose2d pose2 = limelightNames.get(1).megaTag2Pose2d;

    Logger.recordOutput("mergedPose/pose1", pose1);
    Logger.recordOutput("mergedPose/pose2", pose2);

    double LL1FOM = getLLFOM(limelightNames.get(0));
    double LL2FOM = getLLFOM(limelightNames.get(1));
    double confidenceSource1 = 1 / Math.pow(LL1FOM, 2);
    double confidenceSource2 = 1 / Math.pow(LL2FOM, 2);
    Pose2d scaledPose1 = MythicalMath.multiplyOnlyPos(pose1, confidenceSource1);
    Pose2d scaledPose2 = MythicalMath.multiplyOnlyPos(pose2, confidenceSource2);

    if(limelightNames.size() == 2) {
      Pose2d newPose =
        MythicalMath.divideOnlyPos(
          (MythicalMath.addOnlyPosTogether(scaledPose1, scaledPose2)),
          (confidenceSource1 + confidenceSource2));
          pose1 = newPose;
    }
      
      if(limelightNames.size() == 3) {
        Pose2d pose3 = limelightNames.get(2).megaTag2Pose2d;
        double LL3FOM = getLLFOM(limelightNames.get(2));
        double confidenceSource3 = 1 / Math.pow(LL3FOM, 2);
        Pose2d scaledPose3 = MythicalMath.multiplyOnlyPos(pose3, confidenceSource3);
        Pose2d newPose =
            MythicalMath.divideOnlyPos(
                MythicalMath.addOnlyPosTogether(
                  MythicalMath.addOnlyPosTogether(scaledPose1, scaledPose2), scaledPose3),
                (confidenceSource1 + confidenceSource2 + confidenceSource3));
        pose1 = newPose;
      }

      return pose1;
  }
  /**
   * calculates the distance to the closest tag that is seen by the limelight
   *
   * @param limelightName
   * @return
   */
  public double getClosestTagDist(String limelightName) {
    double limelight1y = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName).getY();
    double limelight1x = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName).getX();
    double limelight1z = LimelightHelpers.getTargetPose3d_CameraSpace(limelightName).getZ();
    // use those coordinates to find the distance to the closest apriltag for each limelight
    double distance1 = MythicalMath.DistanceFromOrigin3d(limelight1x, limelight1y, limelight1z);
    return distance1;
  }
  public boolean isPoseNotNull(String limelightName) {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName) != null;
  }
   
  /**
   * larger FOM is bad, and should be used to indicate that this limelight is less trestworthy
   *
   * @param limelightName
   * @return
   */
  public double getLLFOM(LimelightInputs limelightInputs) // larger fom is BAD, and is less trustworthy.
      {
    // the value we place on each variable in the FOM. Higher value means it will get weighted more
    // in the final FOM
    /*These values should be tuned based on how heavily you want a contributer to be favored. Right now, we want the # of tags to be the most important
     * with the distance from the tags also being immportant. and the tx and ty should only factor in a little bit, so they have the smallest number. Test this by making sure the two
     * limelights give very different robot positions, and see where it decides to put the real robot pose.
     */
    double distValue = 6;
    double tagCountValue = 7;
    double xyValue = 1;

    // numTagsContributer is better when smaller, and is based off of how many april tags the
    // Limelight identifies
    double numTagsContributer;
    double tagCount = limelightInputs.tagCount;
    if (tagCount <= 0) {
      numTagsContributer = 0;
    } else {
      numTagsContributer = 1 / tagCount;
    }
    // tx and ty contributers are based off where on the limelights screen the april tag is. Closer
    // to the center means the contributer will bea smaller number, which is better.
    double centeredTxContributer =
        Math.abs(limelightInputs.tx)
            / 29.8; // tx gets up to 29.8, the closer to 0 tx is, the closer to the center it is.
    double centeredTyContributer =
        Math.abs(limelightInputs.ty)
            / 20.5; // ty gets up to 20.5 for LL2's and down. LL3's go to 24.85. The closer to 0 ty
    // is, the closer to the center it is.
    // the distance contributer gets smaller when the distance is closer, and is based off of how
    // far away the closest tag is
    double distanceContributer = (limelightInputs.getClosestTagDistCameraSpace() / 5);

    // calculates the final FOM by taking the contributors and multiplying them by their values,
    // adding them all together and then dividing by the sum of the values.
    double LLFOM =
        ((distValue * distanceContributer)
                    + (tagCountValue * numTagsContributer)
                    + (centeredTxContributer * xyValue)
                    + (centeredTyContributer))
                / distValue
            + tagCountValue
            + xyValue
            + xyValue;
    Logger.recordOutput("Vision/LLFOM/" + limelightInputs.name, LLFOM);
    return LLFOM;
  }

  /**
   * logs trust, avgTagDistance, tagCount, and latency to SmartDashboard
   * @param limelightTrust true if the limelight will be considered a valid pose when merging poses
   * @param limelight the LimelightInputs object where the data to be logged is held
   * @param title the name for the limelight that will appear on SmartDashboard
   */
  private void logTrustToSmartDashboard(boolean limelightTrust, LimelightInputs limelight, String title) {
      SmartDashboard.putBoolean("Vision/" + title + "/valid", limelight.isConnected && limelight.megaTag2Pose2d != null);
      SmartDashboard.putBoolean("Vision/" + title + "/trusted", limelightTrust);
      SmartDashboard.putNumber("Vision/" + title + "/Stats/avgTagDist", limelight.avgTagDistance);
      SmartDashboard.putNumber("Vision/" + title + "/Stats/tagCount", limelight.tagCount);
      SmartDashboard.putNumber("Vision/" + title + "/Stats/latency", limelight.latency);
  }
  /**
   * checks if the megaTag pose reported from the limelight is within the field boundaries declared in Constants.java
   * @param limelight
   * @return
   */
  private boolean isInField(LimelightInputs limelight) {
    return (limelight.megaTag2Pose2d.getX() < FIELD_CORNER.getX()
          && limelight.megaTag2Pose2d.getX() > 0.0
          && limelight.megaTag2Pose2d.getY() < FIELD_CORNER.getY()
          && limelight.megaTag2Pose2d.getY() > 0.0);
  }

  public void periodic() {
      limelightA.updateInputs();
      limelightB.updateInputs();
      limelightC.updateInputs();
      Logger.processInputs("Limelights/Left", limelightA);
      Logger.processInputs("Limelights/Right", limelightB);
      Logger.processInputs("Limelights/Extra", limelightC);
  }

}
