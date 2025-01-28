// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.helpers;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class MythicalMath {
  /**
   * finds the absolute distance from the origin of any point on a 3d plane
   *
   * @param XfromOrigin x-coordinate of point
   * @param YfromOrigin y-coordinate of point
   * @param ZfromOrigin z-coordinate of point
   * @return distance from origin of point
   */
  public static double DistanceFromOrigin3d(
      double XfromOrigin, double YfromOrigin, double ZfromOrigin) {
    double Distance2d = Math.sqrt(Math.pow(YfromOrigin, 2) + Math.pow(XfromOrigin, 2));
    return Math.sqrt(Math.pow(Distance2d, 2) + Math.pow(ZfromOrigin, 2));
  }

  public static Pose2d multiplyOnlyPos(Pose2d pose, Double scalar) {
    return new Pose2d(pose.getX() * scalar, pose.getY() * scalar, pose.getRotation());
  }

  public static Pose2d divideOnlyPos(Pose2d pose, Double scalar) {
    return new Pose2d(pose.getX() / scalar, pose.getY() / scalar, pose.getRotation());
  }
/**
 * returns the minimum of two values, but treats any vaue that is 0 as 10,000
 * @param value1
 * @param value2
 * @return
 */
  public static double minNotZero(double value1, double value2) {
    if(value1 == 0) {
      value1 = 10000;
    }
    if(value2 == 0) {
      value2 = 10000;
    }
    return Math.min(value1, value2);
  }

  /**
   * @param pose1
   * @param pose2
   * @return poses added together, with pose1's rotation
   */
  public static Pose2d addOnlyPosTogether(Pose2d pose1, Pose2d pose2) {
    return new Pose2d(
        pose1.getX() + pose2.getX(), pose1.getY() + pose2.getY(), pose1.getRotation());
  }
/**
 * calculates the distance between two Pose2d's using the .getTranslation.getDistance(Translaiton2d) method
 * @param pose1
 * @param pose2
 * @return the distance in units, will always be positive
 */
  public static double distanceBetweenTwoPoses(Pose2d pose1, Pose2d pose2) {
    return pose1.getTranslation().getDistance(pose2.getTranslation());
  }
}
