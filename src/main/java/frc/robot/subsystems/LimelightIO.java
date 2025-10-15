// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashSet;
import java.util.Set;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.VisionIO.PoseObservation;
import frc.robot.subsystems.VisionIO.TargetObservation;
import frc.robot.subsystems.VisionIO.VisionIOInputs;

public class LimelightIO implements VisionIO {

  private final Supplier<Rotation2d> rotationSupplier;
  private final DoubleArrayPublisher orientationPublisher;

  private final DoubleSubscriber latencySubscriber;
  private final DoubleSubscriber txSubscriber;
  private final DoubleSubscriber tySubscriber;
  private final DoubleArraySubscriber megatag2Subscriber;
  

  /** Creates a new LimelightIO.
   * the limelight IO object is used to subscribe to the necessary data from network tables and store them in an IO object, which can be accessed by the replay simulation
   */
  public LimelightIO(String name, Supplier<Rotation2d> rotationSupplier) {
    this.rotationSupplier = rotationSupplier;
   
    var table = NetworkTableInstance.getDefault().getTable(name); //determines which limelight this IO object will read data from
    orientationPublisher = table.getDoubleArrayTopic("robot_orientation_set").publish();
    //sets each of the subcribers to be monitoring their respective value from the limelights network tables
    latencySubscriber = table.getDoubleTopic("tl").subscribe(0.0);
    txSubscriber = table.getDoubleTopic("tx").subscribe(0.0);
    tySubscriber = table.getDoubleTopic("ty").subscribe(0.0);
    megatag2Subscriber = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[] {});
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
  // Update connection status based on whether an update has been seen in the last 250ms
    inputs.connected = ((RobotController.getFPGATime() - latencySubscriber.getLastChange()) / 1000) < 250;

    inputs.latestTargetObservation = new TargetObservation(
      Rotation2d.fromDegrees(txSubscriber.get()), Rotation2d.fromDegrees(tySubscriber.get()));
    
    orientationPublisher.accept(
      new double[] {rotationSupplier.get().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0}); //updates the orientation on the limelight using the orientationPublisher
    NetworkTableInstance.getDefault().flush(); // apparently Limelight recommends this but I'm not sure what it does

    Set<Integer> tagIds = new HashSet<>();
    for(var rawSample : megatag2Subscriber.readQueue()) {
      if(rawSample.value.length == 0) continue;
      for (int i = 11; i < rawSample.value.length; i+=7) {
        tagIds.add((int) rawSample.value[i]);
      }
    
    //saves the data from any new updates read by the megatag2subscriber into the poseObservation in the inputs object
    inputs.poseObservation = new PoseObservation(
      // Timestamp, based on server timestamp of publish and latency
      rawSample.timestamp * 1.0e-6 - rawSample.value[6] * 1.0e-3,

      //3d pose estimate
      parsePose(rawSample.value), 
      
      //ambiguity, zeroed becuase pose is already disambiguied
      0.0, 
      
      //tag count
      (int) rawSample.value[7],
      
      //average tag distance
      rawSample.value[9]
      );
    
    //save tag IDs to input objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
    }
  }

  /** Parses the 3D pose from a Limelight botpose array. */
  private static Pose3d parsePose(double[] rawLLArray) {
    return new Pose3d(
        rawLLArray[0],
        rawLLArray[1],
        rawLLArray[2],
        new Rotation3d(
            Units.degreesToRadians(rawLLArray[3]),
            Units.degreesToRadians(rawLLArray[4]),
            Units.degreesToRadians(rawLLArray[5])));
  }
}
