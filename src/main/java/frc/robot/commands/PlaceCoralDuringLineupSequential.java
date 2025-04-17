// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.settings.Constants.AlgaeEndeffectorConstants.ALGAE_INTAKE_SPEED;
import static frc.robot.settings.Constants.ElevatorConstants.METERS_FROM_POSE_TO_RAISE_ELEVATOR;
import static frc.robot.settings.Constants.Field.REEF_POSITION_THRESHOLD;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeEndeffectorSubsystem;
import frc.robot.subsystems.CoralEndeffectorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.NamedCommands.DeliverCoral;
import frc.robot.settings.ElevatorEnums;
import frc.robot.settings.PlacementLocations;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.RobotState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

public class PlaceCoralDuringLineupSequential extends SequentialCommandGroup {
  /**
   * This command begins placing coral during the lineup proecess in an attempt to make the process more efficient.
   * @param algaeEndDefector
   * @param driveTrain
   * @param elevator
   * @param coralEndDefector
   * @param placementSupplier
   * @param heightSupplier
   */
  public PlaceCoralDuringLineupSequential(AlgaeEndeffectorSubsystem algaeEndDefector, DrivetrainSubsystem driveTrain, ElevatorSubsystem elevator, CoralEndeffectorSubsystem coralEndDefector, Supplier<PlacementLocations> placementSupplier, Supplier<ElevatorEnums> heightSupplier) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()->RobotState.getInstance().reefLineupRunning = true),
      new InstantCommand(()->RobotState.getInstance().deliveringCoralHeight = heightSupplier.get()),
      new InstantCommand(()->coralEndDefector.stopCoralEndEffector(), coralEndDefector),
      new InstantCommand(()->algaeEndDefector.runAlgaeEndDefector(() -> RobotState.getInstance().goForAlgae ? ALGAE_INTAKE_SPEED : -0.5), algaeEndDefector),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new WaitCommand(()->0.1),
          new WaitUntil(()->driveTrain.getPositionTargetingError() < METERS_FROM_POSE_TO_RAISE_ELEVATOR),
          new InstantCommand(()->elevator.setElevatorPosition(heightSupplier), elevator),
          new WaitUntil(()-> (DriverStation.isAutonomous() ? driveTrain.getPositionTargetingError() < 0.02 : driveTrain.getPositionTargetingError() < REEF_POSITION_THRESHOLD) && elevator.isElevatorAtPose()),
          new ParallelRaceGroup(
            new DeliverCoral(coralEndDefector),//drops coral
            new WaitCommand(()-> DriverStation.isTeleop() ? 0.5 : 0.3))),//0.3 for auto gets to our speed, velocity reports at 33-35 rps
        new SequentialCommandGroup(
          new DriveToPose(placementSupplier, driveTrain, ()->0),
          new InstantCommand(()->driveTrain.drive(new ChassisSpeeds(0, 0, 0)))))
    );
  }
}
