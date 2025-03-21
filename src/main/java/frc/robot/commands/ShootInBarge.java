// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.settings.Constants.ElevatorConstants.HUMAN_PLAYER_STATION_CENTIMETERS;
import static frc.robot.settings.Constants.ElevatorConstants.METERS_FROM_POSE_TO_RAISE_ELEVATOR;
import static frc.robot.settings.Constants.ElevatorConstants.MOTION_MAGIC_ELEVATOR_HIGH_ACCLERATION;
import static frc.robot.settings.Constants.ElevatorConstants.MOTION_MAGIC_ELEVATOR_HIGH_VELOCITY;
import static frc.robot.settings.Constants.Field.BARGE_POSITION_THRESHOLD;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.settings.ElevatorEnums;
import frc.robot.settings.PlacementLocations;
import frc.robot.subsystems.AlgaeEndeffectorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootInBarge extends SequentialCommandGroup {
  /** Creates a new ShootInBarge. */
  DrivetrainSubsystem drivetrainSubsystem;
  ElevatorSubsystem elevatorSubsystem;
  AlgaeEndeffectorSubsystem algaeSubsystem;
  DoubleSupplier controllerSidwaysSupplier;

  public ShootInBarge(DrivetrainSubsystem drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem,
      AlgaeEndeffectorSubsystem algaeSubsystem, DoubleSupplier controllerSupplier) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.algaeSubsystem = algaeSubsystem;
    this.controllerSidwaysSupplier = controllerSupplier;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new WaitCommand(()->0.1), //ensures driveToPose has started before checking the targeting error
          new WaitUntil(()->drivetrainSubsystem.getPositionTargetingErrorBarge() < METERS_FROM_POSE_TO_RAISE_ELEVATOR),
          new InstantCommand(()->elevatorSubsystem.setElevatorPosition(ElevatorEnums.Barge), elevatorSubsystem),
          new WaitUntil(()->drivetrainSubsystem.getPositionTargetingErrorBarge() < BARGE_POSITION_THRESHOLD && elevatorSubsystem.isElevatorAtPose()),
          new ParallelRaceGroup(
            new AlgaeIntakeCommand(algaeSubsystem, ()->-1),
            new WaitCommand(()->1.0)),
          new InstantCommand(()->elevatorSubsystem.setElevatorPosition(ElevatorEnums.HumanPlayer), elevatorSubsystem)),
          new DriveToPose(()->PlacementLocations.Barge, drivetrainSubsystem, controllerSupplier)
      )
    );
  }
}
