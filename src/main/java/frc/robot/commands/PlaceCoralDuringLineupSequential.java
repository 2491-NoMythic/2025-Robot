// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.settings.Constants.AlgaeEndeffectorConstants.ALGAE_INTAKE_SPEED;
import static frc.robot.settings.Constants.ElevatorConstants.METERS_FROM_POSE_TO_RAISE_ELEVATOR;
import static frc.robot.settings.Constants.Field.REEF_POSITION_THRESHOLD;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.AlgaeEndeffectorSubsystem;
import frc.robot.subsystems.CoralEndeffectorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.DriveToPose;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.commands.NamedCommands.DeliverCoral;
import frc.robot.settings.ElevatorEnums;
import frc.robot.settings.PlacementLocations;
import frc.robot.commands.WaitUntil;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.RobotState;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceCoralDuringLineupSequential extends SequentialCommandGroup {
  /** Creates a new PlaceCoralDuringLineupSequential. */
  public PlaceCoralDuringLineupSequential(AlgaeEndeffectorSubsystem algaeEndDefector, DrivetrainSubsystem driveTrain, ElevatorSubsystem elevator, CoralEndeffectorSubsystem coralEndDefector, Supplier<PlacementLocations> placementSupplier, Supplier<ElevatorEnums> heightSupplier) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()->RobotState.getInstance().reefLineupRunning = true),
      new InstantCommand(()->algaeEndDefector.runAlgaeEndDefector(() -> RobotState.getInstance().goForAlgae ? ALGAE_INTAKE_SPEED : -0.5), algaeEndDefector),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new WaitCommand(0.1),
          new WaitUntil(()->driveTrain.getPositionTargetingError() < METERS_FROM_POSE_TO_RAISE_ELEVATOR),
          new InstantCommand(()->elevator.setElevatorPosition(heightSupplier), elevator),
          new WaitUntil(()-> (DriverStation.isAutonomous() ? driveTrain.getPositionTargetingError() < 0.02 : driveTrain.getPositionTargetingError() < REEF_POSITION_THRESHOLD) && elevator.isElevatorAtPose()),
          new ParallelRaceGroup(
            new DeliverCoral(coralEndDefector),//drops coral
            new WaitCommand(0.75))),
        new SequentialCommandGroup(
          new DriveToPose(placementSupplier, driveTrain, ()->0),
          new InstantCommand(()->driveTrain.drive(new ChassisSpeeds(0, 0, 0)))))
    );
  }
}
