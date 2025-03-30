// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NamedCommands;

import static frc.robot.settings.Constants.AlgaeEndeffectorConstants.ALGAE_INTAKE_SPEED;
import static frc.robot.settings.Constants.ElevatorConstants.METERS_FROM_POSE_TO_RAISE_ELEVATOR;
import static frc.robot.settings.Constants.ElevatorConstants.REEF_LEVEL_4_CENTIMETERS_AGAINST_REEF;
import static frc.robot.settings.Constants.Field.REEF_POSITION_THRESHOLD;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.AlgaeEndeffectorSubsystem;
import frc.robot.subsystems.CoralEndeffectorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.WaitCommand;
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
public class PlaceCoralDuringAuto extends SequentialCommandGroup {
  /** Creates a new PlaceCoralDuringLineupSequential. */
  public PlaceCoralDuringAuto(AlgaeEndeffectorSubsystem algaeEndDefector, DrivetrainSubsystem driveTrain, ElevatorSubsystem elevator, CoralEndeffectorSubsystem coralEndDefector, Supplier<PlacementLocations> placementSupplier, Supplier<ElevatorEnums> heightSupplier) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()->RobotState.getInstance().reefLineupRunning = true),
      new InstantCommand(()->coralEndDefector.stopCoralEndEffector(), coralEndDefector),
      new InstantCommand(()->algaeEndDefector.runAlgaeEndDefector(() -> RobotState.getInstance().goForAlgae ? ALGAE_INTAKE_SPEED : -0.5), algaeEndDefector),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new WaitCommand(()->0.1),
          new WaitUntil(()->driveTrain.getPositionTargetingError() < METERS_FROM_POSE_TO_RAISE_ELEVATOR),
          new InstantCommand(()->elevator.setElevatorPosition(REEF_LEVEL_4_CENTIMETERS_AGAINST_REEF), elevator),
          new WaitUntil(()-> (driveTrain.getPositionTargetingError() < REEF_POSITION_THRESHOLD) && elevator.isElevatorAtPose()),
          new WaitCommand(()->0.5),
          new ParallelRaceGroup(
            new DeliverCoral(coralEndDefector),//drops coral
            new WaitCommand(()-> DriverStation.isTeleop() ? 0.5 : 0.17))),
        new SequentialCommandGroup(
          new ParallelRaceGroup(
            new WaitUntil(()->driveTrain.getPositionTargetingError() < REEF_POSITION_THRESHOLD),
            new DriveToPose(placementSupplier, driveTrain, ()->0)),
          new InstantCommand(()->driveTrain.drive(new ChassisSpeeds(0.5, 0, 0))))
      )
    );
  }
}
