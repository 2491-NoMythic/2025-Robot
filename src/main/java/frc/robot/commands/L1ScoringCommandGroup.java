// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.settings.Constants.AlgaeEndeffectorConstants.ALGAE_INTAKE_SPEED;
import static frc.robot.settings.Constants.CoralEndeffectorConstants.CORAL_ENDEFFECTOR_SPEED;
import static frc.robot.settings.Constants.ElevatorConstants.METERS_FROM_POSE_TO_RAISE_ELEVATOR;
import static frc.robot.settings.Constants.ElevatorConstants.REEF_LEVEL_1_CENTIMETERS_AGAINST_REEF;
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

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L1ScoringCommandGroup extends SequentialCommandGroup {
  /** Creates a new PlaceCoralDuringLineupSequential. */
  public L1ScoringCommandGroup(AlgaeEndeffectorSubsystem algaeEndDefector, DrivetrainSubsystem driveTrain, ElevatorSubsystem elevator, CoralEndeffectorSubsystem coralEndDefector, Supplier<PlacementLocations> placementSupplier) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()->RobotState.getInstance().reefLineupRunning = true),
      new InstantCommand(()->coralEndDefector.stopCoralEndEffector(), coralEndDefector),
      new InstantCommand(()->algaeEndDefector.runAlgaeEndDefector(() -> RobotState.getInstance().goForAlgae ? ALGAE_INTAKE_SPEED : -0.5), algaeEndDefector),
      new InstantCommand(()->elevator.setElevatorPosition(ElevatorEnums.Reef1), elevator),
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new DriveToPose(placementSupplier, driveTrain, ()->0),
          new WaitUntil(()->elevator.isElevatorAtPose()),
          new InstantCommand(()->driveTrain.driveForL1Scoring(()->RobotState.getInstance().L1selectedPosition)),
          new WaitCommand(0.7),
          new InstantCommand(()->driveTrain.stop()),
          new InstantCommand(()->coralEndDefector.runCoralEndEffector(900)),
          new WaitCommand(0.5),
          new InstantCommand(()->elevator.setElevatorPositionDynamicConfigs(REEF_LEVEL_1_CENTIMETERS_AGAINST_REEF+30, 800, 300, 0), elevator),
          new WaitUntil(()->elevator.isElevatorAtPose())
        ),
        new SequentialCommandGroup(
        )
      )
    );
  }
}
