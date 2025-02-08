// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.ApproachReef;
import frc.robot.commands.LineUp;
import frc.robot.commands.MoveMeters;
import frc.robot.commands.PlaceCoralNoPath;
import frc.robot.commands.WaitUntil;
import frc.robot.settings.ElevatorEnums;
import frc.robot.subsystems.CoralEndeffectorSubsystem;
import frc.robot.subsystems.DistanceSensors;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceCoralNoOdometry extends SequentialCommandGroup {
  /** Creates a new PlaceCoralNoOdometry. */
  public PlaceCoralNoOdometry(DrivetrainSubsystem drivetrain, ElevatorSubsystem elevator, CoralEndeffectorSubsystem coralEndEffector, DistanceSensors distanceSensors, BooleanSupplier movingLeft, Supplier<ElevatorEnums> heightsupplier) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelRaceGroup(
        new MoveMeters(drivetrain, 2, 0.5, 0, 0),
        new WaitUntil(()->frc.robot.subsystems.RobotState.getInstance().middleLeftSensorTriggered && frc.robot.subsystems.RobotState.getInstance().middleRightSensorTriggered)
      ),
      new PlaceCoralNoPath(elevator, heightsupplier, distanceSensors, drivetrain, ()->0, ()->0, ()->0, coralEndEffector, movingLeft)
    );
  }
}
