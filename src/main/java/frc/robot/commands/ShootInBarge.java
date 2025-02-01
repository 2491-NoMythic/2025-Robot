// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.settings.ElevatorEnums;
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

  public ShootInBarge(DrivetrainSubsystem drivetrainSubsystem, ElevatorSubsystem elevatorSubsystem,
      AlgaeEndeffectorSubsystem algaeSubsystem) {
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.elevatorSubsystem = elevatorSubsystem;
    this.algaeSubsystem = algaeSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new LineUpBarge(drivetrainSubsystem),
        new ElevatorCommand(elevatorSubsystem, () -> ElevatorEnums.Barge),
        (new AlgaeIntakeCommand(algaeSubsystem, 1)));
  }
}
