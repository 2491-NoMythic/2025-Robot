// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.settings.Constants.AlgaeEndeffectorConstants.ALGAE_SHOOT_SPEED;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.settings.ElevatorEnums;
import frc.robot.subsystems.AlgaeEndeffectorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DepositAlgaeSequential extends SequentialCommandGroup {
  /** Creates a new DepositAlgaeSequential. */
  public DepositAlgaeSequential(ElevatorSubsystem elevator, AlgaeEndeffectorSubsystem algaeEndEffector, DrivetrainSubsystem drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SequentialCommandGroup(
          new InstantCommand(()->elevator.setElevatorPosition(ElevatorEnums.AlgaeInProcessor)),
          new WaitUntil(()->elevator.isElevatorAtPose() && drivetrain.getPositionTargetingError() < 0.05),
          new InstantCommand(()->algaeEndEffector.runAlgaeEndDefector(-0.5)),
          new DriveTimeCommand(-0.83, 0, 0, .33, drivetrain),
          new DriveTimeCommand(0.4, 0, 0, 1, drivetrain)));
  }
}