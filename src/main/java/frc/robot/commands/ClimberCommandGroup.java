// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;
import static frc.robot.settings.Constants.ClimberConstants.CLIMBER_CLIMBED_ANGLE;
import static frc.robot.settings.Constants.ClimberConstants.CLIMBER_POWER_FORWARD;
import static frc.robot.settings.Constants.ClimberConstants.CLIMBER_POWER_REVERSE;

import frc.robot.commands.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimberCommandGroup extends SequentialCommandGroup {
  /** Creates a new ClimberCommandGroup. */
  ClimberSubsystem climber;
  boolean isClimbing;
  public ClimberCommandGroup( ClimberSubsystem climber, boolean isClimbing) {
    this.climber = climber;
    this.isClimbing = isClimbing;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (isClimbing) {
      addCommands(new InstantCommand(() -> climber.setServo(true)), new WaitCommand(() -> 0.5),
          new InstantCommand(() -> climber.setClimberPower(CLIMBER_POWER_FORWARD)), new WaitCommand(() -> 0.5),
          new InstantCommand(() -> climber.stopClimber()));
    } else if (!isClimbing) {
      addCommands(new InstantCommand(() -> climber.setServo(false)), new WaitCommand(() -> 0.5),
          new InstantCommand(() -> climber.setClimberPower(CLIMBER_POWER_REVERSE)), new WaitCommand(() -> 0.5),
          new InstantCommand(() -> climber.stopClimber()));
    }
  }
}
