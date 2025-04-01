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
    addCommands(
      new InstantCommand(()->climber.runWheels(()->isClimbing ? -1 : 0), climber),
      new InstantCommand(() -> climber.setServo( ()-> isClimbing), climber),
      new WaitCommand(() -> isClimbing ? 0.5 : 0),
      new InstantCommand(() -> climber.setClimberPower(()->isClimbing ? CLIMBER_POWER_FORWARD : CLIMBER_POWER_REVERSE), climber));
  }

  // @Override
  // public InterruptionBehavior getInterruptionBehavior() {
  //   return InterruptionBehavior.kCancelIncoming;
  // }
}
