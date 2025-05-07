// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ClimberSubsystem;
import static frc.robot.settings.Constants.ClimberConstants.CLIMBER_POWER_FORWARD;
import static frc.robot.settings.Constants.ClimberConstants.CLIMBER_POWER_REVERSE;

public class ClimberCommandGroup extends SequentialCommandGroup {
/**
 * Creates a new ClimberCommandGroup to run the climber.
 * @param climber ClimberSubsystem
 * @param isRaisingClimber checks whether or not the climber is going up
 */
  ClimberSubsystem climber;
  boolean isRaisingClimber;
  public ClimberCommandGroup( ClimberSubsystem climber, boolean isRaisingClimber) {
    this.climber = climber;
    this.isRaisingClimber = isRaisingClimber;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()->climber.runWheels(()->isRaisingClimber ? -1 : 0), climber),
      new InstantCommand(() -> climber.setServo( ()-> isRaisingClimber), climber),
      new WaitCommand(() -> isRaisingClimber ? 0.5 : 0),
      new InstantCommand(() -> climber.setClimberPower(()->isRaisingClimber ? CLIMBER_POWER_FORWARD : CLIMBER_POWER_REVERSE), climber));
  }

  // @Override
  // public InterruptionBehavior getInterruptionBehavior() {
  //   return InterruptionBehavior.kCancelIncoming;
  // }
}
