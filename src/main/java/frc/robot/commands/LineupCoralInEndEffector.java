// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.settings.Constants.CoralEndeffectorConstants;
import frc.robot.subsystems.CoralEndeffectorSubsystem;
import frc.robot.subsystems.RobotState;

public class LineupCoralInEndEffector extends SequentialCommandGroup {
  /**
   * Makes sure the coral is lined up correctly in the endeffector.
   * This has the endeffector side of the controls.
   * @param EndEffector
   */
  public LineupCoralInEndEffector(CoralEndeffectorSubsystem EndEffector) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    final double adjustingSpeed = CoralEndeffectorConstants.CORAL_ENDEFFECTOR_ADJUSTING_INTAKE_SPEED;
    addCommands(
      new InstantCommand(()->RobotState.getInstance().coralLineupRunning = true),
      new InstantCommand(()->EndEffector.runCoralEndEffector(adjustingSpeed), EndEffector),
      new WaitUntil(()->!RobotState.getInstance().coralEndeffSensorTrig),
      new InstantCommand(()->EndEffector.runCoralEndEffector(-adjustingSpeed), EndEffector),
      new WaitUntil(()->RobotState.getInstance().coralEndeffSensorTrig),
      new InstantCommand(()->EndEffector.stopCoralEndEffector(), EndEffector),
      new InstantCommand(()->RobotState.getInstance().coralAligned = true),
      new InstantCommand(()->RobotState.getInstance().coralLineupRunning = false)
    );
  }
}
