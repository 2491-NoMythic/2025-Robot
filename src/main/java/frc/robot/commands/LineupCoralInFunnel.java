// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.settings.Constants.FunnelConstants;
import frc.robot.subsystems.FunnelIntake;
import frc.robot.subsystems.RobotState;

public class LineupCoralInFunnel extends SequentialCommandGroup {
  /**
   * Makes sure the coral is lined up correctly in the endeffector.
   * This has the funnel side of the controls.
   * @param funnel
   */
  public LineupCoralInFunnel(FunnelIntake funnel) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    final double adjustingSpeed = FunnelConstants.FUNNEL_ADJUSTING_INTAKE_SPEED;
    addCommands(
      new InstantCommand(()->RobotState.getInstance().coralLineupRunning = true),
      new InstantCommand(()->funnel.runFunnel(-adjustingSpeed), funnel),
      new WaitUntil(()->!RobotState.getInstance().funnelSensorTrig),
      new InstantCommand(()->funnel.runFunnel(adjustingSpeed), funnel),
      new WaitUntil(()->RobotState.getInstance().funnelSensorTrig),
      new InstantCommand(()->funnel.stopFunnel(), funnel),
      new InstantCommand(()->RobotState.getInstance().coralAligned = true),
      new InstantCommand(()->RobotState.getInstance().coralLineupRunning = false)
    );
  }
}
