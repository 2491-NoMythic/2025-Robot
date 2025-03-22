// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.settings.Constants.CoralEndeffectorConstants.CORAL_ENDEFFECTOR_SPEED;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.settings.Constants.CoralEndeffectorConstants;
import frc.robot.subsystems.CoralEndeffectorSubsystem;
import frc.robot.subsystems.FunnelIntake;
import frc.robot.subsystems.RobotState;

public class PassCoralToEndEffectorSequential extends SequentialCommandGroup {
  /**
   * This command transfers the coral from the funnel to the endeffector.
   * It works like PassCoralToEndEffector, but it uses a bunch of instant commands and sensors to make things work automatically. 
   * @param coralEndEffector
   * @param funnelIntake
   */
  public PassCoralToEndEffectorSequential(CoralEndeffectorSubsystem coralEndEffector, FunnelIntake funnelIntake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
      final double endEffectorAdjustingSpeed = CoralEndeffectorConstants.CORAL_ENDEFFECTOR_ADJUSTING_INTAKE_SPEED;
      final double funnelAdjustingSpeed = CoralEndeffectorConstants.CORAL_ENDEFFECTOR_ADJUSTING_INTAKE_SPEED;
    addCommands(
      new ParallelRaceGroup(
        new ResetCoralStatusOnEnd(),
        new SequentialCommandGroup(
          new InstantCommand(()->RobotState.getInstance().coralLineupRunning = true),
          new InstantCommand(()->coralEndEffector.runCoralEndEffector(CORAL_ENDEFFECTOR_SPEED-900), coralEndEffector),
          new InstantCommand(()->funnelIntake.runFunnel(funnelAdjustingSpeed), funnelIntake),
          new InstantCommand(()->System.out.println("reached checkpoint 1")),
          // new WaitCommand(()->0.3),
          new WaitUntil(()->!(RobotState.getInstance().coralEndeffSensorTrig)),
          new InstantCommand(()->coralEndEffector.runCoralEndEffector(-endEffectorAdjustingSpeed/1.2), coralEndEffector),
          new InstantCommand(()->funnelIntake.stopFunnel(), funnelIntake),
          new ParallelRaceGroup(
            new WaitCommand(()->2),
            new WaitUntil(()->RobotState.getInstance().coralEndeffSensorTrig)),
          new InstantCommand(()->System.out.println("stopped end effector by sequnce")),
          new InstantCommand(()->coralEndEffector.stopCoralEndEffector(), coralEndEffector),
          new InstantCommand(()->RobotState.getInstance().coralAligned = true),
          new InstantCommand(()->RobotState.getInstance().coralLineupRunning = false)
        )
      )
    );
  }
}
