// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.settings.Constants.CoralEndeffectorConstants.CORAL_ENDEFFECTOR_ADJUSTING_INTAKE_SPEED;
import static frc.robot.settings.Constants.FunnelConstants.FUNNEL_ADJUSTING_INTAKE_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEndeffectorSubsystem;
import frc.robot.subsystems.FunnelIntake;
import frc.robot.subsystems.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PassCoralToEndEffector extends Command {
  CoralEndeffectorSubsystem coralEndEffector;
  FunnelIntake funnelIntake;
  /** This command should be run when the coral is detected by the funnel, but not the coral endeffector, and the elevator is in the right place for the pass off. */
  public PassCoralToEndEffector(CoralEndeffectorSubsystem coralEndEffector, FunnelIntake funnelIntake) {
    this.funnelIntake = funnelIntake;
    this.coralEndEffector = coralEndEffector;
    addRequirements(funnelIntake, coralEndEffector);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotState.getInstance().coralLineupRunning = true;
    funnelIntake.runFunnel(FUNNEL_ADJUSTING_INTAKE_SPEED);
    coralEndEffector.runCoralEndEffector(CORAL_ENDEFFECTOR_ADJUSTING_INTAKE_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotState.getInstance().coralLineupRunning = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotState.getInstance().coralEndeffSensorTrig;
  }
}
