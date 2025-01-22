// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NamedCommands;

import static frc.robot.settings.Constants.CoralEndeffectorConstants.CORAL_ENDEFFECTOR_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEndeffectorSubsystem;
import frc.robot.subsystems.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeliverCoral extends Command {
  CoralEndeffectorSubsystem coralEndeffector;


  /** Creates a new deliverCoral. */
  public DeliverCoral( CoralEndeffectorSubsystem coralEndeffector) {
    addRequirements(coralEndeffector);
    this.coralEndeffector = coralEndeffector;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralEndeffector.runCoralEndEffector(CORAL_ENDEFFECTOR_SPEED, CORAL_ENDEFFECTOR_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralEndeffector.stopCoralEndEffector();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotState.getInstance().coralSeen;
  }
}
