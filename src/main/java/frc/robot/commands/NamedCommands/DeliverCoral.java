// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NamedCommands;

import static frc.robot.settings.Constants.CoralEndeffectorConstants.CORAL_ENDEFFECTOR_SPEED;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.ElevatorEnums;
import frc.robot.subsystems.CoralEndeffectorSubsystem;
import frc.robot.subsystems.RobotState;

public class DeliverCoral extends Command {
  /** Creates a new deliverCoral. */
  CoralEndeffectorSubsystem coralEndeffector;

  /**
   * Runs the coral endeffector. Use this one for standard delivery.
   * @param coralEndeffector
   */
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
    if(RobotState.getInstance().deliveringCoralHeight == ElevatorEnums.Reef2 || RobotState.getInstance().deliveringCoralHeight == ElevatorEnums.Reef3) {
      coralEndeffector.runCoralEndEffector(CORAL_ENDEFFECTOR_SPEED/1.4);
    } else {
      coralEndeffector.runCoralEndEffector(CORAL_ENDEFFECTOR_SPEED);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralEndeffector.stopCoralEndEffector();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
