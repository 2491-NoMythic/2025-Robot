// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NamedCommands;

import static frc.robot.settings.Constants.CoralEndeffectorConstants.CORAL_ENDEFFECTOR_SPEED;
import static frc.robot.settings.Constants.FunnelConstants.FUNNEL_INTAKE_SPEED;

import frc.robot.subsystems.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.ElevatorEnums;
import frc.robot.subsystems.CoralEndeffectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelIntake;

public class CoralIntake extends Command {
  /** Creates a new CoralIntake. */
  ElevatorSubsystem elevatorSubsystem;
  FunnelIntake funnelIntake;  
  CoralEndeffectorSubsystem coralIntake;
  /**
   * Sets the elevator position to the Human Player station, then runs the coral endeffector and funnel until the distance sensor detects the coral.
   * @param elevatorSubsystem
   * @param funnelIntake
   * @param coralIntake
   */
  public CoralIntake(ElevatorSubsystem elevatorSubsystem, FunnelIntake funnelIntake, CoralEndeffectorSubsystem coralIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem, funnelIntake, coralIntake);
    this.elevatorSubsystem = elevatorSubsystem;
    this.funnelIntake = funnelIntake;
    this.coralIntake = coralIntake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.setElevatorPosition(ElevatorEnums.HumanPlayer);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(elevatorSubsystem.isElevatorAtIntakeHeight()) {
      coralIntake.runCoralEndEffector(CORAL_ENDEFFECTOR_SPEED);
      funnelIntake.runFunnelSine();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    funnelIntake.stopFunnel();
    coralIntake.stopCoralEndEffector();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotState.getInstance().isCoralSeen();
  }
}
