// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NamedCommands;

import static frc.robot.settings.Constants.ElevatorConstants.HUMAN_PLAYER_STATION_ROTATIONS;

import frc.robot.subsystems.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.CoralIntakeCommand;
import frc.robot.commands.ElevatorCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelIntake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralIntake extends Command {
  /** Creates a new CoralIntake. */
  ElevatorSubsystem elevatorSubsystem;
  FunnelIntake funnelIntake;
  RobotState robotState;
  
  
  public CoralIntake(ElevatorSubsystem elevatorSubsystem, FunnelIntake funnelIntake, RobotState robotState) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevatorSubsystem);
    this.elevatorSubsystem = elevatorSubsystem;
    this.funnelIntake = funnelIntake;
    this.robotState = robotState;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setElevatorPosition(HUMAN_PLAYER_STATION_ROTATIONS);
    
    funnelIntake.runFunnel();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    funnelIntake.stopFunnel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return robotState.getInstane().coralSeen;
  }
}
