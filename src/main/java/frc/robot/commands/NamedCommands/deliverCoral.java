// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.NamedCommands;

import static frc.robot.settings.Constants.ElevatorConstants.REEF_LEVEL_4_ROTATIONS;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralEndeffectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class deliverCoral extends Command {
  ElevatorSubsystem elevatorSubsystem;
  CoralEndeffectorSubsystem coralEndeffector;


  /** Creates a new deliverCoral. */
  public deliverCoral( CoralEndeffectorSubsystem coralEndeffector, ElevatorSubsystem elevatorSubsystem) {
    addRequirements(elevatorSubsystem, coralEndeffector);
    this.elevatorSubsystem = elevatorSubsystem;
    this.coralEndeffector = coralEndeffector;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevatorSubsystem.setElevatorPosition(REEF_LEVEL_4_ROTATIONS);
    
    coralEndeffector.runCoralEndDefector(REEF_LEVEL_4_ROTATIONS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    coralEndeffector.stopCoralEndDefector();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !RobotState.getInstance().coralSeen;
  }
}
