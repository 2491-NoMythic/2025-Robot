// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.settings.Constants.ElevatorConstants;
import frc.robot.settings.ElevatorEnums;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommand extends Command {
  ElevatorSubsystem elevator;
  ElevatorEnums level;
  /** Creates a new ElevatorCommand. 
   * @param elevator elevator subsystem
   * @param level level of reef, human player station is 0. 
  */
  public ElevatorCommand(ElevatorSubsystem elevator, ElevatorEnums level) {
    this.elevator = elevator;
    this.level = level;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(level){
      case HumanPlayer:
        elevator.setElevatorPosition(ElevatorConstants.HUMAN_PLAYER_STATION_MILLIMETERS);
      case Reef1:
        elevator.setElevatorPosition(ElevatorConstants.REEF_LEVEL_1_MILLIMETERS);
      case Reef2:
        elevator.setElevatorPosition(ElevatorConstants.REEF_LEVEL_2_MILLIMETERS);
      case Reef3:
        elevator.setElevatorPosition(ElevatorConstants.REEF_LEVEL_3_MILLIMETERS);
      case Reef4:
        elevator.setElevatorPosition(ElevatorConstants.REEF_LEVEL_4_MILLIMETERS);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopElevator();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
