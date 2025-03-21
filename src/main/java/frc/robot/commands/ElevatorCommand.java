// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import frc.robot.subsystems.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.settings.ElevatorEnums;


public class ElevatorCommand extends Command {
  ElevatorSubsystem elevator;
  ElevatorEnums level;
  Supplier<ElevatorEnums> levelSupplier;
  /** Sets the elevator to a supplied position. 
   * @param elevator elevator subsystem
   * @param level level of reef, human player station is 0. 
  */
  public ElevatorCommand(ElevatorSubsystem elevator, Supplier<ElevatorEnums> levelSupplier) {
    this.elevator = elevator;
    this.levelSupplier = levelSupplier;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(RobotState.getInstance().goForAlgae) {
      elevator.setElevatorPositionWithAlgae(levelSupplier.get());
    } else {
      elevator.setElevatorPosition(levelSupplier.get());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
