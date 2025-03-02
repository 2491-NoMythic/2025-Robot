// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorReset extends Command {
  /** Creates a new ElevatorReset. */
  ElevatorSubsystem elevator;
  public ElevatorReset(ElevatorSubsystem elevator) {
    this.elevator = elevator;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setVoltage(-0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.limitSwitchTrig();
  }
}
