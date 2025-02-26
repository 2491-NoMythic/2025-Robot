// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorTestCommand extends Command {
  /** Creates a new ElevatorTestCommand. */
  ElevatorSubsystem elevator;
  BooleanSupplier elevatorUpSup;
  BooleanSupplier elevatorDownSup;
  BooleanSupplier selectedHeightSup;
  public ElevatorTestCommand(ElevatorSubsystem elevator, BooleanSupplier elevatorUp, BooleanSupplier elevatorDown, BooleanSupplier goToSelectedHeight) {
    this.elevator = elevator;
    elevatorUpSup = elevatorUp;
    elevatorDownSup = elevatorDown;
    selectedHeightSup = goToSelectedHeight;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(elevatorUpSup.getAsBoolean()) {
      elevator.setVoltage(2);
    } else if(elevatorDownSup.getAsBoolean()) {
      elevator.setVoltage(-0.5);
    } else if(selectedHeightSup.getAsBoolean()) {
      elevator.setElevatorPosition(RobotState.getInstance().deliveringCoralHeight);
    } else {
      elevator.holdElevatorPose();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setVoltage(0);
  }
  @Override
  public InterruptionBehavior getInterruptionBehavior() {
      return InterruptionBehavior.kCancelIncoming;
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !DriverStation.isTest();
  }
}
