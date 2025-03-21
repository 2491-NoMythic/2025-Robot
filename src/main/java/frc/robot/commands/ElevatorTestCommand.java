// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.settings.Constants.ElevatorConstants.BARGE_SHOOT_CENTIMETERS;
import static frc.robot.settings.Constants.ElevatorConstants.REEF_LEVEL_1_CENTIMETERS;
import static frc.robot.settings.Constants.ElevatorConstants.REEF_LEVEL_2_CENTIMETERS;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.ElevatorEnums;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorTestCommand extends Command {
  /** Creates a new ElevatorTestCommand. */
  ElevatorSubsystem elevator;
  BooleanSupplier selectedHeightSup;
  final double TEST_ACCELERATION = 200;
  final double TEST_VELOCITY = 100;
  public ElevatorTestCommand(ElevatorSubsystem elevator, BooleanSupplier goToSelectedHeight) {
    this.elevator = elevator;
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
    if(selectedHeightSup.getAsBoolean()) {
      elevator.setElevatorPosition(RobotState.getInstance().deliveringCoralHeight);
      if(RobotState.getInstance().deliveringCoralHeight == ElevatorEnums.Reef1){
        elevator.setElevatorPositionDynamicConfigs(REEF_LEVEL_1_CENTIMETERS_AGAINST_REEF, TEST_ACCELERATION, TEST_VELOCITY, 0);
      }
      if(RobotState.getInstance().deliveringCoralHeight == ElevatorEnums.Reef2){
        elevator.setElevatorPositionDynamicConfigs(REEF_LEVEL_2_CENTIMETERS_AGAINST_REEF, TEST_ACCELERATION, TEST_VELOCITY, 0);
      }
      if(RobotState.getInstance().deliveringCoralHeight == ElevatorEnums.Reef3){
        elevator.setElevatorPositionDynamicConfigs(REEF_LEVEL_3_CENTIMETERS_AGAINST_REEF, TEST_ACCELERATION, TEST_VELOCITY, 0);
      }
      if(RobotState.getInstance().deliveringCoralHeight == ElevatorEnums.Reef4){
        elevator.setElevatorPositionDynamicConfigs(REEF_LEVEL_4_CENTIMETERS_AGAINST_REEF, TEST_ACCELERATION, TEST_VELOCITY, 0);
      }
      if(RobotState.getInstance().deliveringCoralHeight == ElevatorEnums.Barge){
        elevator.setElevatorPositionDynamicConfigs(BARGE_SHOOT_CENTIMETERS, TEST_ACCELERATION, TEST_VELOCITY, 0);
      }
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
