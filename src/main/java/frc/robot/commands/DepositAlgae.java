// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.settings.Constants.ElevatorConstants.HUMAN_PLAYER_STATION_CENTIMETERS;
import static frc.robot.settings.Constants.ElevatorConstants.MOTION_MAGIC_ELEVATOR_HP_ACCLERATION;
import static frc.robot.settings.Constants.ElevatorConstants.MOTION_MAGIC_ELEVATOR_HP_VELOCITY;
import static frc.robot.settings.Constants.ElevatorConstants.MOTION_MAGIC_ELEVATOR_JERK;
import static frc.robot.settings.Constants.ElevatorConstants.PROCESSOR_HEIGHT_CENTIMETERS;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeEndeffectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;


public class DepositAlgae extends Command {
  AlgaeEndeffectorSubsystem algaeEndeffector;
  ElevatorSubsystem elevator;
  double shootSpeed;
  
  /**
   * Aligns the elevator with the processor, dumps collected algae, then aligns elevator with the human player station. 
   * @param algaeEndeffector
   * @param elevator
   * @param shootSpeed
  */
  public DepositAlgae(AlgaeEndeffectorSubsystem algaeEndeffector, ElevatorSubsystem elevator, double shootSpeed) {
    this.algaeEndeffector = algaeEndeffector;
    this.elevator = elevator;
    this.shootSpeed = shootSpeed;
   
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(algaeEndeffector);
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setElevatorPositionDynamicConfigs(PROCESSOR_HEIGHT_CENTIMETERS, MOTION_MAGIC_ELEVATOR_HP_ACCLERATION, MOTION_MAGIC_ELEVATOR_HP_VELOCITY, MOTION_MAGIC_ELEVATOR_JERK);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(elevator.isElevatorAtPose()){
      algaeEndeffector.runAlgaeEndDefector(shootSpeed);
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeEndeffector.stopAlgaeEndDefectorCoast();
    elevator.setElevatorPositionDynamicConfigs(HUMAN_PLAYER_STATION_CENTIMETERS, MOTION_MAGIC_ELEVATOR_HP_ACCLERATION, MOTION_MAGIC_ELEVATOR_HP_VELOCITY, MOTION_MAGIC_ELEVATOR_JERK);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
}
}
