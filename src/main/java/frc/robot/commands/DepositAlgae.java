// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.settings.Constants.ElevatorConstants.PROCESSOR_HEIGHT_MILLIMETERS;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeEndeffectorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RobotState;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DepositAlgae extends Command {
  AlgaeEndeffectorSubsystem algaeEndeffector;
  ElevatorSubsystem elevator;
  double shootSpeed;
  
  /** Creates a new AlgaeIntakeCommand. */
  public DepositAlgae(AlgaeEndeffectorSubsystem algaeEndeffector, ElevatorSubsystem elevator, double shootSpeed) {
    this.algaeEndeffector = algaeEndeffector;
    this.elevator = elevator;
    this.shootSpeed = shootSpeed;
   
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setElevatorPosition(PROCESSOR_HEIGHT_MILLIMETERS);
    elevator.isElevatorAtPose();
    if(elevator.isElevatorAtPose()){
      algaeEndeffector.runAlgaeEndDefector(shootSpeed);
  }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeEndeffector.stopAlgaeEndDefector();
    elevator.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
}
}