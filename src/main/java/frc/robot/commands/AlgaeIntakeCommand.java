// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeEndeffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeIntakeCommand extends Command {
  AlgaeEndeffectorSubsystem algaeEndeffector;
  /** Creates a new AlgaeIntakeCommand. */
  public AlgaeIntakeCommand(AlgaeEndeffectorSubsystem algaeEndeffector) {
    this.algaeEndeffector = algaeEndeffector;
    addRequirements(algaeEndeffector);
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    algaeEndeffector.runAlgaeEndDefector(1);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeEndeffector.stopAlgaeEndDefector();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (algaeEndeffector.powerSpike == true){
      return true;
    } else {
      return false;
  }
}
}