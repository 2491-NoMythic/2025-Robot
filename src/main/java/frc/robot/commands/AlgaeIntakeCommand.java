// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.settings.Constants.AlgaeEndeffectorConstants.ALGAE_INTAKE_SPEED;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeEndeffectorSubsystem;
import frc.robot.subsystems.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeIntakeCommand extends Command {
  AlgaeEndeffectorSubsystem algaeEndeffector;
  DoubleSupplier shootSpeed;
  boolean algaeDetected;
  boolean goForAlagea;
  /** Creates a new AlgaeIntakeCommand. */
  public AlgaeIntakeCommand(AlgaeEndeffectorSubsystem algaeEndeffector, DoubleSupplier shootSpeed) {
    this.algaeEndeffector = algaeEndeffector;
    this.shootSpeed = shootSpeed;
    addRequirements(algaeEndeffector);
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algaeDetected = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // This command is meant to be used with a parallel race group. 
    // if we returned true from isFinished we would cancel the rest of the commands.
    // so instead handle finishing internally.
    if (RobotState.getInstance().hasAlgae) {
      algaeDetected = true;
    }
    if (algaeDetected) {
      algaeEndeffector.stopAlgaeEndDefector();
    }
    if (goForAlagea = false) {
      algaeEndeffector.stopAlgaeEndDefector();
    }
    else {
      algaeEndeffector.runAlgaeEndDefector(shootSpeed.getAsDouble());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    algaeEndeffector.stopAlgaeEndDefector();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
    }
}
