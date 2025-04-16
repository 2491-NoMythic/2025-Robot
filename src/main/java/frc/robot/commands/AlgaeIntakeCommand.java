// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeEndeffectorSubsystem;
import frc.robot.subsystems.RobotState;

public class AlgaeIntakeCommand extends Command {
  AlgaeEndeffectorSubsystem algaeEndeffector;
  DoubleSupplier shootSpeed;
  boolean algaeDetected;
  Timer timer;
  boolean hasAlgaeInternal;
  /**
   * Runs the algae endeffector to intake algae. It should hard-stop when the algae is in. 
   * @param algaeEndeffector
   * @param shootSpeed
   */
  public AlgaeIntakeCommand(AlgaeEndeffectorSubsystem algaeEndeffector, DoubleSupplier shootSpeed) {
    this.algaeEndeffector = algaeEndeffector;
    this.shootSpeed = shootSpeed;
    timer = new Timer();
    addRequirements(algaeEndeffector);
    // Use addRequirements() here to declare subsystem dependencies.

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    algaeDetected = false;
    hasAlgaeInternal = false;
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // This command is meant to be used with a parallel race group. 
    // if we returned true from isFinished we would cancel the rest of the commands.
    // so instead handle finishing internally.
    if (hasAlgaeInternal && shootSpeed.getAsDouble() >= 0) {
      algaeEndeffector.stopAlgaeEndDefectorHard();
    } else {
      algaeEndeffector.runAlgaeEndDefector(shootSpeed.getAsDouble());
    }
    if(RobotState.getInstance().hasAlgae) {
      hasAlgaeInternal = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(shootSpeed.getAsDouble() >= 0) {
      algaeEndeffector.stopAlgaeEndDefectorHard();
    } else {
      algaeEndeffector.stopAlgaeEndDefectorCoast();
    }
    timer.stop();
    timer.reset();
    hasAlgaeInternal = false;
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return false;
    }
}
