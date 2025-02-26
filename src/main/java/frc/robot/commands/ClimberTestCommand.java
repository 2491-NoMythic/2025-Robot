// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimberTestCommand extends Command {
  ClimberSubsystem climber;
  BooleanSupplier climberUp;
  BooleanSupplier climberDown;

  /** Creates a new ClimberTestCommand. */
  public ClimberTestCommand(ClimberSubsystem climber, BooleanSupplier climberUp, BooleanSupplier climberDown) {
    this.climber = climber;
    addRequirements(climber);
    this.climberDown = climberDown;
    this.climberUp = climberUp;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(climberDown.getAsBoolean()){
      climber.setClimberPower(0.2);
    }else if (climberUp.getAsBoolean()) {
      climber.setClimberPower(-0.2);
    }else{
      climber.setClimberPower(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setClimberPower(0);
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
