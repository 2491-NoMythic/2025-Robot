// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.LightsEnums;
import frc.robot.subsystems.Lights;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class blinkWhiteLights extends Command {
  /** Creates a new blinkWhiteLights. */
  Lights lights;
  Timer timer;
  public blinkWhiteLights(Lights lights) {
    this.lights = lights;
    timer = new Timer();
    addRequirements(lights);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    lights.blinkLights(LightsEnums.Drivetrain, 100, 100, 100);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lights.stopBlinkingLights();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 0.25;
  }
}
