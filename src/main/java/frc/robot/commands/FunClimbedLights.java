// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.Constants.LightConstants;
import frc.robot.subsystems.Lights;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FunClimbedLights extends Command {
  Timer timer;
  Lights lights;
  /** Creates a new FunClimbedLights. */
  public FunClimbedLights(Lights lights) {
    this.lights = lights;
    addRequirements(lights);
    timer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int rioStart = (int) (timer.get() / 0.2) % LightConstants.TOTAL_LIGHTS_RIO_STRIP_END;
    int candleStart = (int) (timer.get() / 0.2) % LightConstants.TOTAL_LIGHTS_CANDLE_STRIP_END;
    lights.setAllLights(100, 0, 150);
    lights.setCandleLights(candleStart, candleStart+3, 200, 150, 200);
    lights.setLights(rioStart, rioStart+3, 200, 150, 200);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
