// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IndicatorLights extends Command {
  Lights lights;
  /** Creates a new IndicatorLights. */
  public IndicatorLights(Lights lights) {
    this.lights = lights;
    addRequirements(lights);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(RobotState.getInstance().coralSeen) {
        lights.setBL(2,1,4);
        lights.setBR(2,1,4);
        //TODO adjust values plz
      }
      if(RobotState.getInstance().hasAlgae) {
        lights.setFL(2,1,4);
        lights.setFR(2,1,4);
        //TODO adjust values :3
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
