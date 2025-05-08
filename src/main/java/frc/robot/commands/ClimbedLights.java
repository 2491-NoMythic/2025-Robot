
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.LightsEnums;
import frc.robot.settings.Constants.LightConstants;
import frc.robot.subsystems.Lights;
import static frc.robot.settings.Constants.LightConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbedLights extends Command {
  /** Creates a new ClimbedLights. */
  Lights lights;
  Timer timer;
  public ClimbedLights(Lights lights) {
    timer = new Timer();
    this.lights = lights;
    addRequirements(lights);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double timer1 = timer.get() * 220;
    double timer2 = timer1 + 75;
    int R1 = (int) timer1 % 150;
    int G1 = (int) (timer1 % 150) / 5;
    int B1 = (int) timer1 % 150;
    int R2 = (int) timer2 % 150;
    int G2 = (int) (timer2 % 150) / 5;
    int B2 = (int) timer2 % 150;
    lights.setSystemLights(LightsEnums.ElevatorLeft1, R1, G1, B1);
    lights.setSystemLights(LightsEnums.ElevatorLeft3, R1, G1, B1);
    lights.setSystemLights(LightsEnums.ElevatorRight2, R1, G1, B1);
    lights.setSystemLights(LightsEnums.ElevatorRight4, R1, G1, B1);
    lights.setCandleLights(DRIVETRAIN_LIGHTS_START, 68, R1, G1, B1);
    lights.setSystemLights(LightsEnums.ElevatorRightAlgaeIndicator, R1, G1, B1);
    
    lights.setSystemLights(LightsEnums.ElevatorRight1, R2, G2, B2);
    lights.setSystemLights(LightsEnums.ElevatorRight3, R2, G2, B2);
    lights.setSystemLights(LightsEnums.ElevatorLeft2, R2, G2, B2);
    lights.setSystemLights(LightsEnums.ElevatorLeft4, R2, G2, B2);
    lights.setCandleLights(68, DRIVETRAIN_LIGHTS_END, R2, G2, B2);
    lights.setSystemLights(LightsEnums.ElevatorLeftAlgaeIndicator, R2, G2, B2);
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
