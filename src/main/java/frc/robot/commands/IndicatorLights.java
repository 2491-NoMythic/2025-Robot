// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.ElevatorEnums;
import frc.robot.settings.LightsEnums;
import frc.robot.settings.Constants.LightConstants;
import frc.robot.subsystems.DrivetrainSubsystem;
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
  //sets lights on the funnel to report on coral status in the robot, or indicate if we are lining up to the reef
    if(RobotState.getInstance().reefLineupRunning) {
      lights.setSystemLights(LightsEnums.Funnel, 100, 50, 50);
    }else if(RobotState.getInstance().coralEndeffSensorTrig) {
      lights.setSystemLights(LightsEnums.Funnel, 0, 100, 0);
      //TODO: adjust values plz
    } else if(RobotState.getInstance().funnelSensorTrig) {
      lights.setSystemLights(LightsEnums.Funnel, 100, 100, 100);
    } else if(RobotState.getInstance().inIntakeZone) {
      lights.blinkLights(LightsEnums.Funnel, 0, 100, 0);
    } else {
      lights.stopBlinkingLights();
      lights.setSystemLights(LightsEnums.Funnel, 0, 0, 0);//Turn off
    }
    //updates lights on sides of elevator to indicate our selected coral delivery level
    lights.setElevatorLevel(RobotState.getInstance().deliveringCoralHeight);
    //updates top of elevator lights to indicate drop off side
    if(RobotState.getInstance().deliveringLeft) {
      lights.setSystemLights(LightsEnums.ElevatorLeft5, 200, 0, 200);
      lights.setSystemLights(LightsEnums.ElevatorRight5, 0, 0, 0);
      //TODO adjust values :3
    } else {
      lights.setSystemLights(LightsEnums.ElevatorRight5, 200, 0, 200);
      lights.setSystemLights(LightsEnums.ElevatorLeft5, 0, 0, 0);
    }
    //updates lights on sides of elevator to indicate if we are planning to pick up algae or not
    if(RobotState.getInstance().goForAlgae) {
      lights.setSystemLights(LightsEnums.ElevatorLeftAlgaeIndicator, 0, 0, 100);
      lights.setSystemLights(LightsEnums.ElevatorRightAlgaeIndicator, 0, 0, 100);
    } else {
      lights.setSystemLights(LightsEnums.ElevatorLeftAlgaeIndicator, 0, 0, 0);
      lights.setSystemLights(LightsEnums.ElevatorRightAlgaeIndicator, 0, 0, 0);
    }
    //updates drivetrain lights
    if(RobotState.getInstance().bargeLineUp){
     lights.setSystemLights(LightsEnums.Drivetrain, 100, 50, 50);
    } else if (RobotState.getInstance().LimelightsUpdated){
      lights.setSystemLights(LightsEnums.Drivetrain, 0, 100, 0);
    } else {
      lights.setSystemLights(LightsEnums.Drivetrain, 100, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lights.lightsOut();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
