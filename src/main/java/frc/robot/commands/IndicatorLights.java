// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.settings.ElevatorEnums;
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
      if(RobotState.getInstance().funnelSensorTrig) {
        lights.setFunnel(0, 0, 0);
        //TODO: adjust values plz
      }
      else if(RobotState.getInstance().coralEndeffSensorTrig) {
        lights.setFunnel(0, 0, 0);
      }
      else{
        lights.setFunnel(0, 0, 0);//Turn off
      }

      if(RobotState.getInstance().deliveringCoralHeight == ElevatorEnums.Reef4){
        lights.setLights(LightConstants.HEIGHT4_START, LightConstants.HEIGHT4_END, 24, 0, 0);
      }
      else if(RobotState.getInstance().deliveringCoralHeight == ElevatorEnums.Reef3){
        lights.setLights(LightConstants.HEIGHT3_START, LightConstants.HEIGHT3_END, 24, 0, 0);
      }
      else if(RobotState.getInstance().deliveringCoralHeight == ElevatorEnums.Reef2){
        lights.setLights(LightConstants.HEIGHT2_START, LightConstants.HEIGHT2_END, 24, 0, 0);
      }
      else if(RobotState.getInstance().deliveringCoralHeight == ElevatorEnums.Reef1){
        lights.setLights(LightConstants.HEIGHT1_START, LightConstants.HEIGHT1_END, 24, 0, 0);
      }

      if(RobotState.getInstance().deliveringLeft) {
        lights.setLights(LightConstants.ELEVATOR_L_TOP_START, LightConstants.ELEVATOR_L_TOP_END, 0, 0, 0);
        //TODO adjust values :3
      } else {
        lights.setLights(LightConstants.ELEVATOR_R_TOP_START, LightConstants.ELEVATOR_R_TOP_END, 0, 0, 0);
      }

       if (RobotState.getInstance().odometerUpdated){
        lights.setDrivetrain(24, 9, 1);
       }

       if (RobotState.getInstance().pathFinding){
        lights.setLights(LightConstants.DRIVETRAIN_LIGHTS_START, LightConstants.DRIVETRAIN_LIGHTS_END, 0, 0, 0);
       }
       else if (RobotState.getInstance().sensorApproach){
        RobotState.getInstance().pathFinding = false;
        lights.setLights(LightConstants.DRIVETRAIN_LIGHTS_START, LightConstants.DRIVETRAIN_LIGHTS_END, 1, 1, 1);
       }
       else if ( RobotState.getInstance().sensorLineUp){
        RobotState.getInstance().pathFinding = false;
        lights.setLights(LightConstants.DRIVETRAIN_LIGHTS_START, LightConstants.DRIVETRAIN_LIGHTS_END, 0, 0, 0);
       }
        else if (RobotState.getInstance().raiseElevator){
        RobotState.getInstance().pathFinding = false;
        lights.setLights(LightConstants.DRIVETRAIN_LIGHTS_START, LightConstants.DRIVETRAIN_LIGHTS_END, 0, 0, 0);
       }
       else if(RobotState.getInstance().bargeLineUp){
        RobotState.getInstance().pathFinding = false;
        lights.setLights(LightConstants.DRIVETRAIN_LIGHTS_START, LightConstants.DRIVETRAIN_LIGHTS_END, 0, 0, 0);
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
