// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AntiTippingCommand extends Command {
  /** Creates a new AntiTippingCommand. */
  DrivetrainSubsystem drivetrainSub;
  double currentPitch;
  double currentRoll;
  ChassisSpeeds currentSpeeds;


  public AntiTippingCommand(DrivetrainSubsystem drivetrainSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrainSub = drivetrainSub;
    this.currentPitch = currentPitch;
    this.currentRoll = currentRoll;
    this.currentSpeeds = currentSpeeds;
    addRequirements(drivetrainSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentPitch = drivetrainSub.getPigeonPitch();
    currentRoll = drivetrainSub.getPigeonRoll();
    currentSpeeds = drivetrainSub.getChassisSpeeds();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(currentPitch) >= Math.abs(currentRoll)){
      if(currentPitch > 1){
      drivetrainSub.drive(
        new ChassisSpeeds(Math.sqrt(currentPitch),
       currentSpeeds.vyMetersPerSecond,
        currentSpeeds.omegaRadiansPerSecond));
    }
    else if (currentPitch < -1){
      drivetrainSub.drive(
        new ChassisSpeeds(-(Math.sqrt(Math.abs(currentPitch))),
       currentSpeeds.vyMetersPerSecond,
        currentSpeeds.omegaRadiansPerSecond));
    }
    else{
      if(currentRoll > 1){
        drivetrainSub.drive(
          new ChassisSpeeds(currentSpeeds.vxMetersPerSecond,
          Math.sqrt(currentRoll),
          currentSpeeds.omegaRadiansPerSecond));
      }
      else if (currentRoll < -1){
        drivetrainSub.drive(
          new ChassisSpeeds(currentSpeeds.vxMetersPerSecond,
         -(Math.sqrt(Math.abs(currentRoll))),
          currentSpeeds.omegaRadiansPerSecond));
      }
    }
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
