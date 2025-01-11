// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.AlgaeEndeffectorConstants;
import edu.wpi.first.math.controller.PIDController;
public class AlgaeEndeffectorSubsystem extends SubsystemBase {
  TalonFX algaeEndDefectorMotor;
  PIDController algendController;
  /** Creates a new AlgaeEndDefectorSubsystem. */
  public AlgaeEndeffectorSubsystem() {
    algaeEndDefectorMotor = new TalonFX(AlgaeEndeffectorConstants.ALGAE_ENDEFFECTOR_MOTOR_1_ID);
    algendController = new PIDController(AlgaeEndeffectorConstants.ALGAE_ENDEFFECTOR_KP, AlgaeEndeffectorConstants.ALGAE_ENDEFFECTOR_KI, AlgaeEndeffectorConstants.ALGAE_ENDEFFECTOR_KD);
  }

  public void runAlgaeEndDefector(double speed){
    algaeEndDefectorMotor.set(speed);

  }
  public void stopAlgaeEndDefector(){
    algaeEndDefectorMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
