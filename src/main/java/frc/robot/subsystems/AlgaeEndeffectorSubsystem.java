// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.settings.Constants.AlgaeEndeffectorConstants.*;
import edu.wpi.first.math.controller.PIDController;
public class AlgaeEndeffectorSubsystem extends SubsystemBase {
  TalonFX algaeEndeffectorMotor;
  PIDController algendController;
  public boolean powerSpike;
  /** Creates a new AlgaeEndDefectorSubsystem. */
  public AlgaeEndeffectorSubsystem() {
    algaeEndeffectorMotor = new TalonFX(ALGAE_ENDEFFECTOR_MOTOR_1_ID);
    algaeEndeffectorMotor.getConfigurator().apply(AlgaeEndeffectorConfig);
  }
  public boolean powerCheck(){
    if (algaeEndeffectorMotor.getSupplyCurrent().getValueAsDouble() >= 95){
      powerSpike = true;
    } else {
      powerSpike = false;
    }
    return powerSpike;
  }
  public void runAlgaeEndDefector(double speed){
    algaeEndeffectorMotor.set(speed);

  }
  public void stopAlgaeEndDefector(){
    algaeEndeffectorMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    powerCheck();
  }
}
