// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSystem extends SubsystemBase {
  SparkMax motorIn;

  /** Creates a new IntakeSystem. */
  public IntakeSystem() {
    motorIn = new SparkMax(0, MotorType.kBrushless);
  }
  public void Run(double run) {
    motorIn.set(run);
  }
  public void Off() {
    motorIn.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
