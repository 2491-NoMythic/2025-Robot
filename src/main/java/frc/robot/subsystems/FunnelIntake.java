// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.settings.Constants.FunnelConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FunnelIntake extends SubsystemBase {
  /** Creates a new Funnelintake. */
  SparkMax intakeMotor;
  SparkBaseConfig intakeMotorConfig;
  public FunnelIntake() {
    intakeMotor = new SparkMax(FUNNEL_INTAKE_MOTOR_ID, MotorType.kBrushless);
    intakeMotorConfig = new SparkMaxConfig();
    intakeMotorConfig.apply(new ClosedLoopConfig().pidf(
      FUNNEL_INTAKE_KP,
      FUNNEL_INTAKE_KI,
      FUNNEL_INTAKE_KD,
      FUNNEL_INTAKE_KFF));

    intakeMotorConfig.idleMode(IdleMode.kCoast);
    intakeMotorConfig.smartCurrentLimit(25, 25, 1000);
    intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
  public void runFunnel(double speed){
    intakeMotor.set(speed);
  }
  public void stopFunnel() {
    intakeMotor.set(0);
  }
}
