// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.settings.Constants.FunnelConstants.*;

public class FunnelRotator extends SubsystemBase {

  private SparkBaseConfig rotatorMotorConfig;
  SparkMax rotatorMotor; 
  /** Creates a new FunnelRotator. */
  public FunnelRotator() {
    rotatorMotor = new SparkMax (FUNNEL_ROTATOR_MOTOR_ID, MotorType.kBrushless);
    rotatorMotorConfig = new SparkMaxConfig();
    rotatorMotorConfig.apply(new ClosedLoopConfig().pidf(
      FUNNEL_ROTATOR_KP,
      FUNNEL_ROTATOR_KI,
      FUNNEL_ROTATOR_KD,
      FUNNEL_ROTATOR_KFF));
    rotatorMotorConfig.idleMode(IdleMode.kCoast);
    rotatorMotorConfig.smartCurrentLimit(25, 25, 1000);
    rotatorMotor.configure(rotatorMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setFunnelRotator(double speed) {
    rotatorMotor.set(speed);
  }
  public void stopRotating(){
    rotatorMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
