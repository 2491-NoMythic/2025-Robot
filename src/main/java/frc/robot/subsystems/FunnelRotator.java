// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import static frc.robot.settings.Constants.FunnelConstants.*;

public class FunnelRotator extends SubsystemBase {
  private TalonFX rotatorMotor;
  private TalonFXConfiguration rotatorMotorConfig;
  /** Creates a new FunnelRotator. */
  public FunnelRotator() {
    rotatorMotor = new TalonFX(FUNNEL_ROTATOR_MOTOR_ID);
    rotatorMotor.getConfigurator().apply(FunnelRotatorConfig);
    rotatorMotorConfig = new TalonFXConfiguration().withSlot0(new Slot0Configs()
    .withKP(FUNNEL_ROTATOR_KP)
    .withKS(FUNNEL_ROTATOR_KS)
    .withKA(FUNNEL_ROTATOR_KA)
    .withKV(FUNNEL_ROTATOR_KV)) 
    .withCurrentLimits(new CurrentLimitsConfigs()
    .withSupplyCurrentLimit(FUNNEL_ROTATOR_SUPPLY_CURRENT_LIMIT)
    .withSupplyCurrentLimitEnable(true));
    
    
  }

  public void setFunnelRotation(double rotation) {
    double position = rotation * FUNNEL_ROTATOR_GEAR_RATIO;
    PositionVoltage voltReq = new PositionVoltage(0);
    rotatorMotor.setControl(voltReq.withPosition(position)); 
  }
  public void stopRotating(){
    rotatorMotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
