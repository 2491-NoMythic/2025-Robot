// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import static frc.robot.settings.Constants.FunnelConstants.*;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.MotorLogger;

public class FunnelIntake extends SubsystemBase {
  /** Creates a new Funnelintake. */
  SparkMax intakeMotor1;
  SparkMax intakeMotor2;
  SparkBaseConfig intakeMotor1Config;
  SparkBaseConfig intakeMotor2Config;
  MotorLogger motorLogger1;
  MotorLogger motorLogger2;
  SparkAnalogSensor funnelIntakeSensor;

  
  public FunnelIntake() {
    intakeMotor1 = new SparkMax(FUNNEL_INTAKE_MOTOR_1_ID, MotorType.kBrushless);
    intakeMotor1Config = new SparkMaxConfig();
    intakeMotor1Config.apply(new ClosedLoopConfig().pidf(
      FUNNEL_INTAKE_1_KP,
      FUNNEL_INTAKE_1_KI,
      FUNNEL_INTAKE_1_KD,
      FUNNEL_INTAKE_1_KFF));

    intakeMotor1Config.idleMode(IdleMode.kCoast);
    intakeMotor1Config.smartCurrentLimit(25, 25, 1000);
    intakeMotor1.configure(intakeMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeMotor2 = new SparkMax(FUNNEL_INTAKE_MOTOR_2_ID,MotorType.kBrushless);
    intakeMotor2Config = new SparkMaxConfig();
    intakeMotor2Config.apply(new ClosedLoopConfig().pidf(
      FUNNEL_INTAKE_2_KP,
      FUNNEL_INTAKE_2_KI,
      FUNNEL_INTAKE_2_KD,
      FUNNEL_INTAKE_2_KFF));

    intakeMotor2Config.idleMode(IdleMode.kCoast);
    intakeMotor2Config.smartCurrentLimit(25, 25, 1000);
    intakeMotor2.configure(intakeMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    motorLogger1 = new MotorLogger("/funnelIntake/intakemotor1");
    motorLogger2 = new MotorLogger("/funnelIntake/intakeMotor2");
    
    funnelIntakeSensor = intakeMotor1.getAnalog();

  }
  private void logMotors(){
    motorLogger1.log(intakeMotor1);
    motorLogger2.log(intakeMotor2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    RobotState.getInstance().funnelSensorTrig = funnelIntakeSensor.getVoltage()>2;
    if(Preferences.getBoolean("Motor Logging", false)){
    logMotors();
    }
  }
  public void runFunnel(double speed){
    intakeMotor1.getClosedLoopController().setReference(speed, ControlType.kVelocity);
    intakeMotor2.getClosedLoopController().setReference(speed, ControlType.kVelocity);
  }

  public void stopFunnel() {
    intakeMotor1.set(0);
    intakeMotor2.set(0);
  }

}
