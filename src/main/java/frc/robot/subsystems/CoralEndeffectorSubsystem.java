// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.MotorLogger;

import static frc.robot.settings.Constants.CoralEndeffectorConstants.*;

public class CoralEndeffectorSubsystem extends SubsystemBase {
  SparkMax coralEndeffectorMotor;
  SparkBaseConfig coralConfig;
  SparkAnalogSensor coralEndeffSensor;
  MotorLogger motorLogger;

  /** Creates a new CoralEndDefectorSubsystem. */
  public CoralEndeffectorSubsystem() {
    coralEndeffectorMotor = new SparkMax(CORAL_ENDEFFECTOR_MOTOR, MotorType.kBrushless);
    coralConfig = new SparkMaxConfig();
    if(Preferences.getBoolean("CompBot", true)){
      coralConfig.apply(new ClosedLoopConfig().pidf(
        CORAL_ENDEFFECTOR_KP,
        CORAL_ENDEFFECTOR_KI,
        CORAL_ENDEFFECTOR_KD,
        CORAL_ENDEFFECTOR_KFF));
    }
    else{
      coralConfig.apply(new ClosedLoopConfig().pidf(
        CORAL_ENDEFFECTOR_KP_PRACTICE,
        CORAL_ENDEFFECTOR_KI_PRACTICE,
        CORAL_ENDEFFECTOR_KD_PRACTICE,
        CORAL_ENDEFFECTOR_KFF_PRACTICE));    
    }
    coralConfig.idleMode(IdleMode.kCoast);
    coralConfig.smartCurrentLimit(25, 40, 1000);
    coralEndeffectorMotor.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    coralEndeffSensor = coralEndeffectorMotor.getAnalog();
    motorLogger = new MotorLogger("/coralEndEffector/motor");

  
  }
  /**
   * a method to set the speeds of both motors on the end effector. speeds are percentage of full power, from -1 to 1.
   * @param speed the speed for motor1
   */
  public void set(double speed){
    coralEndeffectorMotor.set(speed);

  }
  public void stopCoralEndEffector(){
    coralEndeffectorMotor.set(0);
  }
  public void runCoralEndEffector(double RPM) {
    coralEndeffectorMotor.getClosedLoopController().setReference(RPM, ControlType.kVelocity);
  }
  private void logMotors(){
    motorLogger.log(coralEndeffectorMotor);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    RobotState.getInstance().coralEndeffSensorTrig = coralEndeffSensor.getVoltage()>2;
    if(Preferences.getBoolean("Motor Logging", false)){
    logMotors();
    }
  }
}
