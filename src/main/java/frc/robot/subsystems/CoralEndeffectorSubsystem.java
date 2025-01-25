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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.MotorLogger;

import static frc.robot.settings.Constants.CoralEndeffectorConstants.*;

public class CoralEndeffectorSubsystem extends SubsystemBase {
  SparkMax coralEndeffectorMotor1;
  SparkMax coralEndeffectorMotor2;
  SparkBaseConfig coralConfig1;
  SparkBaseConfig coralConfig2;
  MotorLogger motorLogger1;
  MotorLogger motorLogger2;

  /** Creates a new CoralEndDefectorSubsystem. */
  public CoralEndeffectorSubsystem() {
    coralEndeffectorMotor1 = new SparkMax(CORAL_ENDEFFECTOR_MOTOR_1_ID, MotorType.kBrushless);
    coralEndeffectorMotor2 = new SparkMax(CORAL_ENDEFFECTOR_MOTOR_1_ID, MotorType.kBrushless);
    coralConfig1 = new SparkMaxConfig();
    coralConfig1.apply(new ClosedLoopConfig().pidf(
      CORAL_ENDEFFECTOR_KP_1,
      CORAL_ENDEFFECTOR_KI_1,
      CORAL_ENDEFFECTOR_KD_1,
      CORAL_ENDEFFECTOR_KFF_1));
    coralConfig1.idleMode(IdleMode.kCoast);
    coralConfig1.smartCurrentLimit(25, 40, 1000);
    coralEndeffectorMotor1.configure(coralConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    coralConfig1 = new SparkMaxConfig();

    coralConfig2.apply(new ClosedLoopConfig().pidf(
      CORAL_ENDEFFECTOR_KP_1,
      CORAL_ENDEFFECTOR_KI_1,
      CORAL_ENDEFFECTOR_KD_1,
      CORAL_ENDEFFECTOR_KFF_1));
    coralConfig2.idleMode(IdleMode.kCoast);
    coralConfig2.smartCurrentLimit(25, 25, 1000);
    coralEndeffectorMotor2.configure(coralConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    motorLogger1 = new MotorLogger("/coralEndEffector/motor1");
    motorLogger2 = new MotorLogger("/coralEndEffector/motor2");
  }
  /**
   * a method to set the speeds of both motors on the end effector. speeds are percentage of full power, from -1 to 1.
   * @param speed1 the speed for motor1
   * @param speed2 the speed for motor2
   */
  public void runCoralEndEffector(double speed1, double speed2){
    coralEndeffectorMotor1.set(speed1);
    coralEndeffectorMotor2.set(speed2);

  }
  public void stopCoralEndEffector(){
    coralEndeffectorMotor1.set(0);
    coralEndeffectorMotor2.set(0);
  }
  private void logMotors(){
    motorLogger1.log(coralEndeffectorMotor1);
    motorLogger2.log(coralEndeffectorMotor2);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    logMotors();
  }
}
