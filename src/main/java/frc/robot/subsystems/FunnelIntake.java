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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.MotorLogger;
import frc.robot.settings.Constants;

public class FunnelIntake extends SubsystemBase {
  /** Creates a new Funnelintake. */
  SparkMax funnelSlantMotor;
  SparkMax funnelStraightMotor;
  SparkBaseConfig slantMotorConfig;
  SparkBaseConfig straightMotorConfig;
  MotorLogger slantMotorLogger;
  MotorLogger straightMotorLogger;
  SparkAnalogSensor funnelIntakeSensor;
  Timer timer;

  
  public FunnelIntake() {
    timer = new Timer();
    funnelSlantMotor = new SparkMax(FUNNEL_SLANT_MOTOR_ID, MotorType.kBrushless);
    slantMotorConfig = new SparkMaxConfig();
    funnelStraightMotor = new SparkMax(FUNNEL_STRAIGHT_MOTOR_ID,MotorType.kBrushless);
    straightMotorConfig = new SparkMaxConfig();

    //applying PID settings based on if we are using the CompBot or the PracticeBot
    if(Preferences.getBoolean("CompBot", true)){
      slantMotorConfig.apply(new ClosedLoopConfig().pidf(
        FUNNEL_SLANT_MOTOR_KP,
        FUNNEL_SLANT_MOTOR_KI,
        FUNNEL_SLANT_MOTOR_KD,
        FUNNEL_SLANT_MOTOR_KFF));
       straightMotorConfig.apply(new ClosedLoopConfig().pidf(
         FUNNEL_STRAIGHT_MOTOR_KP,
         FUNNEL_STRAIGHT_MOTOR_KI,
         FUNNEL_STRAIGHT_MOTOR_KD,
         FUNNEL_STRAIGHT_MOTOR_KFF));
      }
      else{
        slantMotorConfig.apply(new ClosedLoopConfig().pidf(
          FUNNEL_SLANT_MOTOR_KP_PRACTICE,
          FUNNEL_SLANT_MOTOR_KI_PRACTICE,
          FUNNEL_SLANT_MOTOR_KD_PRACTICE,
          FUNNEL_SLANT_MOTOR_KFF_PRACTICE));
        straightMotorConfig.apply(new ClosedLoopConfig().pidf(
          FUNNEL_STRAIGHT_MOTOR_KP_PRACTICE,
          FUNNEL_STRAIGHT_MOTOR_KI_PRACTICE,
          FUNNEL_STRAIGHT_MOTOR_KD_PRACTICE,
          FUNNEL_STRAIGHT_MOTOR_KFF_PRACTICE));
      }
    slantMotorConfig.idleMode(IdleMode.kCoast);
    slantMotorConfig.inverted(true);
    slantMotorConfig.smartCurrentLimit(25, 25, 1000);
    funnelSlantMotor.configure(slantMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    straightMotorConfig.idleMode(IdleMode.kCoast);
    straightMotorConfig.inverted(false);
    straightMotorConfig.smartCurrentLimit(25, 25, 1000);
    funnelStraightMotor.configure(straightMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    slantMotorLogger = new MotorLogger("/funnelIntake/slantMotor");
    straightMotorLogger = new MotorLogger("/funnelIntake/straightMotor");
    
    funnelIntakeSensor = funnelSlantMotor.getAnalog();
  }

  private void logMotors(){
    slantMotorLogger.log(funnelSlantMotor);
    straightMotorLogger.log(funnelStraightMotor);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    RobotState.getInstance().funnelSensorTrig = funnelIntakeSensor.getVoltage()>2;
    if(Preferences.getBoolean("Motor Logging", false)){
    logMotors();
    }
  }
  /**
   * Runs the different FunnelIntake motors at proportinal speeds, with funnelSlantMotor running at 2/3 of speed of funnelStraightMotor
   * @param RPM the rpm setpoint for the wheels the motors control
   */
  public void runFunnel(double RPM){
    funnelSlantMotor.getClosedLoopController().setReference(RPM*(2.0/3), ControlType.kVelocity);
    funnelStraightMotor.getClosedLoopController().setReference(RPM, ControlType.kVelocity);
    System.out.println("run");
  }
  /**
   * Runs the FunnelIntake at a speed that changes over time, based off a sine wave
   */
   public void runFunnelSine( ){
    timer.start();
    funnelStraightMotor.getClosedLoopController().setReference(Math.abs(Math.sin(timer.get()) * FUNNEL_INTAKE_SPEED) + 1600.0, ControlType.kVelocity); 
    funnelSlantMotor.getClosedLoopController().setReference(Math.abs(Math.sin(timer.get() + 0.5)*(1.3/3) * FUNNEL_INTAKE_SPEED) + 1600.0, ControlType.kVelocity); 
   }
  public void stopFunnel() {
    funnelSlantMotor.set(0);
    funnelStraightMotor.set(0);
    timer.stop();                                                                                                                                            
    timer.reset();
  }

}
