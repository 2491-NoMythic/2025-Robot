// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.settings.Constants.AlgaeEndeffectorConstants.*;
import edu.wpi.first.math.controller.PIDController;
public class AlgaeEndeffectorSubsystem extends SubsystemBase {
  SparkMax algaeEndeffectorMotor1;
  SparkMax algaeEndeffectorMotor2;
  SparkBaseConfig algaeConfig1;
  SparkBaseConfig algaeConfig2;
  PIDController algendController;
  public boolean powerSpike;
  /** Creates a new AlgaeEndDefectorSubsystem. */
  public AlgaeEndeffectorSubsystem() {
    algaeEndeffectorMotor1 = new SparkMax(ALGAE_ENDEFFECTOR_MOTOR_1_ID, MotorType.kBrushless);
    algaeEndeffectorMotor2 = new SparkMax(ALGAE_ENDEFFECTOR_MOTOR_2_ID, MotorType.kBrushless);

    algaeConfig1 = new SparkMaxConfig();
    algaeConfig1.apply(new ClosedLoopConfig().pidf(
      ALGAE_ENDEFFECTOR_KP_1,
      ALGAE_ENDEFFECTOR_KI_1,
      ALGAE_ENDEFFECTOR_KD_1,
      ALGAE_ENDEFFECTOR_KFF_1));
    algaeConfig1.idleMode(IdleMode.kCoast);
    algaeConfig1.smartCurrentLimit(ALGAE_ENDEFFECTOR_CURRENT_LIMIT, ALGAE_ENDEFFECTOR_CURRENT_LIMIT, 1000);
    algaeEndeffectorMotor1.configure(algaeConfig1, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   
    algaeConfig2 = new SparkMaxConfig();
    algaeConfig2.apply(new ClosedLoopConfig().pidf(
      ALGAE_ENDEFFECTOR_KP_2,
      ALGAE_ENDEFFECTOR_KI_2,
      ALGAE_ENDEFFECTOR_KD_2,
      ALGAE_ENDEFFECTOR_KFF_2));
    algaeConfig2.idleMode(IdleMode.kCoast);
    algaeConfig2.smartCurrentLimit(ALGAE_ENDEFFECTOR_CURRENT_LIMIT, ALGAE_ENDEFFECTOR_CURRENT_LIMIT, 1000);
    algaeEndeffectorMotor2.configure(algaeConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    powerSpike = false;
  }

  public void runAlgaeEndDefector(double speed){
    algaeEndeffectorMotor1.set(speed);
    algaeEndeffectorMotor2.set(speed);
  }

  public void stopAlgaeEndDefector(){
    algaeEndeffectorMotor1.set(0);
    algaeEndeffectorMotor2.set(0);
  }
  public void powerCheck(){
    //if we are at 80%+ percent of the current limit, assume it's becuse we have an algae
    if(algaeEndeffectorMotor1.getOutputCurrent()>ALGAE_ENDEFFECTOR_CURRENT_LIMIT*0.8){ 
      RobotState.getInstance().hasAlgae = true;
    }else{
      RobotState.getInstance().hasAlgae = false;
    }
    
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    powerCheck();
  }
}
