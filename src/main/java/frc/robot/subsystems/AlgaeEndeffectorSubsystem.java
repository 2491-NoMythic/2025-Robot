// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.MotorLogger;

import static frc.robot.settings.Constants.AlgaeEndeffectorConstants.*;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
public class AlgaeEndeffectorSubsystem extends SubsystemBase {
  TalonFXS algaeEndeffectorMotor;
  // SparkBaseConfig algaeConfig1;
  TalonFXSConfiguration algaeConfig;
  PIDController algendController;
  MotorLogger motorLogger1;
  // MotorLogger motorLogger2;
  int loops;
  public boolean powerSpike;
  /** Creates a new AlgaeEndDefectorSubsystem. */
  public AlgaeEndeffectorSubsystem() {
    algaeEndeffectorMotor = new TalonFXS(ALGAE_ENDEFFECTOR_MOTOR_ID);

    motorLogger1 = new MotorLogger("/algaeEndEffector/motor1");
    // motorLogger2 = new MotorLogger("/algaeEndEffector/motor2");

    // algaeConfig1 = new SparkMaxConfig();
    algaeConfig = new TalonFXSConfiguration();
    algaeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    algaeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    algaeConfig.CurrentLimits.SupplyCurrentLimit = ALGAE_ENDEFFECTOR_CURRENT_LIMIT;
    algaeConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    algaeEndeffectorMotor.getConfigurator().apply(algaeConfig);
    powerSpike = false;
  }
  /**
   * Gets the motor object.
   * @return the motor
   */
  public TalonFXS getMotor(){
    return algaeEndeffectorMotor;
  }
  /**
   * Runs the AlgaeEndEffector at a speed from -1 to 1
   * @param speed 
   */
  public void runAlgaeEndDefector(double speed){
    algaeEndeffectorMotor.set(speed);
  }
  /**
   * Runs the AlgaeEndEffector at a speed from -1 to 1, but uses a double supplier instead of a double.
   * @param speed the double supplier for the speed
   */
  public void runAlgaeEndDefector(DoubleSupplier speed){
    algaeEndeffectorMotor.set(speed.getAsDouble());
  }
  /**
   * stops the algae end effector motor by setting speed to 0, with brake mode enabled.
   */
  public void stopAlgaeEndDefectorHard(){
    algaeEndeffectorMotor.set(0);
  }
  /**
   * Stop the algae end effector motor by setting voltage to 0, letting it coast out.
   */
  public void stopAlgaeEndDefectorCoast(){
    algaeEndeffectorMotor.setVoltage(0);
  }
  /**
   * Updates hasAlgae to true or false based on if algaeEndeffectorMotor1 has a high output current for a period of time
   */
  public void powerCheck(){

    SmartDashboard.putNumber("AlgaeMotorCurrent",algaeEndeffectorMotor.getStatorCurrent().getValueAsDouble());
    //if we are at 80%+ percent of the current limit, assume it's becuse we have an algae
    if(algaeEndeffectorMotor.getSupplyCurrent().getValueAsDouble()>ALGAE_ENDEFFECTOR_CURRENT_LIMIT*0.8){ 
      loops++;
      if(loops > 10){
        RobotState.getInstance().hasAlgae = true;
      }
    }else{
      RobotState.getInstance().hasAlgae = false;
      loops = 0;
    }
  }
  private void logMotors(){
    motorLogger1.log(algaeEndeffectorMotor);
    // motorLogger2.log(algaeEndeffectorMotor2);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(Preferences.getBoolean("Motor Logging", false)){
    logMotors();
    }
    powerCheck();
  }
}
