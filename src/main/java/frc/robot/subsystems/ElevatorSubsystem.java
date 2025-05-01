// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ForwardLimitTypeValue;
import com.ctre.phoenix6.signals.ForwardLimitValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;
import com.ctre.phoenix6.signals.ReverseLimitValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.MotorLogger;
import frc.robot.settings.ElevatorEnums;
import frc.robot.subsystems.RobotState;

import static frc.robot.settings.Constants.DriveConstants.CANIVORE_DRIVETRAIN;
import static frc.robot.settings.Constants.ElevatorConstants.*;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorSubsystem extends SubsystemBase {
  private TalonFX elevatorMotor1;
  private TalonFX elevatorMotor2;
  private TalonFXConfiguration eleMotorConfig;
  private double zeroPoint;
  MotorLogger motorLogger1;
  MotorLogger motorLogger2;
  double elevatorTarget = 0;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorMotor1 = new TalonFX(ELEVATOR_MOTOR_1_ID, CANIVORE_DRIVETRAIN);
    elevatorMotor2 = new TalonFX(ELEVATOR_MOTOR_2_ID, CANIVORE_DRIVETRAIN);
    eleMotorConfig = new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake))
      .withFeedback(new FeedbackConfigs()
        .withSensorToMechanismRatio(0.08910703))
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(200)
        .withSupplyCurrentLowerLimit(40)
        .withSupplyCurrentLowerTime(1)
        .withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimitEnable(false))
      .withVoltage(new VoltageConfigs()
        .withPeakForwardVoltage(7)
        .withPeakReverseVoltage(-7))
      .withMotionMagic(new MotionMagicConfigs()
        .withMotionMagicAcceleration(MOTION_MAGIC_ELEVATOR_ACCLERATION)
        .withMotionMagicCruiseVelocity(MOTION_MAGIC_ELEVATOR_VELOCITY)
        .withMotionMagicJerk(MOTION_MAGIC_ELEVATOR_JERK))
      .withHardwareLimitSwitch(new HardwareLimitSwitchConfigs()
        .withReverseLimitAutosetPositionEnable(true)
        .withReverseLimitEnable(true)
        .withReverseLimitType(ReverseLimitTypeValue.NormallyClosed)
        .withForwardLimitAutosetPositionEnable(true)
        .withForwardLimitAutosetPositionValue(COMP_HEIGHT_AT_UPPER_LIMIT_SWITCH)
        .withForwardLimitEnable(true)
        .withForwardLimitType(ForwardLimitTypeValue.NormallyClosed))
      .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
        .withForwardSoftLimitEnable(true)
        .withForwardSoftLimitThreshold(COMP_HEIGHT_AT_UPPER_LIMIT_SWITCH-1)
        .withReverseSoftLimitEnable(true)
        .withReverseSoftLimitThreshold(COMP_HEIGHT_AT_LOWER_LIMIT_SWITCH+1));
    if (Preferences.getBoolean("CompBot", true)){
      //good values from right before duluth: p: 0.64, d: 0.02, g: 0.8623, a: 0.002, v: 0.01, s: 0.6  
      eleMotorConfig.Slot0 = new Slot0Configs()
        .withKP(0.64)
        .withKD(0.025)
        .withKG(0.97754)
        .withKA(0.0002)
        .withKV(0.014)//0.0075 worked well for 20 cm/s on 3-13 5:27
        .withKS(0.6);
      eleMotorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = COMP_HEIGHT_AT_LOWER_LIMIT_SWITCH;
    } else {
      eleMotorConfig.Slot0 = new Slot0Configs()
        .withKP(0.4)
        .withKG(0.57)
        .withKA(0.00040067)
        .withKV(0.02168)
        .withKS(1.17);
      eleMotorConfig.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = PRAC_HEIGHT_AT_LIMIT_SWITCH;
    }
    elevatorMotor1.getConfigurator().apply(eleMotorConfig);
    elevatorMotor2.setControl(new Follower(ELEVATOR_MOTOR_1_ID, false));
    elevatorMotor2.getConfigurator().apply(new TalonFXConfiguration()
      .withHardwareLimitSwitch(new HardwareLimitSwitchConfigs()
        .withForwardLimitEnable(false)
        .withReverseLimitEnable(false)));

    motorLogger1 = new MotorLogger("/elevator/motor1");
    motorLogger2 = new MotorLogger("/elevator/motor2");
  }
  private void logMotors(){
    motorLogger1.log(elevatorMotor1);
    motorLogger2.log(elevatorMotor2);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(Preferences.getBoolean("Motor Logging", false)){
    logMotors();
    }
    if(limitSwitchTrig()) {
      RobotState.getInstance().elevatorZeroSet = true;
    }
    SmartDashboard.putNumber("TESTING/elevatorHeight", elevatorMotor1.getPosition().getValueAsDouble());
    SmartDashboard.putBoolean("TESTING/limit switch value", limitSwitchTrig());
    SmartDashboard.putBoolean("TESTING/elevatorAtPose", isElevatorAtPose());
    SmartDashboard.putNumber("TESTING/elevatorTarget", elevatorTarget);
    SmartDashboard.putNumber("TESTING/elevatorClosedLoopError", Math.abs(elevatorTarget-elevatorMotor1.getPosition().getValueAsDouble()));
  }
  /**
   * tells the elevator motor what rotations it will have to reach for the elevator to be touching the ground (this will never happen, just theoritically) <p>
   * this is necessary so that the elevator has a reference point to calculate the position of any height off the ground. Run this before ever setting the elevator to a position
   * @param theDistance the distance that the elevator is from the ground, in centimeters
   */
  public void setZero(double theDistance){
    double rotationsFromGround = theDistance;
    zeroPoint = elevatorMotor1.getPosition().getValueAsDouble() - rotationsFromGround;   
  }
  public double getHeightCentimeters() {
    return elevatorMotor1.getPosition().getValueAsDouble();
  }
  /**
   * sets the voltage of the elevator motor. It takes about 2 volts to move the elevator slowly up, and higher than 4 is dangerously fast
   * it takes about 1 volt to move the elevator slowly down, and lower than -2 is dangerously fast
   * @param voltage
   */
  public void setVoltage(double voltage) {
    elevatorMotor1.setControl(new VoltageOut(voltage));
  }
  /**
   * Makes the elevator move to a position relative to the ground. It does this by changing the setpoint for the motor's onboard PID controller
   * @param height the desired height, in centimeters off the ground
   */
  public void setElevatorPosition(double height){
    double targetHeight = calculateRotations(height);
    elevatorTarget = height;
    MotionMagicVoltage request = new MotionMagicVoltage(targetHeight);

    elevatorMotor1.setControl(request);
  }
  public void setElevatorPosition(DoubleSupplier height){
    setElevatorPosition(height.getAsDouble());
  }
  public void setElevatorPositionDynamicConfigs(double height, double acceleration, double velocity, double jerk) {
    double targetHeight = calculateRotations(height);
    elevatorTarget = height;
    DynamicMotionMagicVoltage request = new DynamicMotionMagicVoltage(targetHeight, velocity, acceleration, jerk);
    elevatorMotor1.setControl(request);
  }
  public void setElevatorPosition(Supplier<ElevatorEnums> heightSupplier) {
    setElevatorPosition(heightSupplier.get());
  }
  public void setElevatorPositionWithAlgae(Supplier<ElevatorEnums> heightSupplier) {
    setElevatorPositionWithAlgae(heightSupplier.get());
  }
  /**
   * sets the height of the elvator using constants associated with different values of ElevatorEnums
   * @param height
   */
  public void setElevatorPosition(ElevatorEnums height){
    switch(height){
      case Reef1:
        setElevatorPosition(REEF_LEVEL_1_CENTIMETERS_AWAY_FROM_REEF);
        break;
      case Reef2:
        setElevatorPosition(REEF_LEVEL_2_CENTIMETERS_AWAY_FROM_REEF);
        if(isElevatorAtPose()){
          RobotState.getInstance().elevatorIsHigh = true;
        }
        break;
      case Reef3:
        setElevatorPosition(REEF_LEVEL_3_CENTIMETERS_AWAY_FROM_REEF);
        if(isElevatorAtPose()){
          RobotState.getInstance().elevatorIsHigh = true;
        }
        break;
      case Reef4:
        // if(DriverStation.isAutonomous()) {
        //   setElevatorPositionDynamicConfigs(REEF_LEVEL_4_CENTIMETERS_AWAY_FROM_REEF, MOTION_MAGIC_ELEVATOR_HP_ACCLERATION, MOTION_MAGIC_ELEVATOR_HP_VELOCITY, MOTION_MAGIC_ELEVATOR_JERK);
        // } else {
          setElevatorPositionDynamicConfigs(REEF_LEVEL_4_CENTIMETERS_AWAY_FROM_REEF, MOTION_MAGIC_ELEVATOR_HIGH_ACCLERATION, MOTION_MAGIC_ELEVATOR_HIGH_VELOCITY, MOTION_MAGIC_ELEVATOR_JERK);
        // }
        if(isElevatorAtPose()){
          RobotState.getInstance().elevatorIsHigh = true;
        }
        break;
      case HumanPlayer:
        setElevatorPositionDynamicConfigs(HUMAN_PLAYER_STATION_CENTIMETERS, MOTION_MAGIC_ELEVATOR_HP_ACCLERATION, MOTION_MAGIC_ELEVATOR_HP_VELOCITY, MOTION_MAGIC_ELEVATOR_JERK);
        if(isElevatorAtPose()){
          RobotState.getInstance().elevatorIsHigh = false;
        }
        break;
      case Bottom:
      if(Preferences.getBoolean("CompBot", true)) {
        setElevatorPosition(COMP_HEIGHT_AT_LOWER_LIMIT_SWITCH);
      } else {
        setElevatorPosition(PRAC_HEIGHT_AT_LIMIT_SWITCH);
      }
        break;
      case AlgaeInProcessor:
        setElevatorPosition(PROCESSOR_HEIGHT_CENTIMETERS);
        break;
      case Barge:
        setElevatorPositionDynamicConfigs(BARGE_SHOOT_CENTIMETERS, MOTION_MAGIC_ELEVATOR_HIGH_ACCLERATION, MOTION_MAGIC_ELEVATOR_HIGH_VELOCITY, MOTION_MAGIC_ELEVATOR_JERK);
        break;
    }
  }
  /**
   * sets the height of the elvator using constants associated with different values of ElevatorEnums. Uses slower acceleration so that we are able to pick up aglae
   * @param height
   */

   //default: 320 cm/s 640 cm/s/s

   //works: 100 cm/s/s 200 cm/s 
   // TODO: may need speedup
  public void setElevatorPositionWithAlgae(ElevatorEnums height){
    final double algaeAcceleration = MOTION_MAGIC_ELEVATOR_HIGH_ACCLERATION;
    final double algaeVelocity = MOTION_MAGIC_ELEVATOR_HIGH_VELOCITY-150;
    switch(height){
      case Reef1:
      setElevatorPositionDynamicConfigs(REEF_LEVEL_1_CENTIMETERS_AGAINST_REEF, algaeAcceleration, algaeVelocity, 0);
        break;
      case Reef2:
      setElevatorPositionDynamicConfigs(REEF_LEVEL_2_CENTIMETERS_AGAINST_REEF, algaeAcceleration, algaeVelocity, 0);
        if(isElevatorAtPose()){
          RobotState.getInstance().elevatorIsHigh = true;
        }
        break;
      case Reef3:
        setElevatorPositionDynamicConfigs(REEF_LEVEL_3_CENTIMETERS_AGAINST_REEF, algaeAcceleration, algaeVelocity, 0);
      
      if(isElevatorAtPose()){
        RobotState.getInstance().elevatorIsHigh = true;
      }
      break;
      case Reef3Algae:
        setElevatorPositionDynamicConfigs(REEF_LEVEL_3_ALGAE_HEIGHT, algaeAcceleration, algaeVelocity, 0);
        break;
      case Reef4:
        setElevatorPositionDynamicConfigs(REEF_LEVEL_4_CENTIMETERS_AGAINST_REEF, algaeAcceleration, algaeVelocity, 0);
        if(isElevatorAtPose()){
          RobotState.getInstance().elevatorIsHigh = true;
        }
        break;
      case HumanPlayer:
        setElevatorPositionDynamicConfigs(HUMAN_PLAYER_STATION_CENTIMETERS, algaeAcceleration, algaeVelocity, 0);
        if(isElevatorAtPose()){
          RobotState.getInstance().elevatorIsHigh = false;
        }
        break;
      case Bottom:
      if(Preferences.getBoolean("CompBot", true)) {
        setElevatorPosition(COMP_HEIGHT_AT_LOWER_LIMIT_SWITCH);
      } else {
        setElevatorPosition(PRAC_HEIGHT_AT_LIMIT_SWITCH);
      }
        break;
      case AlgaeInProcessor:
        setElevatorPosition(PROCESSOR_HEIGHT_CENTIMETERS);
        break;
      case Barge:
        setElevatorPositionDynamicConfigs(BARGE_SHOOT_CENTIMETERS, MOTION_MAGIC_ELEVATOR_HIGH_ACCLERATION, MOTION_MAGIC_ELEVATOR_HIGH_VELOCITY, 0);
        break;
    }
  }
  /**
   * calculate the target rotations for the motor based on a desired height off the ground. If the input height is lower than the height of the limit switch, 
   * the output will be the height of the limit switch
   * @param desiredHeight height off the ground, in millimeters
   * @return the taret position for the motor, in rotations
   */
  private double calculateRotations(double desiredHeight) {
    if(Preferences.getBoolean("CompBot", true)) {
      return (Math.max(desiredHeight, COMP_HEIGHT_AT_LOWER_LIMIT_SWITCH));
    } else {
      return (Math.max(desiredHeight, PRAC_HEIGHT_AT_LIMIT_SWITCH));
    }
  }
  public void holdElevatorPose() {
    setVoltage(eleMotorConfig.Slot0.kG);
  }
  /**
   * asks if the error on the closed loop is less than our ELEVATOR_THRESHOLD constant
   * @return true if closed loop error is less than our threshold, false otherwise
   */
  public boolean isElevatorAtPose() {
    return Math.abs(getPIDTarget()-elevatorMotor1.getPosition().getValueAsDouble()) < ELEVATOR_THRESHOLD && Math.abs(elevatorMotor1.getVelocity().getValueAsDouble()) < 15;
  }
  /**
   * asks if the error on the closed loop is less than our ELEVATOR_THRESHOLD constant
   * @return true if closed loop error is less than our threshold, false otherwise
   */
  public boolean isElevatorAtIntakeHeight() {
    return isElevatorAtPose() && Math.abs(getPIDTarget()-HUMAN_PLAYER_STATION_CENTIMETERS) < 4;
  }
  /**
   * the current reference for the onboard ClosedLoopController
   * @return the reference, in rotations
   */
  public double getPIDTarget() {
    return elevatorTarget;
  }
  /**
   * stops the elevator by setting it's target to wherever it is right now
   */
  public void stopElevator(){
    elevatorMotor1.setControl(new MotionMagicVoltage(elevatorMotor1.getPosition().getValueAsDouble()));
  }

  public boolean limitSwitchTrig(){
    return elevatorMotor1.getReverseLimit().getValue() == ReverseLimitValue.Open;
  }

}
