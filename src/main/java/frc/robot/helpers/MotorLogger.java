package frc.robot.helpers;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;

/** logs motor data to a logger */
public class MotorLogger {
  // DoubleLogEntry voltage;
  // DoubleLogEntry current;
  // DoubleLogEntry velocity;

  NetworkTable loggingTable;

  NetworkTableEntry voltageEntry;
  NetworkTableEntry currentEntry;
  NetworkTableEntry velocityEntry;
  NetworkTableEntry temperatureEntry;


  public MotorLogger(String path) {
    // voltage = new DoubleLogEntry(log, path + "/voltage");
    // current = new DoubleLogEntry(log, path + "/current");
    // velocity = new DoubleLogEntry(log, path + "/velocity");

    loggingTable = NetworkTableInstance.getDefault().getTable("MotorData");

    voltageEntry = loggingTable.getEntry(path + "/voltage");
    currentEntry = loggingTable.getEntry(path + "/current");
    velocityEntry = loggingTable.getEntry(path + "/velocity");
    temperatureEntry = loggingTable.getEntry(path + "/temperature");
  }

  public void log(SparkMax motor) {
    // current.append(motor.getOutputCurrent());
    // voltage.append(motor.getAppliedOutput() * motor.getBusVoltage());
    // velocity.append(motor.getEncoder().getVelocity() / 60);

    voltageEntry.setDouble(motor.getAppliedOutput() * motor.getBusVoltage());
    currentEntry.setDouble(motor.getOutputCurrent());
    velocityEntry.setDouble(motor.getEncoder().getVelocity() / 60);
    temperatureEntry.setDouble(motor.getMotorTemperature());
  }

  public void log(TalonFX motor) {
    // current.append(motor.getStatorCurrent().getValueAsDouble());
    // voltage.append(motor.getMotorVoltage().getValueAsDouble());
    // velocity.append(motor.getVelocity().getValueAsDouble());

    currentEntry.setDouble(motor.getStatorCurrent().getValueAsDouble());
    voltageEntry.setDouble(motor.getMotorVoltage().getValueAsDouble());
    velocityEntry.setDouble(motor.getVelocity().getValueAsDouble());
    temperatureEntry.setDouble(motor.getDeviceTemp().getValueAsDouble());
  }
}
