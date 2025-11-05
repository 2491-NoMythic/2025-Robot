package frc.robot.LogInputs;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

@AutoLog

public class DrivetrainInputs {
    public SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
    public SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
    public Rotation2d gyroScopeRotation = new Rotation2d();
    public double pitch;
    public double roll;
    public double gyroTimeStamp;
    public double angularVelocity;
}
