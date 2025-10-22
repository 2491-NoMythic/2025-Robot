package frc.robot.LogOutputs;

import org.littletonrobotics.junction.Logger;

public class CoralEndEffectorOutputs {
    public double targetRPS;

    public void log() {
        Logger.recordOutput("CoralEndEffector/TargetRPS", targetRPS);
    }
}
