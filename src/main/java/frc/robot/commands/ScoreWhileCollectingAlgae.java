// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.settings.Constants.AlgaeEndeffectorConstants.ALGAE_INTAKE_SPEED;
import static frc.robot.settings.Constants.ElevatorConstants.HUMAN_PLAYER_STATION_CENTIMETERS;
import static frc.robot.settings.Constants.ElevatorConstants.MOTION_MAGIC_ELEVATOR_HP_ACCLERATION;
import static frc.robot.settings.Constants.ElevatorConstants.MOTION_MAGIC_ELEVATOR_HP_VELOCITY;
import static frc.robot.settings.Constants.ElevatorConstants.MOTION_MAGIC_ELEVATOR_JERK;

import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.NamedCommands.DeliverCoral;
import frc.robot.settings.ElevatorEnums;
import frc.robot.settings.PlacementLocations;
import frc.robot.subsystems.AlgaeEndeffectorSubsystem;
import frc.robot.subsystems.CoralEndeffectorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.RobotState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ScoreWhileCollectingAlgae extends SequentialCommandGroup {
  /** Creates a new ScoreWhileCollectingAlgae. */
  public ScoreWhileCollectingAlgae(CoralEndeffectorSubsystem coralEndDefector, AlgaeEndeffectorSubsystem algaeEndDefector, DrivetrainSubsystem driveTrain, ElevatorSubsystem elevator, Supplier<PlacementLocations> placementLocationSupplier, Supplier<ElevatorEnums> heightSupplier) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(()->RobotState.getInstance().goForAlgae = true),
      new InstantCommand(()->coralEndDefector.stopCoralEndEffector(), coralEndDefector),
      new InstantCommand(()->elevator.setElevatorPositionDynamicConfigs(HUMAN_PLAYER_STATION_CENTIMETERS+10, MOTION_MAGIC_ELEVATOR_HP_ACCLERATION, MOTION_MAGIC_ELEVATOR_HP_VELOCITY, MOTION_MAGIC_ELEVATOR_JERK), elevator),
      new DriveToPose(placementLocationSupplier, driveTrain, ()->0),
      new InstantCommand(()->driveTrain.drive(new ChassisSpeeds(0.7, 0, 0))),
      new ParallelRaceGroup(
          new AlgaeIntakeCommand(algaeEndDefector, () -> RobotState.getInstance().goForAlgae ? ALGAE_INTAKE_SPEED : -0.5),
          new SequentialCommandGroup(
              new ParallelRaceGroup(
                  new ElevatorCommand(elevator, heightSupplier),//raises elevator to position)
                  new WaitUntil(()->elevator.isElevatorAtPose())),
              new ParallelRaceGroup(
                  new DeliverCoral(coralEndDefector),//drops coral
                  new WaitCommand(()->0.5)))),
      new InstantCommand(()->coralEndDefector.stopCoralEndEffector(), coralEndDefector)
    );
  }
}