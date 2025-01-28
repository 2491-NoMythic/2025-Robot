// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.settings.Constants.DriveConstants.DEFAULT_PATH_CONSTRAINTS;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.settings.ReefSideEnum;
import frc.robot.subsystems.DrivetrainSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PathFindToReef extends Command {
  DrivetrainSubsystem drivetrain;
  Command pathToReef;
  Supplier<ReefSideEnum> closestReefSideSupplier;
  BooleanSupplier LeftSupplier;

  /** Creates a new PathFindToReef. */
  public PathFindToReef(DrivetrainSubsystem drivetrain, Supplier<ReefSideEnum> closestReefSideSupplier, BooleanSupplier LeftSup) {
    this.drivetrain = drivetrain;
    this.closestReefSideSupplier = closestReefSideSupplier;
    this.LeftSupplier = LeftSup;
    addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    switch (closestReefSideSupplier.get()) {
      case backCenter:
        if(LeftSupplier.getAsBoolean()) {
          pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupA"), DEFAULT_PATH_CONSTRAINTS);
        } else {
          pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupB"), DEFAULT_PATH_CONSTRAINTS);
        }
      case backRight:
        if(LeftSupplier.getAsBoolean()) {
          pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupC"), DEFAULT_PATH_CONSTRAINTS);
        } else {
          pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupD"), DEFAULT_PATH_CONSTRAINTS);
        }
      case backLeft:
      if(LeftSupplier.getAsBoolean()) {
        pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupK"), DEFAULT_PATH_CONSTRAINTS);
      } else {
        pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupL"), DEFAULT_PATH_CONSTRAINTS);
      }
      case frontCenter:
      if(LeftSupplier.getAsBoolean()) {
        pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupG"), DEFAULT_PATH_CONSTRAINTS);
      } else { 
        pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupH"), DEFAULT_PATH_CONSTRAINTS);
      }
      case frontRight:
        if(LeftSupplier.getAsBoolean()) {
          pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupE"), DEFAULT_PATH_CONSTRAINTS);
        } else { 
          pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupF"), DEFAULT_PATH_CONSTRAINTS);
        }
      case frontLeft:
        if(LeftSupplier.getAsBoolean()) {
          pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupI"), DEFAULT_PATH_CONSTRAINTS);
        } else { 
          pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupJ"), DEFAULT_PATH_CONSTRAINTS);
        }
      default:
        pathToReef = new InstantCommand(()->System.out.println("tried to run pathFindToReef, but was not close to any reefside"));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
