// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.settings.Constants.DriveConstants.DEFAULT_PATH_CONSTRAINTS;

import java.io.IOException;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import frc.robot.subsystems.RobotState;
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
  public PathFindToReef(DrivetrainSubsystem drivetrain, BooleanSupplier LeftSup) {
    this.drivetrain = drivetrain;
    this.LeftSupplier = LeftSup;
    addRequirements();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  //first, determine which path to use based on the robot state for the closest reef side
  //lots of try/multi-catch becuase that is necessary when using the .fromPathFile() method
    switch (RobotState.getInstance().closestReefSide) {
      case backCenter:
      if(LeftSupplier.getAsBoolean()) {
        try {
          pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupA"), DEFAULT_PATH_CONSTRAINTS);
        } catch (FileVersionException | IOException | ParseException e) {
          e.printStackTrace();
        }
      } else {
        try {
          pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupB"), DEFAULT_PATH_CONSTRAINTS);
        } catch (FileVersionException | IOException | ParseException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      }
    case backRight:
      if(LeftSupplier.getAsBoolean()) {
        try {
          pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupC"), DEFAULT_PATH_CONSTRAINTS);
        } catch (FileVersionException | IOException | ParseException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      } else {
        try {
          pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupD"), DEFAULT_PATH_CONSTRAINTS);
        } catch (FileVersionException | IOException | ParseException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      }
    case backLeft:
    if(LeftSupplier.getAsBoolean()) {
      try {
        pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupK"), DEFAULT_PATH_CONSTRAINTS);
      } catch (FileVersionException | IOException | ParseException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    } else {
      try {
        pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupL"), DEFAULT_PATH_CONSTRAINTS);
      } catch (FileVersionException | IOException | ParseException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }
    case frontCenter:
    if(LeftSupplier.getAsBoolean()) {
      try {
        pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupG"), DEFAULT_PATH_CONSTRAINTS);
      } catch (FileVersionException | IOException | ParseException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    } else { 
      try {
        pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupH"), DEFAULT_PATH_CONSTRAINTS);
      } catch (FileVersionException | IOException | ParseException e) {
        // TODO Auto-generated catch block
        e.printStackTrace();
      }
    }
    case frontRight:
      if(LeftSupplier.getAsBoolean()) {
        try {
          pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupE"), DEFAULT_PATH_CONSTRAINTS);
        } catch (FileVersionException | IOException | ParseException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      } else { 
        try {
          pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupF"), DEFAULT_PATH_CONSTRAINTS);
        } catch (FileVersionException | IOException | ParseException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      }
    case frontLeft:
      if(LeftSupplier.getAsBoolean()) {
        try {
          pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupI"), DEFAULT_PATH_CONSTRAINTS);
        } catch (FileVersionException | IOException | ParseException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      } else { 
        try {
          pathToReef = AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupJ"), DEFAULT_PATH_CONSTRAINTS);
        } catch (FileVersionException | IOException | ParseException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }
      }
    default:
        pathToReef = new InstantCommand(()->System.out.println("tried to run pathFindToReef, but was not close to any reefside"));
    }
    pathToReef.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pathToReef.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pathToReef.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return pathToReef.isFinished();
    return false;
  }
}
