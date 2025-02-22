

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.NamedCommands.DeliverCoral;
import frc.robot.settings.ElevatorEnums;
import frc.robot.subsystems.AlgaeEndeffectorSubsystem;
import frc.robot.subsystems.CoralEndeffectorSubsystem;
import frc.robot.subsystems.DistanceSensors;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.Supplier;

import static frc.robot.settings.Constants.AlgaeEndeffectorConstants.ALGAE_INTAKE_SPEED;
import static frc.robot.settings.Constants.DriveConstants.REEF_LINEUP_SPEED;
import static frc.robot.settings.Constants.ElevatorConstants.HUMAN_PLAYER_STATION_CENTIMETERS;
import static frc.robot.settings.Constants.ElevatorConstants.MOTION_MAGIC_ELEVATOR_SLOWER_ACCLERATION;
import static frc.robot.settings.Constants.ElevatorConstants.MOTION_MAGIC_ELEVATOR_VELOCITY;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.AlgaeEndeffectorSubsystem;


public class PlaceCoralNoPath extends SequentialCommandGroup{

    
    public PlaceCoralNoPath(
        ElevatorSubsystem elevator,
        Supplier<ElevatorEnums> elevatorPose,
        DistanceSensors distanceSensors,
        DrivetrainSubsystem drivetrain,
        DoubleSupplier xSupplier,
        DoubleSupplier ySupplier,
        DoubleSupplier rSupplier,
        CoralEndeffectorSubsystem coralEndeffector,
        BooleanSupplier leftPlace,
        AlgaeEndeffectorSubsystem algaeEndeffectorSubsystem,
        BooleanSupplier goForAlgea)
    {
        addCommands(
            new ApproachReef(distanceSensors, drivetrain, xSupplier, ySupplier, rSupplier),//approaches reef while raising elevator
            new LineUp(drivetrain, leftPlace, REEF_LINEUP_SPEED),//align with reef
            new ParallelRaceGroup(
                //elevator command will stop this so modify it
                new AlgaeIntakeCommand(algaeEndeffectorSubsystem, () -> goForAlgea.getAsBoolean() ? ALGAE_INTAKE_SPEED : 0),
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new ElevatorCommand(elevator, elevatorPose),//raises elevator to position)
                        new WaitUntil(()->elevator.isElevatorAtPose())),
                    new ParallelRaceGroup(
                        new DeliverCoral(coralEndeffector),//drops coral
                        new WaitCommand(()->0.75)))
            ),
            new InstantCommand(()->System.out.println("reeflineupended!!")), //sets elevator back to the bottom position
            new MoveMeters(drivetrain, -0.5, -0.8, 0, 0),
            new InstantCommand(()->elevator.setElevatorPositionDynamicConfigs(HUMAN_PLAYER_STATION_CENTIMETERS, MOTION_MAGIC_ELEVATOR_SLOWER_ACCLERATION, MOTION_MAGIC_ELEVATOR_VELOCITY, 0), elevator) //sets elevator back to the bottom position
        );

    }

}
