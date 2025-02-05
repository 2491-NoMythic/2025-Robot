

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

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import frc.robot.subsystems.AlgaeEndeffectorSubsystem;


public class PlaceCoralNoPath extends SequentialCommandGroup{

    
    public PlaceCoralNoPath(ElevatorSubsystem elevator, Supplier<ElevatorEnums> elevatorPose, DistanceSensors distanceSensors,
            DrivetrainSubsystem drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rSupplier,
            CoralEndeffectorSubsystem coralEndeffector, BooleanSupplier leftPlace,AlgaeEndeffectorSubsystem algaeEndeffectorSubsystem){
        addCommands(
            new ApproachReef(distanceSensors, drivetrain, xSupplier, ySupplier, rSupplier),//approaches reef while raising elevator
            new LineUp(drivetrain, leftPlace, 0.3),//align with reef
            new ParallelCommandGroup(
                new AlgaeIntakeCommand(algaeEndeffectorSubsystem,ALGAE_INTAKE_SPEED),
                new SequentialCommandGroup(
                    new ElevatorCommand(elevator, elevatorPose),//raises elevator to position
                    new DeliverCoral(coralEndeffector),//drops coral
                    new InstantCommand(()->elevator.setElevatorPosition(ElevatorEnums.Bottom), elevator) //sets elevator back to the bottom position
                )

            ),
            new ElevatorCommand(elevator, elevatorPose),//raises elevator to position
            new DeliverCoral(coralEndeffector),//drops coral
            new InstantCommand(()->elevator.setElevatorPosition(ElevatorEnums.Bottom), elevator) //sets elevator back to the bottom position
        );

    }

}