

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.settings.ElevatorEnums;
import frc.robot.subsystems.CoralEndeffectorSubsystem;
import frc.robot.subsystems.DistanceSensors;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import java.util.function.Supplier;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class PlaceCoralCommand extends SequentialCommandGroup{

    
    public PlaceCoralCommand(ElevatorSubsystem elevator, Supplier<ElevatorEnums> elevatorPose, DistanceSensors distanceSensors,
            DrivetrainSubsystem drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rSupplier,
            CoralEndeffectorSubsystem coralEndeffector, BooleanSupplier leftPlace){
        addCommands(
            new PathFindToReef(drivetrain, leftPlace),
            new PlaceCoralNoPath(elevator, elevatorPose, distanceSensors, drivetrain, xSupplier, ySupplier, rSupplier, coralEndeffector, leftPlace)
        );

    }

}