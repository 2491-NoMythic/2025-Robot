

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.NamedCommands.DeliverCoral;
import frc.robot.subsystems.CoralEndeffectorSubsystem;
import frc.robot.subsystems.DistanceSensors;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.NamedCommands.DeliverCoral;
import frc.robot.commands.ApproachReef;
import frc.robot.commands.LineUp;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.NamedCommands;


public class PlaceCoralCommand extends SequentialCommandGroup{

    
    public PlaceCoralCommand(ElevatorSubsystem elevator,double elevatorPosition, DistanceSensors distanceSensors,
            DrivetrainSubsystem drivetrain, DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier rSupplier,
            CoralEndeffectorSubsystem coralEndeffector, BooleanSupplier leftPlace){
        addCommands(
            new InstantCommand(()->elevator.setElevatorPosition(elevatorPosition), elevator),//moves elevator
            new ApproachReef(distanceSensors, drivetrain,xSupplier, ySupplier, rSupplier),//aproches the reef
            new LineUp(drivetrain, leftPlace),//align with reef
            new DeliverCoral(coralEndeffector)//drops coral
        );

    }

}