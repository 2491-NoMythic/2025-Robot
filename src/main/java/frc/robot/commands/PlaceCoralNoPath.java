

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
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


public class PlaceCoralNoPath extends SequentialCommandGroup{

    /**
     * This command drives forwards, raises the elevator, and drops coral on the reef.
     * It's designed to work without odometry.
     * @param elevator
     * @param elevatorPose
     * @param distanceSensors
     * @param drivetrain
     * @param xSupplier
     * @param ySupplier
     * @param rSupplier
     * @param coralEndeffector
     * @param leftPlace
     * @param algaeEndeffectorSubsystem
     * @param goForAlgae
     */
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
        BooleanSupplier goForAlgae)
    {
        addCommands(
            new ParallelRaceGroup(
                new AlgaeIntakeCommand(algaeEndeffectorSubsystem, () -> goForAlgae.getAsBoolean() ? ALGAE_INTAKE_SPEED : -0.5),
                new Drive(drivetrain, ()->true, ()->0.1, ()->0, ()->0),
                new SequentialCommandGroup(
                    new ParallelRaceGroup(
                        new ElevatorCommand(elevator, elevatorPose),//raises elevator to position)
                        new WaitUntil(()->elevator.isElevatorAtPose())),
                    new ParallelRaceGroup(
                        new DeliverCoral(coralEndeffector),//drops coral
                        new WaitCommand(()->0.75)))
            ),
            new InstantCommand(()->System.out.println("Reef Lineup Ended!")) //sets elevator back to the bottom position
        );

    }

}
