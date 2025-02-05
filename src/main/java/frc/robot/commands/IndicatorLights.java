
package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.RobotState;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IndicatorLights extends Command {
  /** Creates a new IndicatorLights. */

  Lights lights;
  Timer timer;

  public IndicatorLights(Lights lights) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(lights);
    this.lights =lights;
    timer = new Timer();
    timer.start();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean limelightsUpdated = RobotState.getInstance().LimelightsUpdated;
    boolean coralSeen = RobotState.getInstance().coralSeen;
    double coralTimer = timer.get();

    if(limelightsUpdated) {
      //TODO: adjust these when we know what the lights are like
      lights.setLights(0, 10, 0, 40, 0);
    } else {
      //TODO: adjust these when we know what the lights are like
      lights.setLights(0, 10, 50, 0, 0);
    }
    if (coralSeen) {
        if (coralTimer<2) {
            if (coralTimer%0.2==0.1) {
                lights.setLights(0, 10, 255, 255, 255);
            }
            else {
                lights.setLights(0, 10, 0, 0, 0);
            }
        }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    lights.lightsOut();
    lights.dataSetter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}