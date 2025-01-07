// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.settings.Constants.DriveConstants.*;
import static frc.robot.settings.Constants.PS4Driver.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Drive;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;
import java.io.IOException;
import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // preferences are information saved on the Rio. They are initialized once, then
  // gotten every time
  // we run the code.
  private final boolean useDetectorLimelight = Preferences.getBoolean("Detector Limelight", true);
  private final boolean useXboxController = Preferences.getBoolean("Xbox Controller", true);

  private DrivetrainSubsystem driveTrain;
  private Drive defaultDriveCommand;
  private Lights lights;
  private XboxController driverControllerXbox;
  private XboxController operatorControllerXbox;
  private PS4Controller driverControllerPS4;
  private PS4Controller operatorControllerPS4;
  private Limelight limelight;
  private SendableChooser<Command> autoChooser;
  private PowerDistribution PDP;

  Alliance currentAlliance;
  BooleanSupplier ZeroGyroSup;
  // Replace with CommandPS4Controller or CommandJoystick if needed

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // preferences are initialized IF they don't already exist on the Rio
    Preferences.initBoolean("Lights", true);
    Preferences.initBoolean("CompBot", true);
    Preferences.initBoolean("Detector Limelight", false);
    Preferences.initBoolean("Use Limelight", true);
    Preferences.initBoolean("Use 2 Limelights", true);
    Preferences.initBoolean("Xbox Controller", true);

    DataLogManager.start(); // Start logging
    DriverStation.startDataLog(DataLogManager.getLog()); // Joystick Data logging
    /*
     * the following code uses the Xbox Controller Preference to determine our controllers and all our bindings. any time you want to use/create a binding,
     * define a supplier as it in both conditions of this if()else{} code.
     */
    if (useXboxController) {
      driverControllerXbox = new XboxController(DRIVE_CONTROLLER_ID);
      operatorControllerXbox = new XboxController(OPERATOR_CONTROLLER_ID);

      ZeroGyroSup = driverControllerXbox::getStartButton;
    } else {
      driverControllerPS4 = new PS4Controller(DRIVE_CONTROLLER_ID);
      operatorControllerPS4 = new PS4Controller(OPERATOR_CONTROLLER_ID);

      ZeroGyroSup = driverControllerPS4::getPSButton;
    }

    limelightInit();
    driveTrainInst();
    lightsInst();

    configureDriveTrain();
    configureBindings(); // Configure the trigger bindings
    autoInit();
  }

  private void driveTrainInst() {
    driveTrain = new DrivetrainSubsystem();
    if (useXboxController) {
      defaultDriveCommand =
          new Drive(
              driveTrain,
              () -> false,
              () -> modifyAxis(-driverControllerXbox.getRawAxis(Y_AXIS), DEADBAND_NORMAL),
              () -> modifyAxis(-driverControllerXbox.getRawAxis(X_AXIS), DEADBAND_NORMAL),
              () -> modifyAxis(-driverControllerXbox.getRawAxis(Z_AXIS), DEADBAND_NORMAL));
      driveTrain.setDefaultCommand(defaultDriveCommand);
    } else {
      defaultDriveCommand =
          new Drive(
              driveTrain,
              () -> false,
              () -> modifyAxis(-driverControllerPS4.getRawAxis(Y_AXIS), DEADBAND_NORMAL),
              () -> modifyAxis(-driverControllerPS4.getRawAxis(X_AXIS), DEADBAND_NORMAL),
              () -> modifyAxis(-driverControllerPS4.getRawAxis(Z_AXIS), DEADBAND_NORMAL));
      driveTrain.setDefaultCommand(defaultDriveCommand);
    }
  }

  private void autoInit() {
    registerNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void limelightInit() {
    limelight = Limelight.getInstance();
  }

  private void lightsInst() {
    lights = new Lights();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    SmartDashboard.putData("drivetrain", driveTrain);
    new Trigger(ZeroGyroSup).onTrue(new InstantCommand(driveTrain::zeroGyroscope));

    InstantCommand setOffsets =
        new InstantCommand(driveTrain::setEncoderOffsets) {
          public boolean runsWhenDisabled() {
            return true;
          }
          ;
        };
    SmartDashboard.putData("set offsets", setOffsets);
    SmartDashboard.putData(new InstantCommand(driveTrain::forceUpdateOdometryWithVision));
    /*
     * bindings:
     * PS4: zero the gyroscope
     */
  }

  // Schedule `exampleMethodCommand` when the Xbox controller's B button is
  // pressed,
  // cancelling on release.

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  private void configureDriveTrain() {
    try {
      AutoBuilder.configure(
          driveTrain::getPose, // Pose2d supplier
          driveTrain
              ::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
          driveTrain::getChassisSpeeds,
          (speeds) -> driveTrain.drive(speeds),
          new PPHolonomicDriveController(
              new com.pathplanner.lib.config.PIDConstants(
                  k_XY_P, k_XY_I,
                  k_XY_D), // PID constants to correct for translation error (used to create the X
              // and Y PID controllers)
              new com.pathplanner.lib.config.PIDConstants(
                  k_THETA_P, k_THETA_I,
                  k_THETA_D) // PID constants to correct for rotation error (used to create the
              // rotation controller)
              ),
          RobotConfig.fromGUISettings(),
          () -> DriverStation.getAlliance().get().equals(Alliance.Red),
          driveTrain);
    } catch (org.json.simple.parser.ParseException a) {
      System.out.println("got ParseException trying to configure AutoBuilder");
    } catch (IOException b) {
      System.out.println("got IOException thrown trying to configure autobuilder");
    }
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void autonomousInit() {
    SmartDashboard.putNumber("autos ran", SmartDashboard.getNumber("autos ran", 0) + 1);
  }

  private double modifyAxis(double value, double deadband) {
    // Deadband
    value = MathUtil.applyDeadband(value, deadband);
    // Square the axis
    value = Math.copySign(value * value, value);
    return value;
  }

  private void registerNamedCommands() {}

  public void logPower() {
    for (int i = 0; i < 16; i++) {
      SmartDashboard.putNumber("PDP Current " + i, PDP.getCurrent(i));
    }
  }

  public void teleopInit() {}

  public void teleopPeriodic() {
    SmartDashboard.putData(driveTrain.getCurrentCommand());
  }

  public void robotPeriodic() {
    currentAlliance = DriverStation.getAlliance().get();
    SmartDashboard.putString(
        "AlliancePeriodic",
        currentAlliance == null ? "null" : currentAlliance == Alliance.Red ? "Red" : "Blue");
    if (Preferences.getBoolean("Use Limelight", false)) {
      limelight.updateLoggingWithPoses();
    }
  }

  public void disabledPeriodic() {}

  public void disabledInit() {}
}
