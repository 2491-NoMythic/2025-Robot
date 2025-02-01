// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.settings.Constants.AlgaeEndeffectorConstants.ALGAE_INTAKE_SPEED;
import static frc.robot.settings.Constants.AlgaeEndeffectorConstants.ALGAE_SHOOT_SPEED;
import static frc.robot.settings.Constants.DriveConstants.*;
import static frc.robot.settings.Constants.SensorConstants.*;
import static frc.robot.settings.Constants.PS4Driver.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.jni.TimeOfFlightJNI;

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
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlgaeEndeffectorCommand;
import frc.robot.commands.ApproachReef;
import frc.robot.commands.AlgaeIntakeCommand;
import frc.robot.commands.Drive;
import frc.robot.commands.IndicatorLights;
import frc.robot.commands.ElevatorCommand;
import frc.robot.commands.LineUp;
import frc.robot.commands.MoveMeters;
import frc.robot.commands.PlaceCoralNoPath;
import frc.robot.commands.WaitCommand;
import frc.robot.subsystems.DistanceSensors;
import frc.robot.commands.NamedCommands.CoralIntake;
import frc.robot.commands.NamedCommands.DeliverCoral;
import frc.robot.settings.SensorNameEnums;
import frc.robot.settings.CommandSelectorEnum;
import frc.robot.settings.ElevatorEnums;
import frc.robot.settings.ReefSideEnum;
import frc.robot.subsystems.AlgaeEndeffectorSubsystem;
import frc.robot.subsystems.CoralEndeffectorSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FunnelIntake;
import frc.robot.subsystems.FunnelRotator;
import frc.robot.subsystems.CimberSubsystem;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.RobotState;

import java.io.IOException;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.json.simple.parser.ParseException;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // preferences are information saved on the Rio. They are initialized once, then
  // gotten every time
  // we run the code.
  private final boolean useDetectorLimelight = Preferences.getBoolean("Detector Limelight", true);
  private final boolean useXboxController = Preferences.getBoolean("Xbox Controller", true);
  private final boolean coralIntakeExists = Preferences.getBoolean("CoralIntake", true);
  private final boolean algaeIntakeExists = Preferences.getBoolean("AlgaeIntake", true);
  private final boolean algaeEndeffectorExists = Preferences.getBoolean("AlgaeEndDefector", true);
  private final boolean coralEndeffectorExists = Preferences.getBoolean("CoralEndDefector", true);
  private final boolean climberExists = Preferences.getBoolean("Climber", true);
  private final boolean elevatorExists = Preferences.getBoolean("Elevator", true);
  private final boolean funnelIntakeExists = Preferences.getBoolean("FunnelIntake", true);
  private final boolean funnelRotatorExists = Preferences.getBoolean("FunnelRotator", true);
  private final boolean DrivetrainExists = Preferences.getBoolean("DrivetrainExists", true);
  private final boolean distanceSensorsExist = Preferences.getBoolean("DistanceSensors Exist", true);
  private final boolean lightsExist = Preferences.getBoolean("Lights Exist", true);

  private DrivetrainSubsystem driveTrain;
  private ElevatorCommand elevatorDefaultCommand;
  private Drive defaultDriveCommand;
  private Lights lights;
  private XboxController driverControllerXbox;
  private XboxController operatorControllerXbox;
  private PS4Controller driverControllerPS4;
  private PS4Controller operatorControllerPS4;
  private Limelight limelight;
  private SendableChooser<Command> autoChooser;
  private PowerDistribution PDP;
  private DistanceSensors distanceSensors;
  private CoralEndeffectorSubsystem coralEndDefector;
  private AlgaeEndeffectorSubsystem algaeEndDefector;
  private CimberSubsystem climber;
  private ElevatorSubsystem elevator;
  private CoralIntake coralIntake;
  private FunnelIntake funnelIntake;
  private FunnelRotator funnelRotator;
  private DeliverCoral deliverCoral;
  private ApproachReef approachReef;

  RobotState robotState;
  Alliance currentAlliance;
  BooleanSupplier ZeroGyroSup;
  BooleanSupplier LeftReefLineupSup;
  BooleanSupplier RightReefLineupSup;
  BooleanSupplier SlowFrontSup;
  BooleanSupplier AlgaeIntakeSup;
  BooleanSupplier AlgaeShooterSup;
  BooleanSupplier ReefHeight1Supplier;
  BooleanSupplier ReefHeight2Supplier;
  BooleanSupplier ReefHeight3Supplier;
  BooleanSupplier ReefHeight4Supplier;
  BooleanSupplier CoralPlaceTeleSupplier;
  BooleanSupplier CoralIntakeHeightSupplier;
  DoubleSupplier ControllerYAxisSupplier;
  DoubleSupplier ControllerXAxisSupplier;
  DoubleSupplier ControllerZAxisSupplier;
  Command pathFindToReef;

  // Replace with CommandPS4Controller or CommandJoystick if needed

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // preferences are initialized IF they don't already exist on the Rio
    Preferences.initBoolean("Lights Exist", true);
    Preferences.initBoolean("CompBot", true);
    Preferences.initBoolean("Detector Limelight", false);
    Preferences.initBoolean("Use Limelight", true);
    Preferences.initBoolean("Use 2 Limelights", true);
    Preferences.initBoolean("Xbox Controller", true);
    Preferences.initBoolean("CoralIntake", false);
    Preferences.initBoolean("AlgaeIntake", false);
    Preferences.initBoolean("Elevator", false);
    Preferences.initBoolean("CoralEndDefector", false);
    Preferences.initBoolean("AlgaeEndDefector", false);
    Preferences.initBoolean("FunnelIntake", false);
    Preferences.initBoolean("FunnelRotator", false);
    Preferences.initBoolean("Climber", false);
    Preferences.initBoolean("DrivetrainExists", false);
    Preferences.initBoolean("AntiTipActive", true);
    Preferences.initBoolean("DistanceSensors Exist", true);

    DataLogManager.start(); // Start logging
    DriverStation.startDataLog(DataLogManager.getLog()); // Joystick Data logging
    /*
     * the following code uses the Xbox Controller Preference to determine our
     * controllers and all our bindings. any time you want to use/create a binding,
     * define a supplier as it in both conditions of this if()else{} code.
     */
    if (useXboxController) {
      driverControllerXbox = new XboxController(DRIVE_CONTROLLER_ID);
      operatorControllerXbox = new XboxController(OPERATOR_CONTROLLER_ID);

      ControllerXAxisSupplier = () -> modifyAxis(-driverControllerXbox.getRawAxis(1), DEADBAND_NORMAL);
      ControllerYAxisSupplier = () -> modifyAxis(-driverControllerXbox.getRawAxis(0), DEADBAND_NORMAL);
      ControllerZAxisSupplier = () -> modifyAxis(-driverControllerXbox.getRawAxis(XBOX_Z_AXIS), DEADBAND_NORMAL);
      
      ZeroGyroSup = driverControllerXbox::getStartButton;
      LeftReefLineupSup = driverControllerXbox::getLeftBumperButton;
      RightReefLineupSup =  driverControllerXbox::getRightBumperButton;
      SlowFrontSup = ()-> driverControllerXbox.getRightTriggerAxis() > 0.1;
      AlgaeIntakeSup = driverControllerXbox::getAButton; //TODO change to actual
      AlgaeShooterSup = driverControllerXbox::getXButton;
      CoralPlaceTeleSupplier = driverControllerXbox::getYButton;

      ReefHeight1Supplier = ()->operatorControllerXbox.getPOV() == 0;
      ReefHeight2Supplier = ()->operatorControllerXbox.getPOV() == 90;
      ReefHeight3Supplier = ()->operatorControllerXbox.getPOV() == 180;
      ReefHeight4Supplier = ()->operatorControllerXbox.getPOV() == 270;
      CoralIntakeHeightSupplier = ()->operatorControllerXbox.getStartButton();
      
    } else {
      driverControllerPS4 = new PS4Controller(DRIVE_CONTROLLER_ID);
      operatorControllerPS4 = new PS4Controller(OPERATOR_CONTROLLER_ID);

      ControllerXAxisSupplier = () -> modifyAxis(-driverControllerPS4.getRawAxis(X_AXIS), DEADBAND_NORMAL);
      ControllerYAxisSupplier = () -> modifyAxis(-driverControllerPS4.getRawAxis(Y_AXIS), DEADBAND_NORMAL);
      ControllerZAxisSupplier = () -> modifyAxis(-driverControllerPS4.getRawAxis(PS4_Z_AXIS), DEADBAND_NORMAL);

      LeftReefLineupSup = operatorControllerPS4::getL1Button;
      RightReefLineupSup = operatorControllerPS4::getR1Button;
      SlowFrontSup = ()->driverControllerPS4.getR2Axis()>-0.5;
      AlgaeIntakeSup = driverControllerPS4::getCrossButton; //TODO change to actual
      AlgaeShooterSup = driverControllerPS4::getSquareButton;
      CoralPlaceTeleSupplier = driverControllerPS4::getTriangleButton;

      ZeroGyroSup = driverControllerPS4::getPSButton;

      ReefHeight1Supplier = ()->operatorControllerPS4.getPOV() == 0;
      ReefHeight2Supplier = ()->operatorControllerPS4.getPOV() == 90;
      ReefHeight3Supplier = ()->operatorControllerPS4.getPOV() == 180;
      ReefHeight4Supplier = ()->operatorControllerPS4.getPOV() == 270;
      CoralIntakeHeightSupplier = ()->operatorControllerPS4.getOptionsButton();
    }

    limelightInit();
    sensorInit();     
    if (DrivetrainExists) {
      driveTrainInst();
      configureDriveTrain();
      commandSelectorInst();
    }
    if (lightsExist) {lightsInst();}
    
    if (coralEndeffectorExists) {coralEndDefectorInst();}
    if (algaeEndeffectorExists) {algaeEndDefectorInst();}
    if (climberExists) {climberInst();}
    if (elevatorExists) {elevatorInst();}
    if (funnelIntakeExists) {funnelIntakeInst();}
    if (funnelRotatorExists) {funnelRotatorInst();}
    
    configureBindings(); // Configure the trigger bindings
    autoInit();
  }

  private void driveTrainInst() {
    driveTrain = new DrivetrainSubsystem();

    defaultDriveCommand = new Drive(
        driveTrain,
        () -> false,
        ControllerXAxisSupplier,
        ControllerYAxisSupplier,
        ControllerZAxisSupplier);
    driveTrain.setDefaultCommand(defaultDriveCommand);
    
    approachReef = new ApproachReef(
      distanceSensors,
      driveTrain,
      ControllerXAxisSupplier,
      ControllerYAxisSupplier,
      ControllerZAxisSupplier);
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
    lights.setDefaultCommand(new IndicatorLights(lights));
  }
  
  private void sensorInit() {
    distanceSensors = new DistanceSensors();
  }

  private void coralEndDefectorInst() {
    coralEndDefector = new CoralEndeffectorSubsystem();
  }

  private void algaeEndDefectorInst() {
    algaeEndDefector = new AlgaeEndeffectorSubsystem();
  }

  private void climberInst() {
    climber = new CimberSubsystem();
  }

  private void elevatorInst() {
    elevator = new ElevatorSubsystem();
    elevatorDefaultCommand = new ElevatorCommand(elevator,()-> ElevatorEnums.HumanPlayer);
  }

  private void funnelIntakeInst() {
    funnelIntake = new FunnelIntake();
  }

  private void funnelRotatorInst() {
    funnelRotator = new FunnelRotator();
  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
//all the triggers that change RobotState
    new Trigger(LeftReefLineupSup).onTrue(new InstantCommand(()->RobotState.getInstance().deliveringLeft = true));
    new Trigger(RightReefLineupSup).onTrue(new InstantCommand(()->RobotState.getInstance().deliveringLeft = false));
    new Trigger(ReefHeight1Supplier).onTrue(new InstantCommand(()->RobotState.getInstance().deliveringCoralHeight = ElevatorEnums.Reef1));
    new Trigger(ReefHeight2Supplier).onTrue(new InstantCommand(()->RobotState.getInstance().deliveringCoralHeight = ElevatorEnums.Reef2));
    new Trigger(ReefHeight3Supplier).onTrue(new InstantCommand(()->RobotState.getInstance().deliveringCoralHeight = ElevatorEnums.Reef3));
    new Trigger(ReefHeight4Supplier).onTrue(new InstantCommand(()->RobotState.getInstance().deliveringCoralHeight = ElevatorEnums.Reef4));

    if (DrivetrainExists){
    SmartDashboard.putData("drivetrain", driveTrain);

    new Trigger(ZeroGyroSup).onTrue(new InstantCommand(driveTrain::zeroGyroscope));
    if(!(elevatorExists&&coralEndeffectorExists&&distanceSensorsExist)) {
      new Trigger(CoralPlaceTeleSupplier).whileTrue(pathFindToReef);
    }
    
    new Trigger(SlowFrontSup).whileTrue(approachReef);
    InstantCommand setOffsets = new InstantCommand(driveTrain::setEncoderOffsets) {
      public boolean runsWhenDisabled() {
        return true;
      };
    };
        
        SmartDashboard.putData("set offsets", setOffsets);
    SmartDashboard.putData(new InstantCommand(driveTrain::forceUpdateOdometryWithVision));
    SmartDashboard.putNumber("speedOut", ALGAE_SHOOT_SPEED);
    SmartDashboard.putNumber("speedIn", ALGAE_INTAKE_SPEED);


    }
    if (algaeEndeffectorExists) {
      new Trigger(AlgaeIntakeSup).whileTrue(new AlgaeIntakeCommand(algaeEndDefector, ALGAE_INTAKE_SPEED));
      new Trigger(AlgaeShooterSup).whileTrue(new AlgaeIntakeCommand(algaeEndDefector, ALGAE_SHOOT_SPEED));
    }

    if(elevatorExists && coralEndeffectorExists && DrivetrainExists && distanceSensorsExist){
      new Trigger(CoralPlaceTeleSupplier).whileTrue(
          new SequentialCommandGroup(
            pathFindToReef,
            new PlaceCoralNoPath(
              elevator,
              ()->RobotState.getInstance().deliveringCoralHeight,
              distanceSensors,
              driveTrain,
              ControllerXAxisSupplier,
              ControllerYAxisSupplier,
              ControllerZAxisSupplier,
              coralEndDefector,
              ()->RobotState.getInstance().deliveringLeft))
          );
    }
    /*
     * bindings:
     * PS4: zero the gyroscope
     */
  }

  // Schedule `exampleMethodCommand` when the Xbox controller's B button is
  // pressed,
  // cancelling on release.

  private void configureDriveTrain() {
    try {
      AutoBuilder.configure(
          driveTrain::getPose, // Pose2d supplier
          driveTrain::resetOdometry, // Pose2d consumer, used to reset odometry at the beginning of auto
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

  private void registerNamedCommands() {
    Command coralIntakeNamedCommand;
    Command deliverCoralLeft1NamedCommand;
    Command deliverCoralLeft2NamedCommand;
    Command deliverCoralLeft3NamedCommand;
    Command deliverCoralLeft4NamedCommand;
    Command deliverCoralRight1NamedCommand;
    Command deliverCoralRight2NamedCommand;
    Command deliverCoralRight3NamedCommand;
    Command deliverCoralRight4NamedCommand;
    Command elevatorResetNamedCommand;
    if(elevatorExists&&funnelIntakeExists&&coralEndeffectorExists) {
      coralIntake = new CoralIntake(elevator, funnelIntake, coralEndDefector);
      coralIntakeNamedCommand = coralIntake;
      deliverCoralLeft1NamedCommand = new PlaceCoralNoPath(elevator, ()->ElevatorEnums.Reef1, distanceSensors, driveTrain, ()->0, ()->0, ()->0, coralEndDefector, ()->true);
      deliverCoralLeft2NamedCommand = new PlaceCoralNoPath(elevator, ()->ElevatorEnums.Reef2, distanceSensors, driveTrain, ()->0, ()->0, ()->0, coralEndDefector, ()->true);
      deliverCoralLeft3NamedCommand = new PlaceCoralNoPath(elevator, ()->ElevatorEnums.Reef3, distanceSensors, driveTrain, ()->0, ()->0, ()->0, coralEndDefector, ()->true);
      deliverCoralLeft4NamedCommand = new PlaceCoralNoPath(elevator, ()->ElevatorEnums.Reef4, distanceSensors, driveTrain, ()->0, ()->0, ()->0, coralEndDefector, ()->true);
      deliverCoralRight1NamedCommand = new PlaceCoralNoPath(elevator, ()->ElevatorEnums.Reef1, distanceSensors, driveTrain, ()->0, ()->0, ()->0, coralEndDefector, ()->false);
      deliverCoralRight2NamedCommand = new PlaceCoralNoPath(elevator, ()->ElevatorEnums.Reef2, distanceSensors, driveTrain, ()->0, ()->0, ()->0, coralEndDefector, ()->false);
      deliverCoralRight3NamedCommand = new PlaceCoralNoPath(elevator, ()->ElevatorEnums.Reef3, distanceSensors, driveTrain, ()->0, ()->0, ()->0, coralEndDefector, ()->false);
      deliverCoralRight4NamedCommand = new PlaceCoralNoPath(elevator, ()->ElevatorEnums.Reef4, distanceSensors, driveTrain, ()->0, ()->0, ()->0, coralEndDefector, ()->false);
    } else {
      coralIntakeNamedCommand = new InstantCommand(()->System.out.println("attempted to create named command but subsytem did not exist"));
      deliverCoralLeft1NamedCommand = new InstantCommand(()->System.out.println("attempted to create named command but subsytem did not exist"));
      deliverCoralLeft2NamedCommand = new InstantCommand(()->System.out.println("attempted to create named command but subsytem did not exist"));
      deliverCoralLeft3NamedCommand = new InstantCommand(()->System.out.println("attempted to create named command but subsytem did not exist"));
      deliverCoralLeft4NamedCommand = new InstantCommand(()->System.out.println("attempted to create named command but subsytem did not exist"));
      deliverCoralRight1NamedCommand = new InstantCommand(()->System.out.println("attempted to create named command but subsytem did not exist"));
      deliverCoralRight2NamedCommand = new InstantCommand(()->System.out.println("attempted to create named command but subsytem did not exist"));
      deliverCoralRight3NamedCommand = new InstantCommand(()->System.out.println("attempted to create named command but subsytem did not exist"));
      deliverCoralRight4NamedCommand = new InstantCommand(()->System.out.println("attempted to create named command but subsytem did not exist"));
    }

    if(elevatorExists) {
      elevatorResetNamedCommand = new InstantCommand(()->elevator.setElevatorPosition(ElevatorEnums.HumanPlayer));
    } else {
      elevatorResetNamedCommand = new InstantCommand(()->System.out.println("attempted to create named command but subsytem did not exist"));
    }

    NamedCommands.registerCommand("CoralIntake", coralIntakeNamedCommand);
    NamedCommands.registerCommand("DeliverCoralLeft1", deliverCoralLeft1NamedCommand);
    NamedCommands.registerCommand("DeliverCoralLeft2", deliverCoralLeft2NamedCommand);
    NamedCommands.registerCommand("DeliverCoralLeft3", deliverCoralLeft3NamedCommand);
    NamedCommands.registerCommand("DeliverCoralLeft4", deliverCoralLeft4NamedCommand);
    NamedCommands.registerCommand("DeliverCoralRight1", deliverCoralRight1NamedCommand);
    NamedCommands.registerCommand("DeliverCoralRight2", deliverCoralRight2NamedCommand);
    NamedCommands.registerCommand("DeliverCoralRight3", deliverCoralRight3NamedCommand);
    NamedCommands.registerCommand("DeliverCoralRight4", deliverCoralRight4NamedCommand);
    NamedCommands.registerCommand("ElevatorReset", elevatorResetNamedCommand);
  }

  public void logPower() {
    for (int i = 0; i < 16; i++) {
      SmartDashboard.putNumber("PDP Current " + i, PDP.getCurrent(i));
    }
  }

  private void commandSelectorInst() {
    try {
      pathFindToReef = new SelectCommand<>(
        Map.ofEntries(
          Map.entry(CommandSelectorEnum.FrontCenterReefLeft, AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupA"), DEFAULT_PATH_CONSTRAINTS)),
          Map.entry(CommandSelectorEnum.FrontCenterReefRight, AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupB"), DEFAULT_PATH_CONSTRAINTS)),
          Map.entry(CommandSelectorEnum.FrontRightReefLeft, AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupC"), DEFAULT_PATH_CONSTRAINTS)),
          Map.entry(CommandSelectorEnum.FrontRightReefRight, AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupD"), DEFAULT_PATH_CONSTRAINTS)),
          Map.entry(CommandSelectorEnum.BackRightReefLeft, AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupE"), DEFAULT_PATH_CONSTRAINTS)),
          Map.entry(CommandSelectorEnum.BackRightReefRight, AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupF"), DEFAULT_PATH_CONSTRAINTS)),
          Map.entry(CommandSelectorEnum.BackCenterReefLeft, AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupG"), DEFAULT_PATH_CONSTRAINTS)),
          Map.entry(CommandSelectorEnum.BackCenterReefRight, AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupH"), DEFAULT_PATH_CONSTRAINTS)),
          Map.entry(CommandSelectorEnum.BackLeftReefLeft, AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupI"), DEFAULT_PATH_CONSTRAINTS)),
          Map.entry(CommandSelectorEnum.BackLeftReefRight, AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupJ"), DEFAULT_PATH_CONSTRAINTS)),
          Map.entry(CommandSelectorEnum.FrontLeftReefLeft, AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupK"), DEFAULT_PATH_CONSTRAINTS)),
          Map.entry(CommandSelectorEnum.FrontLeftReefRight, AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile("LineupL"), DEFAULT_PATH_CONSTRAINTS)),
          Map.entry(CommandSelectorEnum.NoClosestSide, new InstantCommand(()->System.out.println("No Closest Reef Side")))
          ), 
          ()->selectCommand(()->RobotState.getInstance().deliveringLeft));
        } catch (FileVersionException | IOException | ParseException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
          pathFindToReef = new InstantCommand(()->System.out.println("Error thrown while creating path!!"));
        }
  }
  public static CommandSelectorEnum selectCommand(BooleanSupplier LeftSupplier) {
    switch(RobotState.getInstance().closestReefSide) {
      case backCenter:
        if(LeftSupplier.getAsBoolean()) {
          return CommandSelectorEnum.BackCenterReefLeft;
        } else {
          return CommandSelectorEnum.BackCenterReefRight;
        }
      case backRight:
        if(LeftSupplier.getAsBoolean()) {
          return CommandSelectorEnum.BackRightReefLeft;
        } else {
          return CommandSelectorEnum.BackRightReefRight;
        }
      case backLeft:
        if(LeftSupplier.getAsBoolean()) {
          return CommandSelectorEnum.BackLeftReefLeft;
        } else {
          return CommandSelectorEnum.BackLeftReefRight;
        }
      case frontCenter:
        if(LeftSupplier.getAsBoolean()) {
          return CommandSelectorEnum.FrontCenterReefLeft;
        } else { 
          return CommandSelectorEnum.FrontCenterReefRight;
        }
      case frontRight:
        if(LeftSupplier.getAsBoolean()) {
          return CommandSelectorEnum.FrontRightReefLeft;
        } else { 
          return CommandSelectorEnum.FrontRightReefRight;
        }
      case frontLeft:
        if(LeftSupplier.getAsBoolean()) {
          return CommandSelectorEnum.FrontLeftReefLeft;
        } else { 
          return CommandSelectorEnum.FrontLeftReefRight;
        }
      default:
          return CommandSelectorEnum.NoClosestSide;
    } 

}

  public void teleopInit() {
  }

  public void teleopPeriodic() {
    if(DrivetrainExists) {
      SmartDashboard.putData(driveTrain.getCurrentCommand());
    }
  }
  public void robotInit(){
    if (elevatorExists){
      elevator.setZero(distanceSensors.getDistance(SensorNameEnums.Elevator));
    }
  }
  public void robotPeriodic() {
    currentAlliance = DriverStation.getAlliance().get();
    SmartDashboard.putString(
        "AlliancePeriodic",
        currentAlliance == null ? "null" : currentAlliance == Alliance.Red ? "Red" : "Blue");
    if (Preferences.getBoolean("Use Limelight", false)) {
      limelight.updateLoggingWithPoses();
    }
    SmartDashboard.putBoolean("REEFLINEUP/deliveringLeft", RobotState.getInstance().deliveringLeft);
  }

  public void disabledPeriodic() {
  }

  public void disabledInit() {
  }
}
