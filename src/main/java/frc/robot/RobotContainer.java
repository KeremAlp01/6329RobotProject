// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Auto;
import frc.robot.commands.CloseLoop.FeedWhenReady;
import frc.robot.commands.CloseLoop.SetArmAngle;
import frc.robot.commands.CloseLoop.SetArmAngleDistance;
import frc.robot.commands.CloseLoop.SetShooterRPM;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.OpenLoop.FeederOpenLoopCommand;
import frc.robot.commands.OpenLoop.ShooterOpenLoopCommand;
import frc.robot.subsystems.arm.ArmIO;
import frc.robot.subsystems.arm.ArmIOFalcon;
import frc.robot.subsystems.arm.ArmIOSim;
import frc.robot.subsystems.arm.ArmSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveState;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOFalcon;
import frc.robot.subsystems.feeder.FeederSubsystem;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.flywheel.FlywheelIO;
import frc.robot.subsystems.flywheel.FlywheelIOSim;
import frc.robot.subsystems.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOFalcon;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOKraken;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhoton;
import frc.robot.subsystems.vision.VisionSubsystem;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final Map<String, ChoreoTrajectory> trajMap;
  // Subsystems
  public static Drive drive;
  public static Flywheel flywheel;
  public static IntakeSubsystem intake;
  public static Shooter shooter;
  public static ArmSubsystem arm;
  public static FeederSubsystem feeder;
  public static VisionSubsystem vision;

  // Controller
  public static final CommandXboxController controller = new CommandXboxController(0);
  private final LoggedDashboardChooser<Command> autoChooser;
  private final SendableChooser<Command> autoChoose = new SendableChooser<Command>();

  // Dashboard inputs
  private final LoggedDashboardNumber flywheelSpeedInput =
      new LoggedDashboardNumber("Flywheel Speed", 3000.0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations

        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        flywheel = new Flywheel(new FlywheelIOTalonFX());
        intake = new IntakeSubsystem(new IntakeIOFalcon());
        shooter = new Shooter(new ShooterIOKraken());
        arm = new ArmSubsystem(new ArmIOFalcon());
        feeder = new FeederSubsystem(new FeederIOFalcon());
        vision = new VisionSubsystem(new VisionIOLimelight());

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        flywheel = new Flywheel(new FlywheelIOSim());
        intake = new IntakeSubsystem(new IntakeIOFalcon());
        shooter = new Shooter(new ShooterIOSim());
        arm = new ArmSubsystem(new ArmIOSim());
        feeder = new FeederSubsystem(new FeederIOFalcon());
        vision = new VisionSubsystem(new VisionIOPhoton());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        intake = new IntakeSubsystem(new IntakeIO() {});
        shooter = new Shooter(new ShooterIO() {});
        arm = new ArmSubsystem(new ArmIO() {});
        feeder = new FeederSubsystem(new FeederIO() {});
        break;
    }

    trajMap = loadTrajectories();
    Command testAuto = Auto.testAuto(trajMap, drive);
    // Set up auto routines
    NamedCommands.registerCommand(
        "Run Flywheel",
        Commands.startEnd(
                () -> flywheel.runVelocity(flywheelSpeedInput.get()), flywheel::stop, flywheel)
            .withTimeout(5.0));

    // Configure the button bindings

    autoChoose.addOption("TEST Auto", testAuto);
    autoChooser = new LoggedDashboardChooser<Command>("Auto Choices", autoChoose);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRawAxis(4)));
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller.b().whileTrue(new ShooterOpenLoopCommand(shooter, 4));
    // controller.b().onTrue(Commands.runOnce(() ->drive.setPose(new
    // Pose2d(drive.getPose().getTranslation(), new Rotation2d())),drive).ignoringDisable(true));
    // controller.a().whileTrue(new ArmOpenLoopCommand(arm, 4));
    // controller.y().whileTrue(new IntakeOpenLoopCommand(intake, 5));
    // controller.rightBumper().whileTrue(new FeederOpenLoopCommand(feeder, 6));
    // controller.leftBumper().onTrue(new SetArmAngle(arm, 100));
    // controller.y().onTrue(new SetArmAngle(arm, 54));
    controller.x().whileTrue(new SetShooterRPM(shooter, 5000, 5000, true));

    controller
        .a()
        .onTrue(
            new InstantCommand(
                () -> {
                  drive.setDriveState(DriveState.HEADINGLOCK);
                }))
        .onTrue(
            new InstantCommand(
                () -> {
                  drive.setWantsHeadingLock(true);
                }))
        .onTrue(
            new InstantCommand(
                    () -> {
                      drive.setTargetHeading(drive.getSpeakerAngle().getDegrees());
                    })
                .andThen(new SetArmAngle(arm, 90)))
        .onFalse(
            new InstantCommand(
                () -> {
                  drive.setDriveState(DriveState.TELEOP);
                }));

    controller
        .b()
        .whileTrue(getSpeakerShot())
        .whileFalse(
            new InstantCommand(
                () -> {
                  drive.setDriveState(DriveState.TELEOP);
                }));
    // controller.a().whileTrue(new FlywheelCommand(() -> flywheelSpeedInput.get(), flywheel));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private Map<String, ChoreoTrajectory> loadTrajectories() {
    Set<String> trajNames;
    try {
      if (Robot.isReal()) {
        trajNames = listFilesUsingFilesList("/home/lvuser/deploy/choreo");
      } else {
        trajNames = listFilesUsingFilesList("src/main/deploy/choreo");
      }
    } catch (IOException e) {
      DriverStation.reportError("Invalid Directory! Trajectories failed to load!", true);
      return null;
    }
    return trajNames.stream()
        .collect(
            Collectors.toMap(
                entry -> entry.replace(".traj", ""),
                entry -> Choreo.getTrajectory(entry.replace(".traj", ""))));
  }

  private Set<String> listFilesUsingFilesList(String dir) throws IOException {
    try (Stream<Path> stream = Files.list(Paths.get(dir))) {
      return stream
          .filter(file -> !Files.isDirectory(file))
          .map(Path::getFileName)
          .map(Path::toString)
          .collect(Collectors.toSet());
    }
  }

  public Command getAmpShot() {
    return new SetArmAngle(arm, 80)
        .andThen(new FeederOpenLoopCommand(feeder, 3))
        .alongWith(new ShooterOpenLoopCommand(shooter, -4));
  }

  public Command getSpeakerShot() {
    return new SetArmAngleDistance(arm, drive)
        .alongWith(new SetShooterRPM(shooter, 3000, 3000, false))
        .alongWith(
            new InstantCommand(
                () -> {
                  drive.setDriveState(DriveState.HEADINGLOCK);
                }))
        .alongWith(
            new InstantCommand(
                () -> {
                  drive.setWantsHeadingLock(true);
                }))
        .alongWith(
            new InstantCommand(
                () -> {
                  drive.setTargetHeading(drive.getSpeakerAngle().getDegrees());
                }))
        .alongWith(new FeedWhenReady(shooter, arm, feeder, drive));
  }

  public void setAutoHeading() {
    if (controller.getRawAxis(0) > 0 || controller.getRawAxis(0) < 0) {
      drive.setWantsHeadingLock(false);
    } else {
      drive.setWantsHeadingLock(true);
      drive.setTargetHeading(drive.getYaw());
    }
  }

  public boolean wantsKeepHeading() {
    if (controller.getRawAxis(4) > 0 || controller.getRawAxis(4) < 0) {
      drive.setWantsHeadingLock(false);
      return true;
    } else {
      drive.setWantsHeadingLock(true);
      drive.setTargetHeading(drive.getRawGyro());
      return false;
    }
  }
}
