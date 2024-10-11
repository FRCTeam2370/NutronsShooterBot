// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.ShooterCommands.AimShooter;
import frc.robot.commands.ShooterCommands.EjectPiece;
import frc.robot.commands.ShooterCommands.NoteIntake;
import frc.robot.commands.ShooterCommands.Shoot;
import frc.robot.commands.ShooterCommands.ShooterIntake;
import frc.robot.commands.ShooterCommands.SliderAimShooter;
import frc.robot.commands.ShooterCommands.Stow;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;


public class RobotContainer {
  private double MaxSpeed = 3;//TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private SlewRateLimiter xLimiter = new SlewRateLimiter(6.25);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(6.25);

  public static PIDController LimelightTurnPID = new PIDController(1.5,0, 0);

  /* Setting up bindings for necessary control of the swerve drive platform */
  private static final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public static final GenericHID driver = new GenericHID(0);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private static final Shooter mShooter = new Shooter();
  private static final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  private static final LEDSubsystem mLEDSubsystem = new LEDSubsystem();
  private static final Limelight mLimelight = new Limelight();

  public static POVButton upDpad = new POVButton(driver, 0);
  public static JoystickButton leftbumper = new JoystickButton(joystick.getHID(), 5);


  

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(yLimiter.calculate(-joystick.getLeftY() * MaxSpeed)) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(xLimiter.calculate(-joystick.getLeftX() * MaxSpeed)) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // joystick.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // reset the field-centric heading on left bumper press
    joystick.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);

    
    leftbumper.toggleOnTrue(new NoteIntake());//RunIndexer()
    joystick.rightBumper().whileTrue(new EjectPiece());

    joystick.rightTrigger().whileTrue(new Shoot(85, mShooter, mIntakeSubsystem));//80 is really good =)
    joystick.leftTrigger().toggleOnTrue(new ShooterIntake());

    //joystick.b().whileTrue(new RunIntakeManual(0.5));

    joystick.x().onTrue(new AimShooter(mShooter, 2.5));

    joystick.b().onTrue(new Stow());

    joystick.y().whileTrue(new AimShooter(mShooter, 4.2));

    joystick.leftStick().toggleOnTrue(new SliderAimShooter());

    //joystick.rightStick().toggleOnTrue(new AutoAlign());

    joystick.rightStick().toggleOnTrue(drivetrain.applyRequest(()-> drive.withVelocityX((-joystick.getLeftY() * MaxSpeed)* 0.75).withVelocityY((-joystick.getLeftX() * MaxSpeed)* 0.75).withRotationalRate(LimelightTurnPID.calculate(Limelight.txSlowly()))));
    
  }

  public RobotContainer() {
    configureBindings();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
