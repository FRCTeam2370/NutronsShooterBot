// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//PathPlanner Online Library ---> https://3015rangerrobotics.github.io/pathplannerlib/PathplannerLib2024.json

package frc.robot;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.Drive.AimWithLimelight;
import frc.robot.commands.ShooterCommands.AimShooter;
import frc.robot.commands.ShooterCommands.AutoAim;
import frc.robot.commands.ShooterCommands.AutoAutoAim;
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
  public static double MaxSpeed = 3;//TunerConstants.kSpeedAt12VoltsMps; // kSpeedAt12VoltsMps desired top speed
  public static double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity
  private SlewRateLimiter xLimiter = new SlewRateLimiter(6.25);
  private SlewRateLimiter yLimiter = new SlewRateLimiter(6.25);

  public static PIDController LimelightTurnPID = new PIDController(1.5,0, 0);

  /* Setting up bindings for necessary control of the swerve drive platform */
  public static final CommandXboxController joystick = new CommandXboxController(0); // My joystick
  public static final GenericHID driver = new GenericHID(0);
   // My drivetrain

  private static final Shooter mShooter = new Shooter();
  private static final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  private static final LEDSubsystem mLEDSubsystem = new LEDSubsystem();
  private static final Limelight mLimelight = new Limelight();
  public static final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  

  public static POVButton upDpad = new POVButton(driver, 0);
  public static JoystickButton leftbumper = new JoystickButton(joystick.getHID(), 5);


  

   // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  public static void rotate(Supplier<Double> rotationVal){
    drivetrain.applyRequest(()-> CommandSwerveDrivetrain.drive.withRotationalRate(rotationVal.get()));
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> CommandSwerveDrivetrain.drive.withVelocityX(yLimiter.calculate(-joystick.getLeftY() * MaxSpeed)) // Drive forward with
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

    
    leftbumper.toggleOnTrue(new NoteIntake(mShooter, mIntakeSubsystem));//RunIndexer()
    joystick.rightBumper().whileTrue(new EjectPiece(mShooter, mIntakeSubsystem));

    joystick.rightTrigger().whileTrue(new Shoot(85, mShooter, mIntakeSubsystem));//80 is really good =)
    joystick.leftTrigger().toggleOnTrue(new ShooterIntake(mShooter));

    //joystick.b().whileTrue(new RunIntakeManual(0.5));

    joystick.x().toggleOnTrue(new AutoAutoAim(mLimelight, mShooter));//FIX AutoAim(); !!!!!!!!!

    joystick.b().onTrue(new Stow(mShooter));

    joystick.y().whileTrue(new AimShooter(mShooter, 2.38));

    joystick.leftStick().toggleOnTrue(new SliderAimShooter(mShooter));

    //joystick.rightStick().toggleOnTrue(new AutoAlign());

    //joystick.rightStick().toggleOnTrue(drivetrain.applyRequest(()-> CommandSwerveDrivetrain.drive.withVelocityX((-joystick.getLeftY() * MaxSpeed)* 0.75).withVelocityY((-joystick.getLeftX() * MaxSpeed)* 0.75).withRotationalRate(Limelight.getRotationLime().getDegrees())));
    joystick.rightStick().toggleOnTrue(new AimWithLimelight(drivetrain, mLimelight));

  }

  private static SendableChooser<Command> AutoChooser;

  public RobotContainer() {
    NamedCommands.registerCommand("Shoot", new Shoot(85, mShooter, mIntakeSubsystem));
    NamedCommands.registerCommand("Intake", new NoteIntake(mShooter, mIntakeSubsystem));
    NamedCommands.registerCommand("AutoAim", new AutoAutoAim(mLimelight, mShooter));
    NamedCommands.registerCommand("AutoAlign", new AimWithLimelight(drivetrain, mLimelight));

    configureBindings();

     AutoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", AutoChooser);
  }

  public Command getAutonomousCommand() {
    return AutoChooser.getSelected();
  }
}
