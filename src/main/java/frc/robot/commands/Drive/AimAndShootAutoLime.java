// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ShooterCommands.AutoAim;
import frc.robot.commands.ShooterCommands.Shoot;
import frc.robot.commands.ShooterCommands.StopShooter;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimAndShootAutoLime extends SequentialCommandGroup {
  /** Creates a new AimAndShootAutoLime. */
  public AimAndShootAutoLime(CommandSwerveDrivetrain drivetrain, Limelight limelight, Shooter mShooter, IntakeSubsystem mIntakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new AimWithLimelight(drivetrain, limelight).andThen(new AutoAim(limelight, mShooter).andThen(new Shoot(85, mShooter, mIntakeSubsystem)).andThen(new WaitCommand(0.5).andThen(new StopShooter(mShooter, mIntakeSubsystem)))));
  }
}
