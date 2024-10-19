// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.LEDCommands.LEDColor;
import frc.robot.commands.LEDCommands.StopLEDs;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class NoteIntake extends SequentialCommandGroup {
  /** Creates a new NoteIntake. */
  public NoteIntake(Shooter mShooter, IntakeSubsystem mIntakeSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new RunIndexerTilSight(mShooter, mIntakeSubsystem), new LEDColor().alongWith(new WaitCommand(2)), new StopLEDs());

  }
}
