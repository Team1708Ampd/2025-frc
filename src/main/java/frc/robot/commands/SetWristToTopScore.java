// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetWristToTopScore extends Command {
  /** Creates a new SetWristToScore. */
  public SetWristToTopScore() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.coralSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.coralSub.wrist.getRotorPosition().getValueAsDouble() < 8) {
      Robot.coralSub.setWrist(0.09);
    } else if (Robot.coralSub.wrist.getRotorPosition().getValueAsDouble() > 9) {
      Robot.coralSub.setWrist(-0.09);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.coralSub.setWrist(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(8.7 - Robot.coralSub.wrist.getRotorPosition().getValueAsDouble()) <= 0.5) {
      return true;
    }
    return false;
  }
}
