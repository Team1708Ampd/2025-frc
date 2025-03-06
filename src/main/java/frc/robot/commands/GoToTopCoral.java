// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToTopCoral extends Command {
  /** Creates a new GoToBottomCoral. */
  public GoToTopCoral() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.coralSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Robot.coralSub.leftElevator.getRotorPosition().getValueAsDouble() < 20) {
      Robot.coralSub.setElevator(0.22);
    }
    else if (Robot.coralSub.leftElevator.getRotorPosition().getValueAsDouble() < 27) {
      Robot.coralSub.setElevator(0.12);
    } else if (Robot.coralSub.leftElevator.getRotorPosition().getValueAsDouble() > 37) {
      Robot.coralSub.setElevator(-0.06);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Robot.coralSub.setElevator(0.03);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(36.431 - Robot.coralSub.leftElevator.getRotorPosition().getValueAsDouble()) <= 0.6) {
      return true;
    }
    return false;
  }
}
