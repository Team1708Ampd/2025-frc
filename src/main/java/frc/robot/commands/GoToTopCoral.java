// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GoToTopCoral extends Command {
  double targetAngle = 37.6;
  private final double HS_MOVEMENT_THRESHOLD = 7;
  private final double LS_MOVEMENT_THRESHOLD = 1.25;
  private final double FINAL_POSITION_THRESHOLD = 0.6;

  private final double HIGH_SPEED = 0.35;
  private final double LOW_SPEED = 0.2;
  private final double FINAL_SPEED = 0.10;
  private final double DOWN_SPEED = -0.08;
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
    if (Robot.coralSub.leftElevator.getRotorPosition().getValueAsDouble() > targetAngle) {
      Robot.coralSub.setElevator(DOWN_SPEED);
    } else {
      double difference = targetAngle - Robot.coralSub.leftElevator.getRotorPosition().getValueAsDouble();
      if (Math.abs(difference) > HS_MOVEMENT_THRESHOLD) {
        Robot.coralSub.setElevator(HIGH_SPEED);
      } else if (Math.abs(difference) > LS_MOVEMENT_THRESHOLD) {
        Robot.coralSub.setElevator(LOW_SPEED);
      } else {
        Robot.coralSub.setElevator(FINAL_SPEED);
      }
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
    if (Math.abs(targetAngle - Robot.coralSub.leftElevator.getRotorPosition().getValueAsDouble()) <= FINAL_POSITION_THRESHOLD) {
      return true;
    }
    return false;
  }
}
