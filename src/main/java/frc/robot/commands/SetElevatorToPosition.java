package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetElevatorToPosition extends Command {
  private double targetAngle = 0;
  public static final double ELEVATOR_BOTTOM = 0;
  public static final double ELEVATOR_LOW_CORAL = 0.866;
  public static final double ELEVATOR_MID_CORAL = 2.526;
  public static final double ELEVATOR_HIGH_CORAL = 5.012;

  private final double HS_MOVEMENT_THRESHOLD = 4; // 1
  private final double LS_MOVEMENT_THRESHOLD = 0.3; // 0.3
  private final double FINAL_POSITION_THRESHOLD = 0.01; // 0.1

  private final double HIGH_SPEED = 0.6; // 0.6
  private final double LOW_SPEED = 0.4; // 0.4
  private final double FINAL_SPEED = 0.17; // 0.17
  private final double DOWN_SPEED = -0.28;
  /** Creates a new GoToBottomCoral. */
  public SetElevatorToPosition(double Target) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Robot.coralSub);
    targetAngle = Target;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((Robot.coralSub.elevatorEncoder.getPosition().getValueAsDouble()) > targetAngle) {
      Robot.coralSub.setElevator(DOWN_SPEED);
    } else {
      double difference = targetAngle - (Robot.coralSub.elevatorEncoder.getPosition().getValueAsDouble());
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
    if(targetAngle != 0) {
      Robot.coralSub.setElevator(0.03);
    } else {
      Robot.coralSub.setElevator(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (targetAngle == 0 && !Robot.coralSub.elevatorSwitch.get()) return true;

    boolean rc = false;

    if (Math.abs(targetAngle - (Robot.coralSub.elevatorEncoder.getPosition().getValueAsDouble())) <= FINAL_POSITION_THRESHOLD) 
    {
      rc = true;
    }

    return rc;
  }
}

