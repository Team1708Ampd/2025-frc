package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetWristToAngle extends Command{
    double targetAngle;
    private final double HS_MOVEMENT_THRESHOLD = 25; // Threshold to move at high speed
    private final double LS_MOVEMENT_THRESHOLD = 10; // Threshold to move at low speed
    private final double FINAL_POSITION_THRESHOLD = 2; // threshold for terminating command

    private final double HIGH_SPEED_NORMALIZED = 0.3;
    private final double LOW_SPEED_NORMALIZED = 0.2;

    /** Creates a new SetWristToScore. */
    public SetWristToAngle(double angle) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(Robot.coralSub);

        targetAngle = angle;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {

        // Initialize speed to 0 
        Robot.coralSub.setWrist(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        // Get the position difference
        double difference = (targetAngle - Robot.coralSub.wrist.getRotorPosition().getValueAsDouble());
        double speedSetpoint = 0;

        if (Math.abs(difference) > HS_MOVEMENT_THRESHOLD)
        {
            if (difference > 0)
            {
                speedSetpoint = -(HIGH_SPEED_NORMALIZED);
            }
            else
            {
                speedSetpoint = HIGH_SPEED_NORMALIZED;
            }
        }
        else if (Math.abs(difference) > LS_MOVEMENT_THRESHOLD)
        {

        }
        else
        {
            // Move very slowly 
        }

        if (Robot.coralSub.wrist.getRotorPosition().getValueAsDouble() < 4) {
        Robot.coralSub.setWrist(0.1);
        } else if (Robot.coralSub.wrist.getRotorPosition().getValueAsDouble() > 5.5) {
        Robot.coralSub.setWrist(-0.1);
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
        if (Math.abs(4.77 - Robot.coralSub.wrist.getRotorPosition().getValueAsDouble()) <= 0.6) {
        return true;
        }
        return false;
    }
}
