package frc.robot.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class SetWristToAngle extends Command{
    double targetAngle;
    private final double HS_MOVEMENT_THRESHOLD = 8; // Threshold to move at high speed
    private final double LS_MOVEMENT_THRESHOLD = 4; // Threshold to move at low speed

    private final double FINAL_POSITION_THRESHOLD = 0.2; // threshold for terminating command

    private final double HIGH_SPEED_NORMALIZED = 0.18;
    private final double LOW_SPEED_NORMALIZED = 0.15;
    private final double STUPIDLY_SLOW_SPEED_NORMALIZED = 0.08;

    public static final double SCORE_LOW_ANGLE = 6.39;
    public static final double SCORE_MID_ANGLE = 8.89;
    public static final double SCORE_TOP_ANGLE = 15.60;
    public static final double INTAKE_ANGLE = 0;

    private double speedSetpoint = 0;

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

        if (targetAngle == 0) {
            Robot.coralSub.setWrist(-0.4);
        } else {
            // Get the position difference
            double difference = (targetAngle - (Robot.coralSub.wristEncoder.getPosition().getValueAsDouble() * 100));

            speedSetpoint = 0;
            if (Math.abs(difference) > HS_MOVEMENT_THRESHOLD)
            {
                speedSetpoint = -HIGH_SPEED_NORMALIZED;        
            }
            else if (Math.abs(difference) > LS_MOVEMENT_THRESHOLD)
            {
                speedSetpoint = -LOW_SPEED_NORMALIZED;
            }
            else
            {
                // Move very slowly 
                speedSetpoint = -STUPIDLY_SLOW_SPEED_NORMALIZED;
            }

            // Set the sign based on direction that the wrist needs to move
            if (difference > 0)
            {
                speedSetpoint = Math.abs(speedSetpoint);
            }

            Robot.coralSub.setWrist(speedSetpoint);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.coralSub.setWrist(0);
        if (!Robot.coralSub.wristSwitch.get()) {
            Robot.coralSub.wrist.setPosition(0);
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(targetAngle - (Robot.coralSub.wristEncoder.getPosition().getValueAsDouble() * 100)) <= FINAL_POSITION_THRESHOLD || 
        (targetAngle == 0 && !Robot.coralSub.wristSwitch.get())); 
    }
}
