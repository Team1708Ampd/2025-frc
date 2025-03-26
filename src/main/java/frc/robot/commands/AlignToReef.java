package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToReef extends Command{
    private PIDController xController, yController, rotController;
    private boolean isRightScore;
    private Timer dontSeeTagTimer, stopTimer;
    private CommandSwerveDrivetrain drivebase;
    private double tagID = -1;

    public AlignToReef(boolean isRightScore, CommandSwerveDrivetrain drivebase) {

        // !!!!!!!!!!!!!!!!!11THESE PID CONSTANTS NEED TO BE TUNED!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        xController = new PIDController(2, 0.0, 0);  // Vertical movement
        yController = new PIDController(2, 0.0, 0);  // Horitontal movement
        rotController = new PIDController(2, 0, 0);  // Rotation
        this.isRightScore = isRightScore;
        this.drivebase = drivebase;
        addRequirements(drivebase);
    }

    @Override
    public void initialize() {
        this.stopTimer = new Timer();
        this.stopTimer.start();
        this.dontSeeTagTimer = new Timer();
        this.dontSeeTagTimer.start();

        // THESE CONSTANTS SHOULD BE SELECTED BY PUTTING THE ROBOT AT EXACTLY THE POSITION WE WANT IT TO BE AT 
        // IN FRONT OF THE REEF WHEN THE COMMAND ENDS 
        rotController.setSetpoint(Constants.ROT_SETPOINT_REEF_ALIGNMENT);
        rotController.setTolerance(Constants.ROT_TOLERANCE_REEF_ALIGNMENT);

        xController.setSetpoint(Constants.X_SETPOINT_REEF_ALIGNMENT);
        xController.setTolerance(Constants.X_TOLERANCE_REEF_ALIGNMENT);

        yController.setSetpoint(isRightScore ? Constants.Y_SETPOINT_REEF_ALIGNMENT : -Constants.Y_SETPOINT_REEF_ALIGNMENT);
        yController.setTolerance(Constants.Y_TOLERANCE_REEF_ALIGNMENT);

        tagID = LimelightHelpers.getFiducialID("");
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
        this.dontSeeTagTimer.reset();

        double[] postions = LimelightHelpers.getBotPose_TargetSpace("");
        SmartDashboard.putNumber("x", postions[2]);

        double xSpeed = xController.calculate(postions[2]);
        SmartDashboard.putNumber("xspee", xSpeed);
        double ySpeed = -yController.calculate(postions[0]);
        double rotValue = -rotController.calculate(postions[4]);
       

        drivebase.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(xSpeed, ySpeed, rotValue)));

        if (!rotController.atSetpoint() ||
            !yController.atSetpoint() ||
            !xController.atSetpoint()) {
            stopTimer.reset();
        }
        } else {
            drivebase.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0, 0, 0)));
        }

        SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.setControl(new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(0, 0, 0)));
    }

    @Override
    public boolean isFinished() {
        // TUNE THESE AS NEEDED
        // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
        return this.dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME) ||
            stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
    }
}
