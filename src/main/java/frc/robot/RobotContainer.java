// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.ClawBack;
import frc.robot.commands.ClawForward;
import frc.robot.commands.ClimbBrakeOff;
import frc.robot.commands.ClimbBrakeOn;
import frc.robot.commands.ClimberBack;
import frc.robot.commands.ClimberForward;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.CoralOuttake;
import frc.robot.commands.ElevatorDownManual;
import frc.robot.commands.ElevatorUpManual;
import frc.robot.commands.LowerActuators;
import frc.robot.commands.RaiseActuators;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // joystick.start().whileTrue(new ClimberForward());
        // joystick.back().whileTrue(new ClimberBack()); 
        joystick.leftBumper().whileTrue(new ElevatorUpManual());
        joystick.rightBumper().whileTrue(new ElevatorDownManual());
        joystick.leftTrigger().whileTrue(new CoralIntake());
        joystick.rightTrigger().whileTrue(new CoralOuttake());
        joystick.a().whileTrue(new ClawBack());
        joystick.b().whileTrue(new ClawForward());
        // joystick.x().onTrue(new ClimbBrakeOn());
        // joystick.y().onTrue(new ClimbBrakeOff());
        joystick.x().whileTrue(new LowerActuators());
        joystick.y().whileTrue(new RaiseActuators());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
