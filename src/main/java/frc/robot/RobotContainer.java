
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.Map;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.AutoIntakeFeeder;
import frc.robot.commands.ClawBack;
import frc.robot.commands.ClawForward;
import frc.robot.commands.ClimbBrakeOff;
import frc.robot.commands.ClimbBrakeOn;
import frc.robot.commands.ClimberBack;
import frc.robot.commands.ClimberForward;
import frc.robot.commands.CoralIntake;
import frc.robot.commands.CoralIntakeTwoState;
import frc.robot.commands.CoralOuttake;
import frc.robot.commands.ElevatorDownManual;
import frc.robot.commands.ElevatorUpManual;
import frc.robot.commands.GoToIntakePosition;
import frc.robot.commands.LowerActuators;
import frc.robot.commands.RaiseActuators;
import frc.robot.commands.ScoreBottomCoral;
import frc.robot.commands.ScoreMiddleCoral;
import frc.robot.commands.ScoreTopCoral;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // Slew rate limiters for drive control inputs
    // Limiter in the X DIRECTION (NOT X JOYSTICK)
    SlewRateLimiter limiterX = new SlewRateLimiter(0.5, 1.5, 0);
    // Limiter in the Y DIRECTION (NOT Y JOYSTICK)
    SlewRateLimiter limiterY = new SlewRateLimiter(0.5, 1.5, 0);

    // AUTO RELATED VARIABLES AND DEFS
    private SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureBindings();

        // initAuto();

        // Register the commands for the autos
        registerNamedCommands();

        // Schedule the selected auto
        drivetrain.initPathPlannerConfig();
        autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
        CommandScheduler.getInstance().schedule(getAutonomousCommand());
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }
 
    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX((-joystick.getLeftY() * MaxSpeed) / 2) // Drive forward with negative Y (forward)
                    .withVelocityY((-joystick.getLeftX() * MaxSpeed) / 2) // Drive left with negative X (left)
                    .withRotationalRate((-joystick.getRightX() * MaxAngularRate) /2) // Drive counterclockwise with negative X (left)
            )
        );

        // drivetrain.setDefaultCommand(
        //     // Drivetrain will execute this command periodically
        //     drivetrain.applyRequest(() ->
        //         drive.withVelocityX((limiterX.calculate(-joystick.getLeftY()) * MaxSpeed)) // Drive forward with negative Y (forward)
        //              .withVelocityY((limiterY.calculate(-joystick.getLeftX()) * MaxSpeed)) // Drive left with negative X (left)
        //              .withRotationalRate((-joystick.getRightX() * MaxAngularRate) /2) // Drive counterclockwise with negative X (left)
        //     )
        // );

        joystick.povDown().onTrue(new GoToIntakePosition());
        joystick.povLeft().onTrue(new ScoreBottomCoral());
        joystick.povRight().onTrue(new ScoreMiddleCoral());
        joystick.povUp().onTrue(new ScoreTopCoral());

        joystick.a().onTrue(new CoralIntakeTwoState());
        joystick.b().whileTrue(new CoralOuttake());
        joystick.x().whileTrue(new LowerActuators());
        joystick.y().whileTrue(new RaiseActuators());


        joystick.leftBumper().whileTrue(new ElevatorUpManual());
        joystick.rightBumper().whileTrue(new ElevatorDownManual());

        joystick.leftTrigger().whileTrue(new ClawForward());
        joystick.rightTrigger().whileTrue(new ClawBack());

        joystick.start().whileTrue(new ClimberForward());
        joystick.back().whileTrue(new ClimberBack());

        
    } 

    public void initAuto()
    {
        // Initialize PathPlanner config 
        drivetrain.initPathPlannerConfig();

        // Build the auto chooser so that auto routine can be selected 
        // from the dashboard 
        autoChooser = AutoBuilder.buildAutoChooser();
        // Register named commands to be referenced in auto 
        registerNamedCommands();

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public Command getAutonomousCommand()
    {
        return autoChooser.getSelected();
    }

    private void registerNamedCommands()
    {
        // Gotta Register Em All 
        NamedCommands.registerCommand("ScoreTopCoral",    new ScoreTopCoral());
        NamedCommands.registerCommand("ScoreMiddleCoral", new ScoreMiddleCoral());
        NamedCommands.registerCommand("ScoreBottomCoral", new ScoreBottomCoral());
        NamedCommands.registerCommand("AutoIntake",       new AutoIntake());
        NamedCommands.registerCommand("AutoIntakeFeeder", new AutoIntakeFeeder());
        NamedCommands.registerCommand("CoralOuttake",     new CoralOuttake());
        NamedCommands.registerCommand("GoToIntakePosition",new GoToIntakePosition());
        NamedCommands.registerCommand("LowerActuators",   new LowerActuators());
        NamedCommands.registerCommand("RaiseActuators",   new RaiseActuators());
        NamedCommands.registerCommand("ClawForward",      new ClawForward());
        NamedCommands.registerCommand("ClawBack",         new ClawBack());
        NamedCommands.registerCommand("CimberForward",    new ClimberForward());
        NamedCommands.registerCommand("ClimberBack",      new ClimberBack());      

    }
}
