// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSub extends SubsystemBase {

  SparkMax climber;
  public Servo leftActuator;
  public Servo rightActuator;
  Servo climbBrake;

  /** Creates a new ClimberSub. */
  public ClimberSub() {
    climber = new SparkMax(14, MotorType.kBrushless);
    leftActuator = new Servo(3);
    leftActuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    rightActuator = new Servo(3);
    rightActuator.setBoundsMicroseconds(2000, 1800, 1500, 1200, 1000);
    // leftActuator = new PWM(3);
    // leftActuator.setZeroLatch();
    // rightActuator = new PWM(1);
    // rightActuator.setZeroLatch();
    climbBrake = new Servo(2);
  }

  public void setPower(double power) {
      climber.set(power);
  }

  public void moveActuators(double position) {
    double pulseWidth = map(position, -1.0, 1.0, 1.0, 2.0); // Map joystick to pulse width (ms)
    pulseWidth = Math.min(Math.max(pulseWidth, 1.0), 2.0);

    leftActuator.setSpeed((pulseWidth - 1));
    rightActuator.setSpeed((pulseWidth - 1));
  }

  public void extendActuators()
  {
    leftActuator.setSpeed(1);
    rightActuator.setSpeed(1);
  }

  public void retractActuators()
  {
    leftActuator.setSpeed(-1);
    rightActuator.setSpeed(-1);
  }

  private double map(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
    return toLow + (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow);
}

  public void moveBrake(double position) {
    climbBrake.setPosition(position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
