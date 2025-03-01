// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralSub extends SubsystemBase {
  /** Creates a new CoralSub. */
  TalonFX leftElevator = new TalonFX(9);
  TalonFX rightElevator = new TalonFX(8);
  TalonFX intake = new TalonFX(10);
  TalonFX wrist = new TalonFX(12);
  public Encoder wristEncoder = new Encoder(0, 1, true, EncodingType.k2X);
  public Encoder elevatorEncoder = new Encoder(7, 8, true, EncodingType.k2X);
  public DigitalInput beam = new DigitalInput(6);


  public CoralSub() {
    wrist.setNeutralMode(NeutralModeValue.Brake);
    leftElevator.setNeutralMode(NeutralModeValue.Brake);
    rightElevator.setNeutralMode(NeutralModeValue.Brake); 
  }

  public void setElevator(double power) {
    leftElevator.set(power);
    rightElevator.set(power);
  }

  public void setIntake(double power) {
    intake.set(power);
  }

  public void setWrist(double power) {
    wrist.set(power);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
