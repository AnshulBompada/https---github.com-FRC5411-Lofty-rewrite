// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Auton extends SubsystemBase {
  private Swerve m_drive;
  private Arm m_arm;


  public Auton(Swerve drive, Arm arm) {
    m_drive = drive;
    m_arm = arm;

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
