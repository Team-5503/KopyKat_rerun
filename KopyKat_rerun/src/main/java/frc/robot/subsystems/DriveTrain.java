// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.DriveBaseConstants;

public class DriveTrain extends SubsystemBase {
  private SparkMax frontL, frontR, backL, backR;
  private RelativeEncoder encoderL, encoderR;
  private SparkMaxConfig frontLc, frontRc, backLc, backRc;
  private DifferentialDrive drive;
  /** Creates a new DriveTrain. */
  public DriveTrain() {
    frontL = new SparkMax(0, MotorType.kBrushless);
    backL = new SparkMax(0, MotorType.kBrushless);

    encoderL = frontL.getEncoder();

    frontR = new SparkMax(0, MotorType.kBrushless);
    backR = new SparkMax(0, MotorType.kBrushless);

    encoderR = frontR.getEncoder();

    drive = new DifferentialDrive(frontL, frontR);

    configure();
  }

  private void configure(){
    frontLc = new SparkMaxConfig();
    frontLc
      .inverted(DriveBaseConstants.kInvertedL)
      .smartCurrentLimit(DriveBaseConstants.kStallLimit, DriveBaseConstants.kFreeLimit)
      .idleMode(DriveBaseConstants.kIdleMode);
    backLc = new SparkMaxConfig();
    backLc
      .apply(frontLc)
      .follow(frontL);
    
    frontL.configure(frontLc, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    backL.configure(backLc, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    
    frontRc = new SparkMaxConfig();
    frontRc
      .inverted(DriveBaseConstants.kInvertedR)
      .smartCurrentLimit(DriveBaseConstants.kStallLimit, DriveBaseConstants.kFreeLimit)
      .idleMode(DriveBaseConstants.kIdleMode);
    backRc = new SparkMaxConfig();
    backRc
      .apply(frontRc)
      .follow(frontR);
    
    frontR.configure(frontRc, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    backR.configure(backRc, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void drive(double forw, double rot){
    drive.arcadeDrive(forw, rot);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
