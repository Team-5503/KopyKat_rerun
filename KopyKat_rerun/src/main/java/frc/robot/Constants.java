// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriverDeadbandX = .07;
    public static final double kDriverDeadbandY = .07;
  }

  public static class DriveBaseConstants {
    public static final int kFrontLeftID = 0;
    public static final int kBackLeftID = 0;
    public static final int kFrontRightID = 0;
    public static final int kBackRightID = 0;

    public static final boolean kInvertedL = true;
    public static final boolean kInvertedR = false;

    public static final int kStallLimit = 70;
    public static final int kFreeLimit = 45;

    public static final IdleMode kIdleMode = IdleMode.kCoast;
  }
}
