// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final static class DriveTrainConstants{
        
        
    }
    public final static class EncoderContants{
        public final static int canCoderTicksToRotation = 2048;
    }

    public static class SwerveConstants {
        public static double homeFrontRightAngle = 37.001;
        public static double homeRearRightAngle = 224.38;
        public static double homeRearLeftAngle = 353.93;
        public static double homeFrontLeftAngle = 289.24;

        /*
        right front encoder: 37.001953125
        right rear encoder: 224.384765625
        left rear encoder: 353.935546875
        left front encoder: 289.248046875
        */
    }

}
