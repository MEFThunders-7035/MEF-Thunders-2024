package frc.robot;

public final class Constants {
    private Constants() {} // Prevents instantiation, as this is a utility class
    private static final class UtilConstants {
        public static int kRoborioDIOCount = 9;
    }

    public static final class DriveConstants {
        public static final class InterphaseConstants {
            public static final int kLeftEncoderPortA = 9 + UtilConstants.kRoborioDIOCount + 1; // 0 at navX Mxp
            public static final int kLeftEncoderPortB = 1 + UtilConstants.kRoborioDIOCount + 1; // 1 at navX Mxp
            public static final int kRightEncoderPortA = 2 + UtilConstants.kRoborioDIOCount + 1; // 2 at navX Mxp
            public static final int kRightEncoderPortB = 3 + UtilConstants.kRoborioDIOCount + 1; // 3 at navX Mxp
        }
        
        public static final class MotorConstants {
            public static final int kLeftMotor1ID = 0;
            public static final int kLeftMotor2ID = 1;
            public static final int kRightMotor1ID = 2;
            public static final int kRightMotor2ID = 3;
        }
    }
}
