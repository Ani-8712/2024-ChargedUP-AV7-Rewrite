package team3647.frc2024.constants;

// import com.ctre.phoenix.ErrorCode;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.TalonFXConfigurator;
// import com.ctre.phoenix.sensors.AbsoluteSensorRange;
// import com.ctre.phoenix.sensors.CANCoder;
// import com.ctre.phoenix.sensors.CANCoderConfiguration;
// import com.ctre.phoenix.sensors.Pigeon2;
// import com.ctre.phoenix.sensors.Pigeon2Configuration;
// import com.ctre.phoenix.sensors.SensorInitializationStrategy;
// import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SwerveModuleSteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import team3647.lib.CommandSwerveDrivetrain;

public class SwerveDriveConstants {
    // default falcon rotates counter clockwise (CCW)
    // make sure gyro -CW, +CCW

    public static final SensorDirectionValue canCoderInvert =
            SensorDirectionValue.CounterClockwise_Positive;
    public static final InvertedValue kDriveMotorInverted = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue kTurnMotorInverted = InvertedValue.Clockwise_Positive;

    // physical possible max speed
    public static final double kDrivePossibleMaxSpeedMPS = 5;
    public static final double kRotPossibleMaxSpeedRadPerSec = 10;

    public static final NeutralModeValue kTurnNeutralMode = NeutralModeValue.Coast;
    public static final NeutralModeValue kDriveNeutralMode = NeutralModeValue.Brake;

    public static final PIDController kAutoSteerXYPIDController = new PIDController(0.10, 0, 0);

    public static final Pigeon2 kGyro =
            new Pigeon2(GlobalConstants.SwerveDriveIds.gyroPin, "drive");

    // config swerve module reversed here, module class doens't reverse for you

    // distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(20.75);
    // distance between front and back wheels

    public static final double kWheelBase = Units.inchesToMeters(20.75);
    // translations are locations of each module wheel
    // 0 --> ++ --> front left
    // 1 --> +- --> front right
    // 2 --> -+ --> back left
    // 3 --> -- --> back right
    // c is center of robot,
    // +x towards front of robot, +y towards left of robot
    // +x
    // ^
    // |
    // +y<--c

    public static final double kFrontModulePoseX = kWheelBase / 2.0;
    public static final double kBackModulePoseX = -kFrontModulePoseX;
    public static final double kLeftModulePoseY = kTrackWidth / 2.0;
    public static final double kRightModulePoseY = -kLeftModulePoseY;

    public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                    new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
                    new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
                    new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
                    new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0));

    // config conversion factors here for each module. in meters for postiion and
    // radians for
    // rotation.

    // from motor to output shaft
    public static final double kDriveMotorGearRatio = 1 / 6.75;
    public static final double kTurnMotorGearRatio = 7.0 / 150.0;
    public static final double kWheelDiameterMeters = 0.097; // 97mm

    // // divide for tick to deg
    public static final double kTurnMotorNativeToDeg = kTurnMotorGearRatio * 360.0;

    public static final double kTurnMotorNativeToDPS = kTurnMotorNativeToDeg; // RPS / Native/10ms

    public static final double kWheelRotationToMetersDrive =
            kWheelDiameterMeters * Math.PI * kDriveMotorGearRatio;

    // Multiply by 10 because velocity is in ticks/100ms
    public static final double kFalconVelocityToMpS = kWheelRotationToMetersDrive;

    public static final double kFalconTicksToMeters = kWheelRotationToMetersDrive;

    public static final double kNominalVoltage = 10;
    public static final double kStallCurrent = 35;
    public static final double kMaxCurrent = 60;

    // prac bot
    // public static final double kAbsFrontLeftEncoderOffsetDeg = 302.52;
    // public static final double kAbsFrontRightEncoderOffsetDeg = 244.77;
    // public static final double kAbsBackLeftEncoderOffsetDeg = 121.9;
    // public static final double kAbsBackRightEncoderOffsetDeg = 240.3;

    // comp bot
    // public static final double kAbsFrontLeftEncoderOffsetDeg = 37.01;
    // public static final double kAbsFrontRightEncoderOffsetDeg = 184.48;
    // public static final double kAbsBackLeftEncoderOffsetDeg = 348.13;
    // public static final double kAbsBackRightEncoderOffsetDeg = 246.88;

    // comp bot
    public static final double kAbsFrontLeftEncoderOffsetDeg = 196.7; // 215.77; //35.595;
    public static final double kAbsFrontRightEncoderOffsetDeg = 182.3; // 183.51; //182.724;
    public static final double kAbsBackLeftEncoderOffsetDeg = 348.2; // 347.34; //348.222;
    public static final double kAbsBackRightEncoderOffsetDeg = 248.1; // 67.23; //247.851;

    // max speed limits that we want
    public static final double kTeleopDriveMaxAccelUnitsPerSec = kDrivePossibleMaxSpeedMPS / 2;
    public static final double kTeleopDriveMaxAngularAccelUnitsPerSec =
            kRotPossibleMaxSpeedRadPerSec / 3;

    // master FF for drive for all modules
    public static final double kS = (0.56744 / 12); // 0.56744; // Volts
    public static final double kV = (2.5 / 12.0); // Volts
    public static final double kA = (0.0 / 12); // Volts

    public static final SimpleMotorFeedforward kMasterDriveFeedforward =
            new SimpleMotorFeedforward(kS, kV, kA);

    // master PID constants for turn and drive for all modules
    public static final double kDriveP = 0.00014;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;

    public static final double kTurnP = 0.4;
    public static final double kTurnI = 0.0;
    public static final double kTurnD = 0;

    public static final double kYP = 1;
    public static final double kYI = 0.0;
    public static final double kYD = 0;

    static class CustomSlotGains extends Slot0Configs {
        public CustomSlotGains(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
        }
    }

    public static final SwerveDrivetrainConstants kDrivetrainConstants =
            new SwerveDrivetrainConstants()
                    .withPigeon2Id(GlobalConstants.SwerveDriveIds.gyroPin)
                    .withCANbusName("drive")
                    .withSupportsPro(false);

    public static final SwerveModuleConstantsFactory kConstantsCreator =
            new SwerveModuleConstantsFactory()
                    .withDriveMotorGearRatio(kDriveMotorGearRatio)
                    .withSteerMotorGearRatio(kTurnMotorGearRatio)
                    .withWheelRadius(Units.metersToInches(kWheelDiameterMeters / 2))
                    .withDriveMotorGains(new CustomSlotGains(kDriveP, kDriveI, kDriveD))
                    .withSteerMotorGains(new CustomSlotGains(kTurnP, kTurnI, kTurnD))
                    .withFeedbackSource(SwerveModuleSteerFeedbackType.RemoteCANcoder);

    public static final SwerveModuleConstants kFrontLeftModule =
            kConstantsCreator.createModuleConstants(
                    GlobalConstants.SwerveDriveIds.kFrontLeftTurnId,
                    GlobalConstants.SwerveDriveIds.kFrontLeftDriveId,
                    GlobalConstants.SwerveDriveIds.kFrontLeftAbsEncoderPort,
                    kAbsFrontLeftEncoderOffsetDeg,
                    kFrontModulePoseX,
                    kLeftModulePoseY,
                    false);

    public static final SwerveModuleConstants kFrontRightModule =
            kConstantsCreator.createModuleConstants(
                    GlobalConstants.SwerveDriveIds.kFrontRightTurnId,
                    GlobalConstants.SwerveDriveIds.kFrontRightDriveId,
                    GlobalConstants.SwerveDriveIds.kFrontRightAbsEncoderPort,
                    kAbsFrontRightEncoderOffsetDeg,
                    kFrontModulePoseX,
                    kRightModulePoseY,
                    false);

    public static final SwerveModuleConstants kBackLeftModule =
            kConstantsCreator.createModuleConstants(
                    GlobalConstants.SwerveDriveIds.kBackLeftTurnId,
                    GlobalConstants.SwerveDriveIds.kBackLeftDriveId,
                    GlobalConstants.SwerveDriveIds.kBackLeftAbsEncoderPort,
                    kAbsBackLeftEncoderOffsetDeg,
                    kBackModulePoseX,
                    kLeftModulePoseY,
                    false);

    public static final SwerveModuleConstants kBackRightModule =
            kConstantsCreator.createModuleConstants(
                    GlobalConstants.SwerveDriveIds.kBackRightTurnId,
                    GlobalConstants.SwerveDriveIds.kBackRightDriveId,
                    GlobalConstants.SwerveDriveIds.kBackRightAbsEncoderPort,
                    kAbsBackRightEncoderOffsetDeg,
                    kBackModulePoseX,
                    kRightModulePoseY,
                    false);

    public static final CommandSwerveDrivetrain kSwerveDrivetrain =
            new CommandSwerveDrivetrain(
                    kDrivetrainConstants,
                    kFrontLeftModule,
                    kFrontRightModule,
                    kBackLeftModule,
                    kBackRightModule);
    public static final PIDController kAutoSteerHeadingController = new PIDController(0.12, 0, 0);

    private SwerveDriveConstants() {}
}
