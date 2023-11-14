package team3647.frc2024.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.Idle;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import team3647.lib.CommandSwerveDrivetrain;
import team3647.lib.PeriodicSubsystem;

public class SwerveDrive implements PeriodicSubsystem {

    private final CommandSwerveDrivetrain drivetrain;

    private final Pigeon2 gyro;

    private final PIDController kHeadingController = new PIDController(1, 0, 0);

    private final double maxSpeedMpS;
    private final double maxRotRadPerSec;

    private PeriodicIO periodicIO = new PeriodicIO();

    private final Pose2d zeroPose2d = new Pose2d();

    private final double kDt;

    private double pitchZero = 0;

    public static class PeriodicIO {
        // inputs
        public boolean isOpenloop = true;
        public double heading = 0;
        public double roll = 0;
        public double pitch = 0;
        public double rawHeading = 0;
        public Rotation2d gyroRotation = new Rotation2d();

        public SwerveRequest.ApplyChassisSpeeds requestFromChassisSpeeds =
                new SwerveRequest.ApplyChassisSpeeds();
        public SwerveRequest.RobotCentric robotCentricRequest = new SwerveRequest.RobotCentric();
        public SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric();

        public SwerveRequest request = new Idle();

        public double timestamp = 0;
    }

    public SwerveDrive(
            CommandSwerveDrivetrain drivetrain,
            double maxSpeedMpS,
            double maxRotRadPerSec,
            double kDt) {

        this.drivetrain = drivetrain;
        this.gyro = this.drivetrain.getGyro();
        this.maxSpeedMpS = maxSpeedMpS;
        this.maxRotRadPerSec = maxRotRadPerSec;
        this.kDt = kDt;
    }

    public void zeroPitch() {
        this.pitchZero = this.getPitch();
    }

    public double getPitch() {
        return periodicIO.pitch;
    }

    @Override
    public void readPeriodicInputs() {
        periodicIO.roll = this.gyro.getRoll().getValue();
        periodicIO.pitch = this.gyro.getPitch().getValue() - this.pitchZero;
        periodicIO.heading = this.gyro.getYaw().getValue();
        periodicIO.rawHeading = this.gyro.getYaw().getValue();
    }

    @Override
    public void writePeriodicOutputs() {
        this.drivetrain.setControl(periodicIO.request);
    }

    public void drive(
            Translation2d translation,
            double rotation,
            boolean fieldRelative,
            boolean isOpenLoop,
            boolean correct) {
        double actualRotation = Units.degreesToRadians(gyro.getRate());
        boolean corrects = correct;
        double correction =
                ((Math.abs(rotation - actualRotation) < 0.05 && Math.abs(rotation) > 0.05)
                                        || Math.abs(rotation) < 0.05)
                                && Math.abs(getPitch()) < 3
                                && corrects
                        ? kHeadingController.calculate(rotation - actualRotation)
                        : 0;

        SwerveModuleState[] swerveModuleStates = null;
        ChassisSpeeds speeds =
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                translation.getX(),
                                translation.getY(),
                                rotation + correction,
                                Rotation2d.fromDegrees(getRawHeading()))
                        : new ChassisSpeeds(translation.getX(), translation.getY(), rotation);

        Pose2d robot_pose_vel =
                new Pose2d(
                        speeds.vxMetersPerSecond * this.kDt,
                        speeds.vyMetersPerSecond * this.kDt,
                        Rotation2d.fromRadians(speeds.omegaRadiansPerSecond * kDt));
        Twist2d twist_vel = robot_pose_vel.log(zeroPose2d);
        ChassisSpeeds updated_chassis_speeds =
                new ChassisSpeeds(
                        -twist_vel.dx / kDt, -twist_vel.dy / kDt, -twist_vel.dtheta / kDt);

        periodicIO.requestFromChassisSpeeds.withSpeeds(updated_chassis_speeds);

        periodicIO.request = periodicIO.requestFromChassisSpeeds;

        periodicIO.isOpenloop = isOpenLoop;
    }

    public double getHeading() {
        return periodicIO.heading;
    }

    public double getMaxSpeed() {
        return maxSpeedMpS;
    }

    public double getMaxRot() {
        return maxRotRadPerSec;
    }

    public double getRawHeading() {
        return periodicIO.rawHeading;
    }

    @Override
    public void periodic() {
        readPeriodicInputs();
        writePeriodicOutputs();
    }

    public double getMaxSpeedMpS() {
        return this.maxSpeedMpS;
    }

    public double getMaxRotationRadpS() {
        return this.maxRotRadPerSec;
    }

    @Override
    public String getName() {
        return "Swerve Drivetrain";
    }
}
