package team3647.frc2024.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import team3647.lib.CommandSwerveDrivetrain;
import team3647.lib.PeriodicSubsystem;
import team3647.lib.SwerveModule;

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
        public SwerveRequest.RobotCentric robotCentricRequest = 
            new SwerveRequest.RobotCentric();
        public SwerveRequest.FieldCentric fieldCentricRequest = 
            new SwerveRequest.FieldCentric();
        public String requestType = "chassisSpeeds";
        
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

    public double getPitch(){
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
        switch (periodicIO.requestType) {
            case "chassisSpeeds":
                drivetrain.setControl(periodicIO.requestFromChassisSpeeds);
                break;
        
            default:
                drivetrain.setControl(periodicIO.requestFromChassisSpeeds);
                SmartDashboard.putBoolean("Invalid requestType", false);
                break;
        }
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
