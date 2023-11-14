package team3647.frc2024.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import team3647.frc2024.commands.DrivetrainCommands;
import team3647.frc2024.constants.GlobalConstants;
import team3647.frc2024.constants.LimelightConstant;
import team3647.frc2024.constants.SwerveDriveConstants;
import team3647.frc2024.subsystems.SwerveDrive;
import team3647.frc2024.util.AutoDrive;
import team3647.lib.GroupPrinter;
import team3647.lib.inputs.Joysticks;
import team3647.lib.vision.LimelightHelpers;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        pdh.clearStickyFaults();
        scheduler.registerSubsystem(swerve);

        configureDefaultCommands();
        configureButtonBindings();
        configureSmartDashboardLogging();
    }

    private void configureButtonBindings() {}

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(
                swerveDriveCommands.driveVisionTeleop(
                        mainController::getLeftStickX,
                        mainController::getLeftStickY,
                        mainController::getRightStickX,
                        autoSteer::getVelocities,
                        autoSteer::getLockY,
                        autoSteer::getLockRot,
                        autoSteer::isAllDisabled,
                        mainController.leftTrigger));
    }

    public void teleopInit() {
        // if (superstructure.getWantedIntakePiece() == GamePiece.Cone) {
        // LEDS.setPiece(true);
        // } else if (superstructure.getWantedIntakePiece() == GamePiece.Cube) {
        // LEDS.setPiece(false);
        // }

    }

    void configTestCommands() {

        // Commands.run(() -> {}, grabber).schedule();
    }

    public double getPivotFFVoltage() {
        return 0.0;
    }

    public void configureSmartDashboardLogging() {

        // printer.addBoolean("intkae", () -> goodForLockIntake.getAsBoolean());
        // printer.addBoolean("score", () -> goodForLockScore.getAsBoolean());
        // printer.addBoolean("cube", () ->
        // limelightTriggers.wantedCube.getAsBoolean());
        // printer.addDouble(
        // "tx", () -> -LimelightHelpers.getTX(LimelightConstant.kLimelightCenterHost));
        // printer.addBoolean(
        // "aligned",
        // () ->
        //
        // (Math.abs(-LimelightHelpers.getTX(LimelightConstant.kLimelightCenterHost))
        // < 1)
        // && (Math.abs(
        // -LimelightHelpers.getTX(
        // LimelightConstant.kLimelightCenterHost))
        // > 0)
        // && (Math.abs((swerve.getHeading() % 360) - 180) < 30
        // || Math.abs((swerve.getHeading() % 360) + 180) < 30));
        // printer.addString("stored", LEDS::getStoredPiece);
    }

    // counted relative to what driver sees
    public Command getAutonomousCommand() {
        return new InstantCommand(() -> {});
    }

    private final Joysticks mainController = new Joysticks(0);
    private final Joysticks coController = new Joysticks(1);

    private final SwerveDrive swerve =
            new SwerveDrive(
                    SwerveDriveConstants.kSwerveDrivetrain,
                    SwerveDriveConstants.kDrivePossibleMaxSpeedMPS,
                    SwerveDriveConstants.kRotPossibleMaxSpeedRadPerSec,
                    GlobalConstants.kDt);
    private final DrivetrainCommands swerveDriveCommands = new DrivetrainCommands(swerve);

    private final Compressor compressor = new Compressor(GlobalConstants.kPCMType);

    private final AutoDrive autoSteer =
            new AutoDrive(
                    SwerveDriveConstants.kAutoSteerXYPIDController,
                    swerve,
                    () -> LimelightHelpers.getTX(LimelightConstant.kLimelightCenterHost));

    // private final AutoSteer autoSteer =
    // new AutoSteer(
    // swerve::getOdoPose,
    // () -> -LimelightHelpers.getTX(LimelightConstant.kLimelightCenterHost),
    // SwerveDriveConstants.kAutoSteerXYPIDController,
    // SwerveDriveConstants.kAutoSteerXYPIDController,
    // SwerveDriveConstants.kAutoSteerHeadingController);

    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

    private final CommandScheduler scheduler = CommandScheduler.getInstance();
    final GroupPrinter printer = GroupPrinter.getInstance();

    // .and(() -> superstructure.getGamePiece() == GamePiece.Cone)

    private final Pose2d kEmptyPose = new Pose2d();
}
