package team3647.frc2024.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import team3647.frc2024.subsystems.SwerveDrive;

public class DrivetrainCommands {
    private final SwerveDrive swerve;
    private final double maxSpeed;
    private final double maxRot;

    public DrivetrainCommands(SwerveDrive swerve) {
        this.swerve = swerve;
        this.maxSpeed = swerve.getMaxSpeed();
        this.maxRot = swerve.getMaxRot();
    }

    public Command driveVisionTeleop(
            DoubleSupplier xSpeedFunction,
            DoubleSupplier ySpeedFunction,
            DoubleSupplier turnSpeedFunction,
            Supplier<Twist2d> autoSteerVelocitiesSupplier,
            BooleanSupplier shouldLockY,
            BooleanSupplier shouldLockRot,
            BooleanSupplier shouldCorrect,
            BooleanSupplier isInSlowMode) {
        return Commands.run(
                () -> {
                    double triggerSlow = isInSlowMode.getAsBoolean() ? 0.2 : 1;
                    boolean isLockY = shouldLockY.getAsBoolean();
                    boolean isLockRotation = shouldLockRot.getAsBoolean();
                    boolean isCorrected = shouldCorrect.getAsBoolean();
                    double motionXComponent = ySpeedFunction.getAsDouble() * maxSpeed * triggerSlow;

                    double motionYComponent = xSpeedFunction.getAsDouble() * maxSpeed * triggerSlow;

                    double motionTurnComponent =
                            turnSpeedFunction.getAsDouble() * maxRot * triggerSlow;

                    motionTurnComponent =
                            isLockRotation
                                    ? autoSteerVelocitiesSupplier.get().dtheta
                                            + 0.1 * motionTurnComponent
                                    : motionTurnComponent;

                    motionYComponent =
                            isLockY
                                    ? autoSteerVelocitiesSupplier.get().dtheta
                                            + 0.1 * motionYComponent
                                    : motionYComponent;

                    Translation2d translation =
                            new Translation2d(motionXComponent, motionYComponent);

                    swerve.drive(translation, motionTurnComponent, true, false, isCorrected);
                },
                swerve);
    }
}
