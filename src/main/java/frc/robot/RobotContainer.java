// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.RobotModeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* Requisição de controle principal do Swerve */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Controles: Piloto (Porta 0) e Operador (Porta 1)
    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private IntakeSubsystem intakeSubsystem;
    private ArmSubsystem armSubsystem;
    private ElevatorSubsystem elevatorSubsystem;
    private ShooterSubsystem shooterSubsystem;

    public RobotContainer() {
        if (RobotModeConstants.IS_ATTACHMENTS_MOUNTED || RobotBase.isSimulation()) {
            intakeSubsystem = new IntakeSubsystem();
            armSubsystem = new ArmSubsystem();
            elevatorSubsystem = new ElevatorSubsystem();
            shooterSubsystem = new ShooterSubsystem();
        }

        configureBindings();
    }

    private void configureBindings() {
        // Comando de movimentação padrão (Piloto)
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                double x = MathUtil.applyDeadband(-joystick.getLeftY(), 0.1) * MaxSpeed;
                double y = MathUtil.applyDeadband(-joystick.getLeftX(), 0.1) * MaxSpeed;
                double rot = MathUtil.applyDeadband(-joystick.getRightX(), 0.1) * MaxAngularRate;

                return drive.withVelocityX(x).withVelocityY(y).withRotationalRate(rot);
            })
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() -> {
            double x = MathUtil.applyDeadband(-joystick.getLeftY(), 0.1);
            double y = MathUtil.applyDeadband(-joystick.getLeftX(), 0.1);
            return point.withModuleDirection(new Rotation2d(x, y));
        }));

        // ALINHAMENTO AUTOMÁTICO DE MIRA (Limelight 4 - Gatilho Direito do Piloto)
        joystick.rightBumper().whileTrue(
            drivetrain.applyRequest(() -> {
                boolean hasTarget = LimelightHelpers.getTV(VisionConstants.kLimelightName);
                double tx = LimelightHelpers.getTX(VisionConstants.kLimelightName);

                // Publica dados de diagnóstico no SmartDashboard
                SmartDashboard.putBoolean("Limelight/HasTarget", hasTarget);
                SmartDashboard.putNumber("Limelight/TX", tx);

                // Mantém a pilotagem de translação na alavanca esquerda
                double x = MathUtil.applyDeadband(-joystick.getLeftY(), 0.1) * MaxSpeed;
                double y = MathUtil.applyDeadband(-joystick.getLeftX(), 0.1) * MaxSpeed;

                double rot;
                if (hasTarget) {
                    // Ganho proporcional (kP = 0.06). Multiplica o erro angular 'tx' para gerar rotação em rad/s
                    rot = tx * 0.06;
                    // Limita a velocidade máxima de giro automático para segurança
                    rot = MathUtil.clamp(rot, -MaxAngularRate, MaxAngularRate);
                } else {
                    // Se não houver alvo visível, devolve a rotação para a alavanca direita
                    rot = MathUtil.applyDeadband(-joystick.getRightX(), 0.1) * MaxAngularRate;
                }

                return drive.withVelocityX(x).withVelocityY(y).withRotationalRate(rot);
            })
        );

        // Rotinas de SysId
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset da orientação do campo (Left Bumper)
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        // ------------------------------------------------------------------------
        // CONTROLES DO OPERADOR (Anexos)
        // ------------------------------------------------------------------------
        if (!RobotModeConstants.IS_ATTACHMENTS_MOUNTED && !RobotBase.isSimulation()) {
            return;
        }

        operatorController.leftTrigger().whileTrue(intakeSubsystem.runIntakeCommand());
        operatorController.leftBumper().whileTrue(intakeSubsystem.runEjectCommand());

        operatorController.rightBumper()
            .onTrue(shooterSubsystem.spinUpCommand(ShooterConstants.kTargetRPS))
            .onFalse(shooterSubsystem.stopCommand());

        new Trigger(shooterSubsystem::isAtSpeed)
            .and(operatorController.rightBumper())
            .whileTrue(intakeSubsystem.runIntakeCommand());

        operatorController.a()
            .onTrue(armSubsystem.setAngleCommand(Math.toRadians(15.0)))
            .onTrue(elevatorSubsystem.setHeightCommand(0.1));

        operatorController.y()
            .onTrue(armSubsystem.setAngleCommand(Math.toRadians(90.0)))
            .onTrue(elevatorSubsystem.setHeightCommand(1.2));
    }

    public Command getAutonomousCommand() {
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            drivetrain.applyRequest(() -> idle)
        );
    }
}