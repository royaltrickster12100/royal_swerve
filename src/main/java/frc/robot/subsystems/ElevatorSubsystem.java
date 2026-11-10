package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX elevatorMotor;
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    private double targetRotations = 0.0;

    public ElevatorSubsystem() {
        elevatorMotor = new TalonFX(ElevatorConstants.kElevatorMotorCanId);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = ElevatorConstants.kElevatorGearRatio;

        config.Slot0.kP = ElevatorConstants.kP;
        config.Slot0.kI = ElevatorConstants.kI;
        config.Slot0.kD = ElevatorConstants.kD;
        config.Slot0.kG = ElevatorConstants.kG;
        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;

        elevatorMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        elevatorMotor.setControl(positionRequest.withPosition(targetRotations));
    }

    public Command setHeightCommand(double heightMeters) {
        return this.runOnce(() -> {
            double perimeter = 2 * Math.PI * ElevatorConstants.kDrumRadiusMeters;
            double rotations = heightMeters / perimeter;
            this.targetRotations = MathUtil.clamp(rotations, ElevatorConstants.kMinHeightRotations, ElevatorConstants.kMaxHeightRotations);
        });
    }
}