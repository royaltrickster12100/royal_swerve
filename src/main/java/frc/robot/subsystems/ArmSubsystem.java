package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX armMotor;
    private final PositionVoltage positionRequest = new PositionVoltage(0);

    private double targetRotations = 0.0;

    public ArmSubsystem() {
        armMotor = new TalonFX(ArmConstants.kArmMotorCanId);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Feedback.SensorToMechanismRatio = ArmConstants.kArmGearRatio;

        config.Slot0.kP = ArmConstants.kP;
        config.Slot0.kI = ArmConstants.kI;
        config.Slot0.kD = ArmConstants.kD;
        config.Slot0.kG = ArmConstants.kG;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        armMotor.getConfigurator().apply(config);
    }

    @Override
    public void periodic() {
        armMotor.setControl(positionRequest.withPosition(targetRotations));
    }

    public Command setAngleCommand(double targetRadians) {
        return this.runOnce(() -> {
            double rotations = targetRadians / (2 * Math.PI);
            this.targetRotations = MathUtil.clamp(rotations, ArmConstants.kMinPositionRotations, ArmConstants.kMaxPositionRotations);
        });
    }
}