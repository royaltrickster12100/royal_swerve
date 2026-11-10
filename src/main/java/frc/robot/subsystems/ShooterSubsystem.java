package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    private final TalonFX masterMotor;
    private final TalonFX followerMotor;

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
    private final VoltageOut stopRequest = new VoltageOut(0);

    private double targetRPS = 0.0;

    public ShooterSubsystem() {
        masterMotor = new TalonFX(ShooterConstants.kMasterCanId);
        followerMotor = new TalonFX(ShooterConstants.kFollowerCanId);

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;

        config.Slot0.kP = ShooterConstants.kP;
        config.Slot0.kI = ShooterConstants.kI;
        config.Slot0.kD = ShooterConstants.kD;
        config.Slot0.kV = ShooterConstants.kV;

        masterMotor.getConfigurator().apply(config);
        followerMotor.getConfigurator().apply(config);

        followerMotor.setControl(new Follower(ShooterConstants.kMasterCanId, MotorAlignmentValue.Opposed));
    }

    @Override
    public void periodic() {
        if (targetRPS <= 0) {
            masterMotor.setControl(stopRequest);
        } else {
            masterMotor.setControl(velocityRequest.withVelocity(targetRPS));
        }
    }

    public boolean isAtSpeed() {
        if (RobotBase.isSimulation()) {
            return targetRPS > 0;
        }
        double currentRPS = masterMotor.getVelocity().getValueAsDouble();
        return Math.abs(currentRPS - targetRPS) <= ShooterConstants.kRPSTolerance;
    }

    public Command spinUpCommand(double rps) {
        return this.runOnce(() -> this.targetRPS = rps);
    }

    public Command stopCommand() {
        return this.runOnce(() -> this.targetRPS = 0.0);
    }
}