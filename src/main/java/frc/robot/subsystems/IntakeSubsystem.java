package frc.robot.subsystems;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private final TalonFX intakeMotor;
    private final DigitalInput beamBreak;
    private final VoltageOut voltageRequest = new VoltageOut(0);

    public IntakeSubsystem() {
        intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorCanId);
        beamBreak = new DigitalInput(IntakeConstants.kBeamBreakDioChannel);

        intakeMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public boolean hasGamePiece() {
        return !beamBreak.get();
    }

    public Command runIntakeCommand() {
        return this.run(() -> intakeMotor.setControl(voltageRequest.withOutput(IntakeConstants.kIntakeVoltage)))
                   .until(this::hasGamePiece)
                   .finallyDo(() -> intakeMotor.setControl(voltageRequest.withOutput(0)));
    }

    public Command runEjectCommand() {
        return this.startEnd(
            () -> intakeMotor.setControl(voltageRequest.withOutput(IntakeConstants.kEjectVoltage)),
            () -> intakeMotor.setControl(voltageRequest.withOutput(0))
        );
    }
}