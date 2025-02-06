// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorPivot;

public class SetPivotAngle extends Command {
    private final EndEffectorPivot m_shooterPivot;
    private final DoubleSupplier m_angleProvider;

    public SetPivotAngle(EndEffectorPivot shooterPivot, DoubleSupplier angleRadians) {
        m_shooterPivot = shooterPivot;
        m_angleProvider = angleRadians;

        addRequirements(m_shooterPivot);
    }

    public SetPivotAngle(EndEffectorPivot shooterPivot, double angleRadians) {
        this(shooterPivot, () -> angleRadians);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_shooterPivot.setAngle(Rotation2d.fromDegrees(m_angleProvider.getAsDouble()));
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return m_shooterPivot.angleWithinTolerance();
    }
}
