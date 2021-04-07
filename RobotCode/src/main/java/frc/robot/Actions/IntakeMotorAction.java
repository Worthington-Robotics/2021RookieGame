package frc.robot.Actions;

import frc.lib.statemachine.Action;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;

public class IntakeMotorAction extends Action {

    @Override
    public void onStart() {
        Shooter.getInstance().setIntakePower(Constants.ID_SUPER_INTAKE);
    }

    @Override
    public void onLoop() {
        Shooter.getInstance().setShooterPower(Constants.ID_SUPER_INTAKE);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void onStop() {
        Shooter.getInstance().setIntakePower(0);
    }
}
