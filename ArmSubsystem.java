package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ArmSubsystem {
	private final DcMotor armMotor;

	public ArmSubsystem(HardwareMap hw) {
		// アームモーターの初期化（DcMotorとして）
		armMotor = hw.get(DcMotor.class, "armmotor");

		// モーターの設定
		if (armMotor != null) {
			// 停止時はブレーキ
			armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			// 初期化時にモーターを停止
			armMotor.setPower(0.0);
		}
	}

	public void moveArm(double power) {
		if (armMotor != null) {
			// デッドゾーン設定（0.1以下は無視）
			if (power < 0.1) {
				armMotor.setPower(0.0);
				System.out.println("Arm Motor (DcMotor): STOPPED");
			} else {
				// L2 trigger値をモーターパワーとして使用
				// パワー値を-1.0から1.0の範囲に制限
				double limitedPower = Math.max(-1.0, Math.min(1.0, power));
				armMotor.setPower(limitedPower);
				System.out.println("Arm Motor (DcMotor) Power: " + limitedPower);
			}
		} else {
			System.out.println("Arm Motor (DcMotor) is NULL!");
		}
	}
}