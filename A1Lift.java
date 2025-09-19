package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.AnalogInput;

public class A1Lift {
	
	// リフト用モーター lift motor_.
	private DcMotor liftMotor;
	
	// ポテンショメーター制御用変数
	private static final double LIFT_LIMIT_NORM = 0.600; // M1時の上限（正規化）
	private static final double LIFT_HOLD_POWER = -0.08; // 上限到達後の保持用微小パワー（下がらないように負方向）
	private static final double LIFT_LIMIT_HYST = 0.01; // ヒステリシスでチャタリング緩和
	
	// アナログ入力（ポテンショメータ）
	private AnalogInput poten;
	
	public A1Lift(HardwareMap hardwareMap) {
		// リフトモーター初期化
		liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");
		
		// アナログ入力（poten）初期化（Analogue Input Port 0）
		try {
			poten = hardwareMap.get(AnalogInput.class, "poten");
		} catch (Exception e) {
			poten = null;
		}
		
		// モーター設定
		if (liftMotor != null) {
			liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			liftMotor.setDirection(DcMotor.Direction.REVERSE);
		}
	}
	
	// リフト制御メソッド（左スティックY軸）
	public void moveLift(double leftStickY) {
		// デッドゾーン設定
		if (Math.abs(leftStickY) < 0.1) leftStickY = 0.0;
		
		// リフトモーター制御
		if (liftMotor != null) {
			liftMotor.setPower(leftStickY);
		}
	}
	
	// M1コンボ用リフト制御（三角ボタン押下時）
	public void moveLiftM1Combo() {
		if (liftMotor != null) {
			double liftPower = -0.8; // 上方向固定値（反転）
			
			// ポテンショ正規化値（0.0〜1.0）
			double potenNorm = -1.0;
			if (poten != null && poten.getMaxVoltage() > 0) {
				potenNorm = poten.getVoltage() / poten.getMaxVoltage();
			}
			
			if (potenNorm >= 0.0) {
				// 目標付近〜超過では保持パワー。ヒステリシスでチャタリング抑制
				if (potenNorm >= LIFT_LIMIT_NORM) {
					liftPower = LIFT_HOLD_POWER;
				} else if (potenNorm >= LIFT_LIMIT_NORM - LIFT_LIMIT_HYST) {
					// しきい値手前の薄い帯域では少し弱めてもよいが、まずは固定のまま
					liftPower = -0.8;
				}
			}
			
			liftMotor.setPower(liftPower);
		}
	}
	
	// リフトモーターの状態を取得
	public double getLiftMotorPower() {
		return liftMotor != null ? liftMotor.getPower() : 0.0;
	}
	
	// リフトモーターの接続状態を取得
	public boolean isLiftMotorConnected() {
		return liftMotor != null;
	}
	
	// ポテンショメーターの正規化値を取得
	public double getPotentiometerNormalized() {
		if (poten != null && poten.getMaxVoltage() > 0) {
			return poten.getVoltage() / poten.getMaxVoltage();
		}
		return -1.0;
	}
	
	// ポテンショメーターの接続状態を取得
	public boolean isPotentiometerConnected() {
		return poten != null;
	}
	
	// ポテンショメーターの電圧を取得
	public double getPotentiometerVoltage() {
		return poten != null ? poten.getVoltage() : 0.0;
	}
	
	// 制限値定数を取得
	public double getLiftLimitNorm() { return LIFT_LIMIT_NORM; }
	public double getLiftHoldPower() { return LIFT_HOLD_POWER; }
	public double getLiftLimitHyst() { return LIFT_LIMIT_HYST; }
} 
