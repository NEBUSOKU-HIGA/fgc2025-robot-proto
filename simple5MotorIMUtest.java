package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

@TeleOp(name="Simple5MotorIMUtest", group="Test")
public class simple5MotorIMUtest extends LinearOpMode {

	// Control Hub のモーター（HD Hex Motor）
	private DcMotorEx leftMotor;
	private DcMotorEx rightMotor;
	private DcMotorEx topMotor;

	// Expansion Hub のモーター
	private DcMotorEx armMotor;  // HD Hex Motor
	private DcMotor liftMotor;   // Core Hex Motor

	// トップモーター用の変数
	private double currentTopPower = 0.0;
	private double targetTopPower = 0.0;
	private static final double POWER_INCREMENT = 0.05; // 5%ずつ増加
	private static final double POWER_DECREMENT = 0.1; // 10%ずつ減少
	private static final double LIFT_LIMIT_NORM = 0.60; // M1時の上限（正規化）
	private static final double LIFT_HOLD_POWER = -0.08; // 上限到達後の保持用微小パワー（下がらないように負方向）
	private static final double LIFT_LIMIT_HYST = 0.01; // ヒステリシスでチャタリング緩和
	// 右スティック左右（旋回）感度調整用パラメータ（現場で調整してください）
	private static final double TURN_DEADZONE = 0.15; // これ未満は旋回0にする（0.10〜0.20の範囲で調整）
	private static final double TURN_EXPONENT = 2.0;  // 小入力を弱めるカーブ（1.0=直線, 2.0〜3.0で穏やかに）
	private static final double TURN_GAIN = 0.7;	  // 全体の強さ倍率（0.5〜1.0で調整）
	private static final double ROTATE_BUMPER_POWER = 0.6; // L1/R1での定速旋回パワー
	private static final long AUTO_TURN_TIMEOUT_MS = 2000; // options自動整向の最大制御時間
	private static final long AUTO_TURN_TIMEOUT_MS_MIN = 200;  // 目標付近の最短タイムアウト（近い時はより短く）
	private static final long AUTO_TURN_TIMEOUT_MS_MAX = 3000; // 反対側(≈180°)の最長タイムアウト（遠い時はより長く）
	private static final double AUTO_TURN_TIMEOUT_SCALE = 0.70; // タイムアウト全体スケール（70%）
	// 自動整向の回転出力スケーリング（角度差でKPと最大出力を補間）
	private static final double HEADING_KP_NEAR = 0.004; // 小角度用（ゆっくり）
	private static final double HEADING_KP_FAR  = 0.012; // 大角度用（強め）
	private static final double HEADING_MAX_NEAR = 0.20; // 小角度時の上限
	private static final double HEADING_MAX_FAR  = 0.80; // 大角度時の上限
	
	// アナログ入力（ポテンショメータ）
	private AnalogInput poten;
	// IMU センサー
	private IMU imu;
	// SHARE/BACKでヘディング記憶
	private boolean prevSharePressed = false;
	private double storedHeadingDeg = 0.0;
	private boolean hasStoredHeading = false;
	// OPTIONS整向用のラッチ状態（目標到達後は押し続けても補正しない）
	private boolean prevOptionsPressed = false;
	private boolean headingLocked = false;
	private boolean isAutoTurnActive = false;
	private long autoTurnStartMs = 0L;
	private long autoTurnTimeoutMs = AUTO_TURN_TIMEOUT_MS; // 実行毎に設定される動的タイムアウト
	private int autoTurnPhase = 0; // 0:なし, 1:第1段, 2:第2段
	private boolean autoTurnPhaseHold = false; // フェーズ切替直後に一度停止

	private String getHeadingDirection(double yawDegrees) {
		double normalizedYaw = ((yawDegrees % 360) + 360) % 360;
		if (normalizedYaw >= 337.5 || normalizedYaw < 22.5) return "NORTH";
		if (normalizedYaw < 67.5) return "NORTHEAST";
		if (normalizedYaw < 112.5) return "EAST";
		if (normalizedYaw < 157.5) return "SOUTHEAST";
		if (normalizedYaw < 202.5) return "SOUTH";
		if (normalizedYaw < 247.5) return "SOUTHWEST";
		if (normalizedYaw < 292.5) return "WEST";
		return "NORTHWEST";
	}

	@Override
	public void runOpMode() {
		// Control Hub のモーター初期化（HD Hex Motor）
		leftMotor = hardwareMap.get(DcMotorEx.class, "left_motor");
		rightMotor = hardwareMap.get(DcMotorEx.class, "right_motor");
		topMotor = hardwareMap.get(DcMotorEx.class, "top_motor");

		// Expansion Hub のモーター初期化
		armMotor = hardwareMap.get(DcMotorEx.class, "arm_motor");  // HD Hex Motor
		liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");   // Core Hex Motor

		// アナログ入力（poten）初期化（Analogue Input Port 0）
		try {
			poten = hardwareMap.get(AnalogInput.class, "poten");
		} catch (Exception e) {
			poten = null;
		}

		// モーターの設定
		if (leftMotor != null) {
			leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		}
		if (rightMotor != null) {
			rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		}
		if (topMotor != null) {
			topMotor.setDirection(DcMotor.Direction.REVERSE);
			topMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			topMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
			System.out.println("Top Motor initialized successfully");
		} else {
			System.out.println("Top Motor initialization failed - motor is null");
		}
		if (armMotor != null) {
			armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		}
		if (liftMotor != null) {
			liftMotor.setDirection(DcMotor.Direction.REVERSE);
			liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		}

		// IMU 初期化
		try {
			imu = hardwareMap.get(IMU.class, "imu");
			if (imu != null) {
				IMU.Parameters params = new IMU.Parameters(
					new RevHubOrientationOnRobot(
						RevHubOrientationOnRobot.LogoFacingDirection.UP,
						RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
					)
				);
				imu.initialize(params);
				telemetry.addData("IMU", "Initialized");
			} else {
				telemetry.addData("IMU", "NOT FOUND");
			}
		} catch (Exception e) {
			telemetry.addData("IMU Init Error", e.getMessage());
		}

		// 初期化デバッグ情報を表示
		telemetry.addData("=== MOTOR INITIALIZATION DEBUG ===", "");
		telemetry.addData("Left Motor", leftMotor != null ? "OK" : "NULL");
		telemetry.addData("Right Motor", rightMotor != null ? "OK" : "NULL");
		telemetry.addData("Top Motor", topMotor != null ? "OK" : "NULL");
		telemetry.addData("Arm Motor", armMotor != null ? "OK" : "NULL");
		telemetry.addData("Lift Motor", liftMotor != null ? "OK" : "NULL");
		telemetry.addData("=== INITIALIZATION COMPLETE ===", "");
		telemetry.update();
		
		// 初期化完了を待つ
		waitForStart();

		while (opModeIsActive()) {
			// ゲームパッド入力の取得
			double leftY = gamepad1.left_stick_y;
			double rightY = gamepad1.right_stick_y;
			double rightX = gamepad1.right_stick_x;
			double leftTrigger = gamepad1.left_trigger;
			double rightTrigger = gamepad1.right_trigger;
			boolean leftBumper = gamepad1.left_bumper;
			boolean rightBumper = gamepad1.right_bumper;
			boolean m1Button = gamepad1.left_stick_button; // M1 ボタン相当
			boolean sharePressed = gamepad1.back; // SHARE/BACK ボタン
			boolean optionsPressed = gamepad1.start; // OPTIONS ボタン
			
			// デッドゾーン設定
			if (Math.abs(leftY) < 0.1) leftY = 0.0;
			if (Math.abs(rightY) < 0.1) rightY = 0.0;
			if (leftTrigger < 0.1) leftTrigger = 0.0;
			if (rightTrigger < 0.1) rightTrigger = 0.0;

			// ポテンショ正規化値（0.0〜1.0）
			double potenNorm = -1.0;
			if (poten != null && poten.getMaxVoltage() > 0) {
				potenNorm = poten.getVoltage() / poten.getMaxVoltage();
			}

			// SHAREで現在ヘディングを記憶
			if (imu != null) {
				double yawNowDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
				if (sharePressed && !prevSharePressed) {
					storedHeadingDeg = yawNowDeg;
					hasStoredHeading = true;
				}
			}
			prevSharePressed = sharePressed;

			// M1 コンボ（M1押下中はliftを上げ、topを回す）
			boolean m1ComboActive = false;
			if (m1Button) {
				m1ComboActive = true;
				if (liftMotor != null) {
					double liftPower = -0.8; // 上方向固定値（反転）
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
				if (topMotor != null) {
					// コンボ時は強制的に一定速度で回す
					currentTopPower = 1.0;
					targetTopPower = 1.0;
					topMotor.setPower(currentTopPower);
				}
			}
			
			// モーター制御
			// ホイールモーター（右Joystick 前後: Y、旋回: X）
			if (leftMotor != null && rightMotor != null) {
				double forward = rightY;	  // 既存の前後（上で前進）
				// 感度調整：デッドゾーン、指数カーブ、ゲイン
				double shapedTurn;
				double absX = Math.abs(rightX);
				if (absX < TURN_DEADZONE) {
					shapedTurn = 0.0;
				} else {
					double norm = (absX - TURN_DEADZONE) / (1.0 - TURN_DEADZONE);
					double curved = Math.pow(norm, TURN_EXPONENT);
					shapedTurn = Math.copySign(curved, rightX) * TURN_GAIN;
				}
				final double TURN_NEUTRAL_BAND = 0.08; // 前後判定用ヒステリシス帯
				boolean isForward = forward > TURN_NEUTRAL_BAND;
				boolean isBackward = forward < -TURN_NEUTRAL_BAND;
				boolean isNeutral = !isForward && !isBackward;
								 // 要求仕様:
				 // 1) 前進時: 現在の左右回転を反転（rawTurnを使用）
				 // 2) 静止時: 少しのヒステリシス内で前進と同じ扱い（rawTurn）
				 // 3) 後退時: 既存の動きが望ましいので変更なし（-rawTurn）
				 double turn = (isForward || isBackward) ? shapedTurn : shapedTurn;
  
				  // 左に倒すと左回転（左モータ減、右モータ増）
				double leftPower = forward - turn;
				double rightPower = -forward - turn; // 現在の構成を維持

				// 手動入力優先: 自動整向中でも右スティック/バンパー入力があれば即キャンセル
				if (isAutoTurnActive) {
					boolean hasManualDriveInput = (Math.abs(rightY) > 0.1) || (Math.abs(rightX) > TURN_DEADZONE) || leftBumper || rightBumper;
					if (hasManualDriveInput) {
						isAutoTurnActive = false;
						headingLocked = false;
						autoTurnPhase = 0;
						autoTurnPhaseHold = false;
					}
				}
 
				 // バンパー優先のその場旋回（L1=左回転, R1=右回転）
				 if (leftBumper ^ rightBumper) {
					 double rot = leftBumper ? -ROTATE_BUMPER_POWER : ROTATE_BUMPER_POWER;
					 // その場旋回：左右対称に反対向き（ユーザー指定：両方 -rot）
					 leftPower = -rot;
					 rightPower = -rot;
				 }
 
				 // OPTIONS立ち上がりで自動整向を開始（押しっぱなしは無視）
				 if (optionsPressed && !prevOptionsPressed && hasStoredHeading && imu != null) {
					 isAutoTurnActive = true;
					 headingLocked = false;
					 autoTurnStartMs = System.currentTimeMillis();
					 // 初期誤差に応じてタイムアウト時間を設定（近いほど短く、遠いほど長く）
					 double yawDegAtStart = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
					 double errStart = (storedHeadingDeg - yawDegAtStart);
					 errStart = ((errStart + 540) % 360) - 180;
					 double absErrStart = Math.abs(errStart); // 0〜180
					 double tRatio = Math.min(1.0, Math.max(0.0, absErrStart / 180.0));
					 // 正マッピング: 近い→短い, 遠い→長い
					 autoTurnTimeoutMs = AUTO_TURN_TIMEOUT_MS_MIN + Math.round((AUTO_TURN_TIMEOUT_MS_MAX - AUTO_TURN_TIMEOUT_MS_MIN) * tRatio);
					 autoTurnTimeoutMs = Math.round(autoTurnTimeoutMs * AUTO_TURN_TIMEOUT_SCALE);
					 autoTurnPhase = 1; // 第1段開始
					 autoTurnPhaseHold = false;
				 }
				 prevOptionsPressed = optionsPressed;

				// 自動整向（±2°以内で停止）
				if (isAutoTurnActive && hasStoredHeading && imu != null) {
					// タイムアウト判定
					long elapsed = System.currentTimeMillis() - autoTurnStartMs;
					if (elapsed >= autoTurnTimeoutMs) {
						leftPower = 0.0;
						rightPower = 0.0;
						if (autoTurnPhase == 1) {
							// 第1段タイムアウト直後：一旦停止して第2段を自動開始
							autoTurnPhase = 2;
							autoTurnPhaseHold = true; // 1サイクル停止
							// 第2段の開始時間とタイムアウト再設定（同じロジック）
							autoTurnStartMs = System.currentTimeMillis();
							double yawDegAtPhase2 = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
							double err2 = ((storedHeadingDeg + 180.0) - yawDegAtPhase2);
							err2 = ((err2 + 540) % 360) - 180;
							double absErr2 = Math.abs(err2);
							double r2 = Math.min(1.0, Math.max(0.0, absErr2 / 180.0));
							double inv2 = 1.0 - r2;
							autoTurnTimeoutMs = AUTO_TURN_TIMEOUT_MS_MIN + Math.round((AUTO_TURN_TIMEOUT_MS_MAX - AUTO_TURN_TIMEOUT_MS_MIN) * inv2);
							autoTurnTimeoutMs = Math.round(autoTurnTimeoutMs * AUTO_TURN_TIMEOUT_SCALE);
						} else {
							// 第2段も終了 → 完全停止
							isAutoTurnActive = false;
							headingLocked = true;
						}
					} else {
						// すでにロック済みなら左右を常に停止
						if (headingLocked) {
							leftPower = 0.0;
							rightPower = 0.0;
						} else {
							double yawDeg = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
							double error = ((storedHeadingDeg + 180.0) - yawDeg);
							// -180..+180 に正規化
							error = ((error + 540) % 360) - 180;
							double absErr = Math.abs(error);
							// フェーズ切替直後は一度停止してから反転方向へ
							if (autoTurnPhaseHold) {
								leftPower = 0.0;
								rightPower = 0.0;
								autoTurnPhaseHold = false; // 次サイクルから制御
							} else {
							if (absErr <= 2.0) { // 許容誤差±2°
								leftPower = 0.0;
								rightPower = 0.0;
								headingLocked = true; // 以降は押し続けても補正しない
								isAutoTurnActive = false; // 自動整向完了
							} else {
								// 誤差に比例（角度差に応じてKP/上限を補間）
								double ratio = Math.min(1.0, Math.max(0.0, absErr / 180.0));
								double kp = HEADING_KP_NEAR + (HEADING_KP_FAR - HEADING_KP_NEAR) * ratio;
								double maxOut = HEADING_MAX_NEAR + (HEADING_MAX_FAR - HEADING_MAX_NEAR) * ratio;
								double cmd = kp * absErr;
								if (cmd > maxOut) cmd = maxOut;   // 上限
								double rot = Math.copySign(cmd, error); // error>0 なら左回り
								// ユーザー指定：left=-rot, right=-rot
								leftPower = -rot;
								rightPower = -rot;
							}
							}
						}
					}
				}

				// パワー値を制限
				leftPower = Math.max(-1.0, Math.min(1.0, leftPower));
				rightPower = Math.max(-1.0, Math.min(1.0, rightPower));

				leftMotor.setPower(leftPower);
				rightMotor.setPower(rightPower);
			}

			// トップモーター（R2トリガー）- 徐々にパワーを増加
			if (topMotor != null && !m1ComboActive) {
				// 目標パワーを設定
				targetTopPower = rightTrigger;
				
				// 徐々にパワーを調整
				if (targetTopPower > currentTopPower) {
					// パワーを増加（ゆっくり）
					currentTopPower = Math.min(targetTopPower, currentTopPower + POWER_INCREMENT);
				} else if (targetTopPower < currentTopPower) {
					// パワーを減少（少し速く）
					currentTopPower = Math.max(targetTopPower, currentTopPower - POWER_DECREMENT);
				}
				
				// モーターにパワーを設定
				topMotor.setPower(currentTopPower);
				System.out.println("Top Motor Target: " + targetTopPower + ", Current: " + currentTopPower);
			} else {
				System.out.println("Top Motor is NULL!");
			}

			// アームモーター（L2トリガー）
			if (armMotor != null) {
				double armPower = m1ComboActive ? 0.0 : leftTrigger; // M1中は強制停止
				armMotor.setPower(armPower);
			}

			// リフトモーター（左Joystick Y軸）
			if (liftMotor != null && !m1ComboActive) {
				liftMotor.setPower(leftY);
			}

			// === IMU Telemetry ===
			if (imu != null) {
				YawPitchRollAngles ypr = imu.getRobotYawPitchRollAngles();
				AngularVelocity angVel = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
				double yawDeg = ypr.getYaw(AngleUnit.DEGREES);
				telemetry.addData("=== IMU ===", "");
				telemetry.addData("Yaw (Z)", "%.2f°", yawDeg);
				telemetry.addData("Pitch (Y)", "%.2f°", ypr.getPitch(AngleUnit.DEGREES));
				telemetry.addData("Roll (X)", "%.2f°", ypr.getRoll(AngleUnit.DEGREES));
				telemetry.addData("AngVel Z", "%.2f°/s", angVel.zRotationRate);
				telemetry.addData("Heading", "%s (%.1f°)", getHeadingDirection(yawDeg), yawDeg);
				if (hasStoredHeading) {
					telemetry.addData("Stored Heading", "%s (%.1f°)", getHeadingDirection(storedHeadingDeg), storedHeadingDeg);
				} else {
					telemetry.addData("Stored Heading", "NONE - press SHARE/BACK to store");
				}
			} else {
				telemetry.addData("IMU", "NOT CONNECTED");
			}

			// テレメトリ表示
			telemetry.addData("=== MOTOR STATUS ===", "");
			telemetry.addData("Left Motor", leftMotor != null ? "OK" : "NULL");
			telemetry.addData("Right Motor", rightMotor != null ? "OK" : "NULL");
			telemetry.addData("Top Motor", topMotor != null ? "OK" : "NULL");
			telemetry.addData("Arm Motor", armMotor != null ? "OK" : "NULL");
			telemetry.addData("Lift Motor", liftMotor != null ? "OK" : "NULL");
			telemetry.addData("Heading Lock", headingLocked ? "ON" : "OFF");
			telemetry.addData("=== MOTOR POWER ===", "");
			telemetry.addData("Left Motor Power", "%.2f", leftMotor != null ? leftMotor.getPower() : 0.0);
			telemetry.addData("Right Motor Power", "%.2f", rightMotor != null ? rightMotor.getPower() : 0.0);
			telemetry.addData("Top Motor Target", "%.2f", targetTopPower);
			telemetry.addData("Top Motor Current", "%.2f", currentTopPower);
			telemetry.addData("Arm Motor Power", "%.2f", armMotor != null ? armMotor.getPower() : 0.0);
			telemetry.addData("Lift Motor Power", "%.2f", liftMotor != null ? liftMotor.getPower() : 0.0);
			telemetry.addData("M1 Combo Active", m1ComboActive ? "YES" : "NO");
			if (m1ComboActive) {
				telemetry.addData("M1 Lift Forced Power", "%.2f", -0.8);
				telemetry.addData("M1 Top Forced Power", "%.2f", 1.0);
			}
			telemetry.addData("=== GAMEPAD INPUT ===", "");
			telemetry.addData("Left Stick Y", "%.2f", leftY);
			telemetry.addData("Right Stick Y", "%.2f", rightY);
			telemetry.addData("Left Trigger", "%.2f", leftTrigger);
			telemetry.addData("Right Trigger", "%.2f", rightTrigger);
			telemetry.addData("=== ANALOG INPUT ===", "");
			if (poten != null) {
				double volts = poten.getVoltage();
				double raw = poten.getMaxVoltage() > 0 ? poten.getVoltage() / poten.getMaxVoltage() : 0.0;
				telemetry.addData("poten Voltage", "%.3f V", volts);
				telemetry.addData("poten Normalized", "%.3f", raw);
				telemetry.addData("Lift Limit Norm", "%.2f (hyst: %.2f)", LIFT_LIMIT_NORM, LIFT_LIMIT_HYST);
				telemetry.addData("Lift Hold Power", "%.2f", LIFT_HOLD_POWER);
			} else {
				telemetry.addData("poten", "NOT CONNECTED (Analog Port 0)");
			}
			telemetry.addData("=== DEBUG INFO ===", "");
			if (topMotor != null) {
				telemetry.addData("Top Motor Status", "Connected");
				telemetry.addData("Top Motor Direction", "REVERSE");
				telemetry.addData("Top Motor Mode", "RUN_WITHOUT_ENCODER");
				telemetry.addData("Power Increment", "%.2f", POWER_INCREMENT);
				telemetry.addData("Power Decrement", "%.2f", POWER_DECREMENT);
			} else {
				telemetry.addData("Top Motor Status", "NOT CONNECTED");
				telemetry.addData("Top Motor Error", "Check Robot Configuration");
			}
			telemetry.update();

			sleep(10); // 100Hz loop
		}
	}
} 