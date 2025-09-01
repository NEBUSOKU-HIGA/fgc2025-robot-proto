package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

public class A1ImuMotor {
	private IMU imu;
	private DcMotorEx leftMotor;
	private DcMotorEx rightMotor;
	private boolean isInitialized = false;
	
	// IMUデータ
	private double yaw = 0.0;
	private double pitch = 0.0;
	private double roll = 0.0;
	private double angularVelocityZ = 0.0;
	
	// SHARE/BACKでヘディング記憶
	private boolean prevSharePressed = false;
	private boolean prevOptionsPressed = false;
	private double storedHeadingDeg = 0.0;
	private boolean hasStoredHeading = false;
	
	// エラー情報
	private String lastError = "";
	private boolean hasError = false;
	
	// 自動整向機能
	private boolean isAutoTurnActive = false;
	private boolean headingLocked = false;
	private long autoTurnStartMs = 0;
	private long autoTurnTimeoutMs = 0;
	private int autoTurnPhase = 0; // 1: 第1段, 2: 第2段
	private boolean autoTurnPhaseHold = false;

	private static final long AUTO_TURN_TIMEOUT_MS_MIN = 1000; // 最小タイムアウト (ms)
	private static final long AUTO_TURN_TIMEOUT_MS_MAX = 5000; // 最大タイムアウト (ms)
	private static final double AUTO_TURN_TIMEOUT_SCALE = 0.5; // タイムアウトのスケーリング係数
	private static final double HEADING_KP_NEAR = 0.05; // 近い場合のKP
	private static final double HEADING_KP_FAR = 0.01; // 遠い場合のKP
	private static final double HEADING_MAX_NEAR = 0.5; // 近い場合の最大出力
	private static final double HEADING_MAX_FAR = 0.2; // 遠い場合の最大出力

	public A1ImuMotor(HardwareMap hardwareMap) {
		initializeImu(hardwareMap);
		initializeMotors(hardwareMap);
	}
	
	private void initializeImu(HardwareMap hardwareMap) {
		try {
			imu = hardwareMap.get(IMU.class, "imu");
			if (imu != null) {
				// BHI260AP IMU用の設定（動作していたコードを参考）
				IMU.Parameters parameters = new IMU.Parameters(
					new RevHubOrientationOnRobot(
						RevHubOrientationOnRobot.LogoFacingDirection.UP,
						RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
					)
				);
				imu.initialize(parameters);
				isInitialized = true;
				System.out.println("A1ImuMotor: BHI260AP IMU initialized successfully");
			} else {
				hasError = true;
				lastError = "BHI260AP IMU not found in hardware map";
				System.out.println("A1ImuMotor: ERROR - BHI260AP IMU not found in hardware map");
			}
		} catch (Exception e) {
			hasError = true;
			lastError = "BHI260AP IMU initialization failed: " + e.getMessage();
			System.out.println("A1ImuMotor: ERROR - BHI260AP IMU initialization failed: " + e.getMessage());
		}
	}
	
	private void initializeMotors(HardwareMap hardwareMap) {
		try {
			leftMotor = hardwareMap.get(DcMotorEx.class, "left_motor");
			rightMotor = hardwareMap.get(DcMotorEx.class, "right_motor");
			
			if (leftMotor != null) {
				leftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
				leftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
				leftMotor.setDirection(DcMotorEx.Direction.FORWARD);
				leftMotor.setPower(0.0);
			}
			if (rightMotor != null) {
				rightMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
				rightMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
				rightMotor.setDirection(DcMotorEx.Direction.REVERSE);
				rightMotor.setPower(0.0);
			}
			
			System.out.println("A1ImuMotor: Motors initialized successfully");
		} catch (Exception e) {
			hasError = true;
			lastError = "Motor initialization failed: " + e.getMessage();
			System.out.println("A1ImuMotor: ERROR - Motor initialization failed: " + e.getMessage());
		}
	}
	
	public void updateImuData() {
		if (!isInitialized || imu == null) {
			return;
		}
		
		try {
			// Yaw, Pitch, Roll角度を取得
			YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
			yaw = orientation.getYaw(AngleUnit.DEGREES);
			pitch = orientation.getPitch(AngleUnit.DEGREES);
			roll = orientation.getRoll(AngleUnit.DEGREES);
			
			// 角速度を取得
			AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
			angularVelocityZ = angularVelocity.zRotationRate;
			
			hasError = false;
			lastError = "";
			
		} catch (Exception e) {
			hasError = true;
			lastError = "IMU data read error: " + e.getMessage();
			System.out.println("A1ImuMotor: ERROR reading IMU data - " + e.getMessage());
		}
	}
	
	// SHAREボタンでヘディングを記憶
	public void updateHeadingMemory(boolean sharePressed) {
		if (imu != null) {
			double yawNowDeg = getCurrentYaw();
			if (sharePressed && !prevSharePressed) {
				storedHeadingDeg = yawNowDeg;
				hasStoredHeading = true;
				System.out.println("A1ImuMotor: Heading saved: " + storedHeadingDeg + "°");
			}
		}
		prevSharePressed = sharePressed;
	}
	
	// OPTIONSボタンで自動整向を開始
	public void startAutoTurn(boolean optionsPressed) {
		if (optionsPressed && !prevOptionsPressed && hasStoredHeading && imu != null) {
			isAutoTurnActive = true;
			headingLocked = false;
			autoTurnStartMs = System.currentTimeMillis();
			
			// 初期誤差に応じてタイムアウト時間を設定（近いほど短く、遠いほど長く）
			double yawDegAtStart = getCurrentYaw();
			double errStart = (storedHeadingDeg - yawDegAtStart);
			errStart = ((errStart + 540) % 360) - 180;
			double absErrStart = Math.abs(errStart); // 0〜180
			double tRatio = Math.min(1.0, Math.max(0.0, absErrStart / 180.0));
			// 正マッピング: 近い→短い, 遠い→長い
			autoTurnTimeoutMs = AUTO_TURN_TIMEOUT_MS_MIN + Math.round((AUTO_TURN_TIMEOUT_MS_MAX - AUTO_TURN_TIMEOUT_MS_MIN) * tRatio);
			autoTurnTimeoutMs = Math.round(autoTurnTimeoutMs * AUTO_TURN_TIMEOUT_SCALE);
			autoTurnPhase = 1; // 第1段開始
			autoTurnPhaseHold = false;
			
			System.out.println("A1ImuMotor: Auto turn started - Target: " + storedHeadingDeg + "°, Current: " + yawDegAtStart + "°, Error: " + errStart + "°");
		}
		prevOptionsPressed = optionsPressed;
	}
	
	// 自動整向の制御値を計算
	public double[] calculateAutoTurnControl() {
		double[] result = {0.0, 0.0}; // [leftPower, rightPower]
		
		if (!isAutoTurnActive || !hasStoredHeading || imu == null) {
			return result;
		}
		
		// タイムアウト判定
		long elapsed = System.currentTimeMillis() - autoTurnStartMs;
		if (elapsed >= autoTurnTimeoutMs) {
			result[0] = 0.0;
			result[1] = 0.0;
			if (autoTurnPhase == 1) {
				// 第1段タイムアウト直後：一旦停止して第2段を自動開始
				autoTurnPhase = 2;
				autoTurnPhaseHold = true; // 1サイクル停止
				// 第2段の開始時間とタイムアウト再設定（同じロジック）
				autoTurnStartMs = System.currentTimeMillis();
				double yawDegAtPhase2 = getCurrentYaw();
				double err2 = ((storedHeadingDeg + 180.0) - yawDegAtPhase2);
				err2 = ((err2 + 540) % 360) - 180;
				double absErr2 = Math.abs(err2);
				double r2 = Math.min(1.0, Math.max(0.0, absErr2 / 180.0));
				double inv2 = 1.0 - r2;
				autoTurnTimeoutMs = AUTO_TURN_TIMEOUT_MS_MIN + Math.round((AUTO_TURN_TIMEOUT_MS_MAX - AUTO_TURN_TIMEOUT_MS_MIN) * inv2);
				autoTurnTimeoutMs = Math.round(autoTurnTimeoutMs * AUTO_TURN_TIMEOUT_SCALE);
				
				System.out.println("A1ImuMotor: Auto turn phase 2 started - Target: " + (storedHeadingDeg + 180.0) + "°, Current: " + yawDegAtPhase2 + "°, Error: " + err2 + "°");
			} else {
				// 第2段も終了 → 完全停止
				isAutoTurnActive = false;
				headingLocked = true;
				System.out.println("A1ImuMotor: Auto turn completed");
			}
		} else {
			// すでにロック済みなら左右を常に停止
			if (headingLocked) {
				result[0] = 0.0;
				result[1] = 0.0;
			} else {
				double yawDeg = getCurrentYaw();
				double error = ((storedHeadingDeg + 180.0) - yawDeg);
				// -180..+180 に正規化
				error = ((error + 540) % 360) - 180;
				double absErr = Math.abs(error);
				
				// フェーズ切替直後は一度停止してから反転方向へ
				if (autoTurnPhaseHold) {
					result[0] = 0.0;
					result[1] = 0.0;
					autoTurnPhaseHold = false; // 次サイクルから制御
				} else {
					if (absErr <= 2.0) { // 許容誤差±2°
						result[0] = 0.0;
						result[1] = 0.0;
						headingLocked = true; // 以降は押し続けても補正しない
						isAutoTurnActive = false; // 自動整向完了
						System.out.println("A1ImuMotor: Auto turn target reached - Error: " + absErr + "°");
					} else {
						// 誤差に比例（角度差に応じてKP/上限を補間）
						double ratio = Math.min(1.0, Math.max(0.0, absErr / 180.0));
						double kp = HEADING_KP_NEAR + (HEADING_KP_FAR - HEADING_KP_NEAR) * ratio;
						double maxOut = HEADING_MAX_NEAR + (HEADING_MAX_FAR - HEADING_MAX_NEAR) * ratio;
						double cmd = kp * absErr;
						if (cmd > maxOut) cmd = maxOut;   // 上限
						double rot = Math.copySign(cmd, error); // error>0 なら左回り
						// 旋回制御：left=-rot, right=+rot（左右逆方向で旋回）
						result[0] = -rot;
						result[1] = +rot;
					}
				}
			}
		}
		
		return result;
	}
	
	// 手動入力で自動整向をキャンセル
	public void cancelAutoTurn(boolean hasManualInput) {
		if (isAutoTurnActive && hasManualInput) {
			isAutoTurnActive = false;
			headingLocked = false;
			autoTurnPhase = 0;
			autoTurnPhaseHold = false;
			System.out.println("A1ImuMotor: Auto turn cancelled by manual input");
		}
	}
	
	// 現在のYaw角度を取得
	public double getCurrentYaw() {
		if (imu != null) {
			try {
				return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
			} catch (Exception e) {
				hasError = true;
				lastError = "Error getting Yaw: " + e.getMessage();
				return 0.0;
			}
		}
		return 0.0;
	}
	
	// ヘディング方向を文字列で取得
	public String getHeadingDirection(double yawDegrees) {
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
	
	// 通常のドライブ制御
	public void drive(double rightStickY, double rightStickX, boolean turnLeft, boolean turnRight) {
		// 厳密なデッドゾーン設定
		if (Math.abs(rightStickY) < 0.05) rightStickY = 0.0;
		if (Math.abs(rightStickX) < 0.05) rightStickX = 0.0;

		double leftPower = 0.0;
		double rightPower = 0.0;

		// 前後移動制御（右スティックY軸）- 符号修正
		if (Math.abs(rightStickY) >= 0.05) {
			leftPower = -rightStickY;   // 左モーター（前進で正転）- 符号反転
			rightPower = rightStickY;   // 右モーター（前進で正転）
		}

		// 左右回転制御（右スティックX軸）- 符号修正
		if (Math.abs(rightStickX) >= 0.05) {
			leftPower += -rightStickX;  // 左モーター（右回転で正転）- 符号反転
			rightPower -= -rightStickX; // 右モーター（右回転で逆転）- 符号反転
		}

		// 左右回転制御（L1/R1ボタン）- 片方のモーター方向修正
		if (turnLeft) {
			leftPower += 0.5;   // 左回転（L1）- 左モーター正転
			rightPower += 0.5;  // 左回転（L1）- 右モーター正転（修正）
		}
		if (turnRight) {
			leftPower -= 0.5;   // 右回転（R1）- 左モーター逆転
			rightPower -= 0.5;  // 右回転（R1）- 右モーター逆転（修正）
		}

		// パワー値を-1.0から1.0の範囲に制限
		leftPower = Math.max(-1.0, Math.min(1.0, leftPower));
		rightPower = Math.max(-1.0, Math.min(1.0, rightPower));

		// 安全機能：入力がない場合は確実に0.0に設定
		if (Math.abs(rightStickY) < 0.05 && Math.abs(rightStickX) < 0.05 && !turnLeft && !turnRight) {
			leftPower = 0.0;
			rightPower = 0.0;
		}

		// モーターにパワーを設定
		if (leftMotor != null) {
			leftMotor.setPower(leftPower);
		}
		if (rightMotor != null) {
			rightMotor.setPower(rightPower);
		}
	}
	
	// Getter methods
	public double getYaw() { return yaw; }
	public double getPitch() { return pitch; }
	public double getRoll() { return roll; }
	public double getAngularVelocityZ() { return angularVelocityZ; }
	public double getHeading() { return yaw; } // Yawと同じ値としてheadingを返す
	public boolean isInitialized() { return isInitialized; }
	public boolean hasError() { return hasError; }
	public String getLastError() { return lastError; }
	public double getLeftMotorPower() { return leftMotor != null ? leftMotor.getPower() : 0.0; }
	public double getRightMotorPower() { return rightMotor != null ? rightMotor.getPower() : 0.0; }
	public boolean isLeftMotorConnected() { return leftMotor != null; }
	public boolean isRightMotorConnected() { return rightMotor != null; }
	
	// ヘディング記憶関連のメソッド
	public boolean hasStoredHeading() { return hasStoredHeading; }
	public double getStoredHeading() { return storedHeadingDeg; }
	
	// 自動整向関連のメソッド
	public boolean isAutoTurnActive() { return isAutoTurnActive; }
	public boolean isHeadingLocked() { return headingLocked; }
	
	// モーターへの直接アクセス（自動整向用）
	public DcMotorEx getLeftMotor() { return leftMotor; }
	public DcMotorEx getRightMotor() { return rightMotor; }
} 