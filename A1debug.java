package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.A1Drive;
import org.firstinspires.ftc.teamcode.A1Flask;
import org.firstinspires.ftc.teamcode.A1Arm;
import org.firstinspires.ftc.teamcode.A1Lift;
import org.firstinspires.ftc.teamcode.A1Potension;
import org.firstinspires.ftc.teamcode.A1Servo;
import org.firstinspires.ftc.teamcode.A1Top2;
import org.firstinspires.ftc.teamcode.A1Imu;
import org.firstinspires.ftc.teamcode.A1ImuMotor;


@TeleOp(name="A1debug", group="Test")
public class A1debug extends LinearOpMode {

	// A1システムのインスタンス
	private A1ImuMotor imuMotorSystem;
	private A1Flask flaskSystem = null; // Flaskはオプション
	private A1Arm armSystem;
	private A1Lift liftSystem;
	private A1Potension potensionSystem;
	private A1Servo servoSystem;
	private A1Top2 topSystem;
	
	// ボタンの状態管理（チャタリング防止）
	private boolean lastShareButton = false;
	private boolean lastOptionsButton = false;
	
	// △ボタン前進スピード制御用変数
	private double triangleForwardSpeed = 0.3; // デフォルト前進スピード（0.0〜1.0）
	private static final double TRIANGLE_SPEED_INCREMENT = 0.1; // スピード変更の増減値
	
	// R1アーム位置制御用変数
	private double r1ArmTargetHeight = 0.55; // デフォルトアーム目標高さ（0.0〜1.0）
	private static final double R1_ARM_HEIGHT_INCREMENT = 0.05; // 高さ変更の増減値
	
	// アーム自重補正用変数
	private double armHoldPower = -0.15; // アーム自重を支えるリフトモーターパワー（デフォルト-0.15）
	private static final double ARM_HOLD_POWER_INCREMENT = 0.02; // 保持パワー変更の増減値
	
	// R2ボタン用リフト制御変数
	private double r2LiftValue = -0.55; // R2ボタンでシミュレートするleft joystick Yの値（デフォルト-0.55）
	private static final double R2_LIFT_VALUE_INCREMENT = 0.05; // 値変更の増減値
	
	// アーム位置保持制御用変数
	private double armHoldMaxPower = -0.15; // アーム位置保持 최대 출력（デフォルト-0.15 = 15%）
	private static final double ARM_HOLD_MAX_POWER_INCREMENT = 0.02; // 保持 최대 출력 변경の増減値
	private static final double ARM_HOLD_MIN_HEIGHT = 0.34; // 位置保持 시작 높이（0.34）
	private static final double ARM_HOLD_MAX_HEIGHT = 0.56; // 位置保持 최대 출력 높이（0.56）
	
	// R2 servo 지연 제어는 제거됨 (top_motor와 servo 독립 제어)
	

	
	// Servo1 위치 변경 추적용 변수
	private double lastServo1Position = 0.0;
	
	// 크랩 드라이브 제어용 변수
	private boolean crabDriveActive = false; // 크랩 드라이브 활성화 상태
	private double crabDriveTargetHeading = 0.0; // 크랩 드라이브 목표 헤딩
	private long crabDriveStartTime = 0; // 크랩 드라이브 시작 시간
	private static final long CRAB_DRIVE_DURATION = 800; // 후진 단계 지속 시간 (ms)
	private static final double CRAB_DRIVE_POWER = 0.6; // 크랩 드라이브 파워 (60%)
	private static final double CRAB_DRIVE_BACKWARD_POWER = 0.7; // 후진 파워 (70%) - 조절 가능
	private static final double CRAB_DRIVE_FORWARD_POWER = 0.4; // 전진 파워 (40%) - 조절 가능
	private static final double CRAB_DRIVE_DISTANCE = 0.03; // 크랩 드라이브 거리 (3cm = 0.03m)
	private static final double CRAB_DRIVE_HEADING_STOP_RATIO = 0.4; // 타겟 헤딩 도달 전 40%에서 정지 (오버슈팅 방지) - 미세조정 가능
	
	// D-pad 위쪽 버튼 천천히 전진 제어용 변수
	private static final double DPAD_UP_FORWARD_POWER = 0.15; // D-pad 위쪽 버튼 전진 출력 (15%) - 조정 가능
	
	/**
	 * アーム位置保持制御：암의 자중에 의한 하강을 방지하는 위치 유지 제어（비선형 제어）
	 * @param currentHeight 현재 암 높이 (normalized potentiometer value)
	 * @return 위치 유지에 필요한 lift_motor 출력값
	 */
	private double calculateArmHoldPower(double currentHeight) {
		// 0.34 미만이면 위치 유지 불필요
		if (currentHeight < ARM_HOLD_MIN_HEIGHT) {
			return 0.0;
		}
		
		// 0.56 이상이면 최대 출력
		if (currentHeight >= ARM_HOLD_MAX_HEIGHT) {
			return armHoldMaxPower;
		}
		
		// 0.34 ~ 0.56 사이에서는 비선형 제어로 출력 계산
		double heightRatio = (currentHeight - ARM_HOLD_MIN_HEIGHT) / (ARM_HOLD_MAX_HEIGHT - ARM_HOLD_MIN_HEIGHT);
		
		// 비선형 곡선 적용 (0.42 근처에서 더 높은 출력을 위해)
		// 제곱근 함수를 사용하여 낮은 높이에서도 더 높은 출력 제공
		double nonLinearRatio = Math.sqrt(heightRatio);
		
		// 0.42 근처에서 추가 보정 (0.42는 약 0.36의 heightRatio에 해당)
		double targetHeight = 0.42;
		double targetRatio = (targetHeight - ARM_HOLD_MIN_HEIGHT) / (ARM_HOLD_MAX_HEIGHT - ARM_HOLD_MIN_HEIGHT);
		
		if (Math.abs(heightRatio - targetRatio) < 0.1) {
			// 0.42 근처에서는 추가 30% 증가
			nonLinearRatio *= 1.3;
		}
		
		double holdPower = armHoldMaxPower * nonLinearRatio;
		
		return holdPower;
	}
	
	/**
	 * 2단계 크랩 드라이브 제어: 먼저 후진한 다음 전진하여 좌우 이동
	 * @param leftStickX gamepad1의 왼쪽 조이스틱 X축 입력 (-1.0 ~ 1.0)
	 * @param hasNormalDriveInput 일반 드라이브 입력이 있는지 여부
	 * @return 크랩 드라이브가 활성화되었는지 여부
	 */
	private boolean controlCrabDrive(double leftStickX, boolean hasNormalDriveInput) {
		// 일반 드라이브 입력이 있으면 크랩 드라이브를 즉시 중단
		if (hasNormalDriveInput) {
			if (crabDriveActive) {
				crabDriveActive = false;
				System.out.println("A1debug - Crab Drive: Interrupted by normal drive input");
			}
			return false;
		}
		
		// 데드존 설정 (0.05 이하는 무시)
		if (Math.abs(leftStickX) < 0.05) {
			if (crabDriveActive) {
				// 크랩 드라이브 종료
				crabDriveActive = false;
				System.out.println("A1debug - Crab Drive: Deactivated");
			}
			return false;
		}
		
		// 크랩 드라이브 활성화
		if (!crabDriveActive) {
			crabDriveActive = true;
			crabDriveTargetHeading = imuMotorSystem.getYaw(); // 현재 헤딩을 목표로 저장
			crabDriveStartTime = System.currentTimeMillis(); // 시작 시간 설정
			System.out.println("A1debug - Crab Drive: Started - Target Heading: " + crabDriveTargetHeading + "° - Will return to this heading in Phase 2");
		}
		
		// 크랩 드라이브 진행 시간 계산
		long elapsedTime = System.currentTimeMillis() - crabDriveStartTime;
		
		// 디버깅을 위한 시간 정보 출력
		System.out.println("A1debug - Crab Drive Debug: elapsedTime=" + elapsedTime + "ms, CRAB_DRIVE_DURATION=" + CRAB_DRIVE_DURATION + "ms");
		
		// 헤딩 보정을 위한 차동 제어
		double currentHeading = imuMotorSystem.getYaw();
		double headingError = crabDriveTargetHeading - currentHeading;
		double correctionPower = Math.max(-0.2, Math.min(0.2, headingError * 0.01));
		
		if (elapsedTime < CRAB_DRIVE_DURATION) {
			// 1단계: 후진 (800ms) - 조이스틱 입력 반전으로 선회 방향 수정
			System.out.println("A1debug - Crab Drive: PHASE 1 - BACKWARD FIRST - elapsedTime=" + elapsedTime + "ms");
			if (leftStickX < 0) { // 왼쪽 입력을 오른쪽으로 처리 (조이스틱 반전)
				// 왼쪽 모터: 전진, 오른쪽 모터: 정지 (왼쪽 앞으로 전진 - 후진 효과)
				if (imuMotorSystem.isLeftMotorConnected()) {
					imuMotorSystem.getLeftMotor().setPower(CRAB_DRIVE_BACKWARD_POWER + correctionPower);
				}
				if (imuMotorSystem.isRightMotorConnected()) {
					imuMotorSystem.getRightMotor().setPower(0.0);
				}
				System.out.println("A1debug - Crab Drive: Phase 1 - Moving left forward (LEFT input as RIGHT direction) - Left Motor: " + CRAB_DRIVE_BACKWARD_POWER + ", Right Motor: 0.0");
			} else { // 오른쪽 입력을 왼쪽으로 처리 (조이스틱 반전)
				// 왼쪽 모터: 정지, 오른쪽 모터: 전진 (오른쪽 앞으로 전진 - 후진 효과)
				if (imuMotorSystem.isLeftMotorConnected()) {
					imuMotorSystem.getLeftMotor().setPower(0.0);
				}
				if (imuMotorSystem.isRightMotorConnected()) {
					imuMotorSystem.getRightMotor().setPower(CRAB_DRIVE_BACKWARD_POWER - correctionPower);
				}
				System.out.println("A1debug - Crab Drive: Phase 1 - Moving right forward (RIGHT input as LEFT direction) - Left Motor: 0.0, Right Motor: " + CRAB_DRIVE_BACKWARD_POWER);
			}
		} else if (elapsedTime < CRAB_DRIVE_DURATION + 500) { // 후진 800ms + 전진 500ms
			// 2단계: 앞으로 전진하면서 원래 헤딩으로 복귀 - heading 보정 강화
			System.out.println("A1debug - Crab Drive: PHASE 2 - FORWARD SECOND - elapsedTime=" + elapsedTime + "ms");
			double forwardPower = Math.abs(leftStickX) * CRAB_DRIVE_FORWARD_POWER;
			
			// 2단계용 heading 정보 업데이트 (기존 변수 재할당)
			currentHeading = imuMotorSystem.getYaw();
			headingError = crabDriveTargetHeading - currentHeading;
			
			// heading 오차를 정규화 (-180 ~ 180도 범위로)
			while (headingError > 180) headingError -= 360;
			while (headingError < -180) headingError += 360;
			
			// 2단계용 heading 보정을 위한 차동 제어 (더 강한 보정)
			correctionPower = Math.max(-0.4, Math.min(0.4, headingError * 0.03));
		
		System.out.println("A1debug - Crab Drive: Phase 2 - Target Heading: " + crabDriveTargetHeading + "°, Current: " + currentHeading + "°, Error: " + headingError + "°, Correction: " + correctionPower);
		
		// 오버슈팅 방지: 타겟에 도달하기 전 70%에서 정지
		double headingStopThreshold = 2.0 * CRAB_DRIVE_HEADING_STOP_RATIO; // 2도 * 0.7 = 1.4도
		boolean shouldStopForHeading = Math.abs(headingError) < headingStopThreshold;
		
		if (leftStickX < 0) { // 왼쪽 입력을 오른쪽으로 처리 (조이스틱 반전)
			// 진행 방향 반대로: 좌우 모터 출력 파워 서로 바꿈
			// 왼쪽 모터: 후진 + 보정, 오른쪽 모터: 후진 - 보정 (기존과 반대)
			double leftMotorPower = shouldStopForHeading ? 0.0 : (-forwardPower + correctionPower);
			double rightMotorPower = shouldStopForHeading ? 0.0 : (-forwardPower - correctionPower);
			
			if (imuMotorSystem.isLeftMotorConnected()) {
				imuMotorSystem.getLeftMotor().setPower(leftMotorPower);
			}
			if (imuMotorSystem.isRightMotorConnected()) {
				imuMotorSystem.getRightMotor().setPower(rightMotorPower);
			}
			
			if (shouldStopForHeading) {
				System.out.println("A1debug - Crab Drive: Phase 2 - STOPPING for heading (LEFT input) - Target: " + crabDriveTargetHeading + "°, Current: " + currentHeading + "°, Error: " + headingError + "°, Stop Threshold: " + headingStopThreshold + "°");
			} else {
				System.out.println("A1debug - Crab Drive: Phase 2 - Moving backward (LEFT input as RIGHT direction) - REVERSED POWER - Left Motor: -" + (forwardPower - correctionPower) + ", Right Motor: -" + (forwardPower + correctionPower));
			}
		} else { // 오른쪽 입력을 왼쪽으로 처리 (조이스틱 반전)
			// heading 보정을 위한 차동 제어: 왼쪽 모터: 후진 + 보정, 오른쪽 모터: 후진 - 보정
			double leftMotorPower = shouldStopForHeading ? 0.0 : (-forwardPower + correctionPower);
			double rightMotorPower = shouldStopForHeading ? 0.0 : (-forwardPower - correctionPower);
			
			if (imuMotorSystem.isLeftMotorConnected()) {
				imuMotorSystem.getLeftMotor().setPower(leftMotorPower);
			}
			if (imuMotorSystem.isRightMotorConnected()) {
				imuMotorSystem.getRightMotor().setPower(rightMotorPower);
			}
			
			if (shouldStopForHeading) {
				System.out.println("A1debug - Crab Drive: Phase 2 - STOPPING for heading (RIGHT input) - Target: " + crabDriveTargetHeading + "°, Current: " + currentHeading + "°, Error: " + headingError + "°, Stop Threshold: " + headingStopThreshold + "°");
			} else {
				System.out.println("A1debug - Crab Drive: Phase 2 - Moving backward (RIGHT input as LEFT direction) - Left Motor: -" + (forwardPower - correctionPower) + ", Right Motor: " + (-forwardPower + correctionPower));
			}
		}
		
		// heading이 목표에 도달했는지 확인 (오버슈팅 방지 적용)
		if (Math.abs(headingError) < headingStopThreshold) { // 1.4도 이내로 도달 (70%에서 정지)
			System.out.println("A1debug - Crab Drive: Phase 2 - Target heading reached (with overshoot prevention)! Error: " + headingError + "°, Stop Threshold: " + headingStopThreshold + "°");
		}
		} else {
			// 크랩 드라이브 완료
			crabDriveActive = false;
			System.out.println("A1debug - Crab Drive: Completed - Final Heading: " + imuMotorSystem.getYaw() + "°");
		}
		return true;
	}
	

	@Override
	public void runOpMode() {
		// A1システムの初期化
		imuMotorSystem = new A1ImuMotor(hardwareMap);
		
		// Flaskの初期化（オプション）
		try {
			flaskSystem = new A1Flask();
			System.out.println("A1Task - Flask system initialized successfully");
		} catch (Exception e) {
			flaskSystem = null;
			System.out.println("A1Task - Flask system initialization failed: " + e.getMessage());
			System.out.println("A1Task - Robot will continue without Flask functionality");
		}
		
		armSystem = new A1Arm(hardwareMap);
		liftSystem = new A1Lift(hardwareMap);
		potensionSystem = new A1Potension(hardwareMap);
		servoSystem = new A1Servo(hardwareMap);
		topSystem = new A1Top2(hardwareMap);
		

		
		// 初期化完了を待つ
		waitForStart();

		while (opModeIsActive()) {
			// ゲームパッド入力値を読み込み
			double leftStickX = gamepad1.left_stick_x;
			double leftStickY = gamepad1.left_stick_y;
			double rightStickX = gamepad1.right_stick_x;
			double rightStickY = gamepad1.right_stick_y;
			
			double leftTrigger = gamepad1.left_trigger;
			double rightTrigger = gamepad1.right_trigger;
			
			boolean leftBumper = gamepad1.left_bumper;
			boolean rightBumper = gamepad1.right_bumper;
			
			boolean aButton = gamepad1.a;
			boolean bButton = gamepad1.b;
			boolean xButton = gamepad1.x;
			boolean yButton = gamepad1.y;
			
			boolean dpadUp = gamepad1.dpad_up;
			boolean dpadDown = gamepad1.dpad_down;
			boolean dpadLeft = gamepad1.dpad_left;
			boolean dpadRight = gamepad1.dpad_right;

			// A1ImuMotorでIMUデータを更新し、ドライブ制御を実行
			imuMotorSystem.updateImuData();
			
			// 일반 드라이브 입력 확인
			boolean hasNormalDriveInput = (Math.abs(rightStickY) > 0.05) || (Math.abs(rightStickX) > 0.05) || leftBumper || rightBumper || 
										 (Math.abs(gamepad2.right_stick_y) > 0.05) || (Math.abs(gamepad2.right_stick_x) > 0.05) || 
										 gamepad2.left_bumper || gamepad2.right_bumper;
			
			// 크랩 드라이브 제어 (gamepad1 왼쪽 조이스틱 X축) - 일반 드라이브 입력이 없을 때만
			boolean crabDriveInput = controlCrabDrive(leftStickX, hasNormalDriveInput);
			
			// 手動入力で自動整向をキャンセル
			boolean hasManualInput = hasNormalDriveInput || crabDriveInput;
			imuMotorSystem.cancelAutoTurn(hasManualInput);
			
			// 自動整向の制御値を取得
			double[] autoTurnControl = imuMotorSystem.calculateAutoTurnControl();
			double autoTurnLeftPower = autoTurnControl[0];
			double autoTurnRightPower = autoTurnControl[1];
			
			// 크랩 드라이브가 활성화된 경우 크랩 드라이브 제어 사용
			if (crabDriveActive && !hasNormalDriveInput) {
				// 크랩 드라이브는 이미 controlCrabDrive() 메서드에서 모터 제어
				System.out.println("A1Task - Crab Drive: Active - Skipping normal drive control");
			}
			// 自動整向がアクティブな場合は自動整向の制御値を使用、そうでなければ通常のドライブ制御
			else if (imuMotorSystem.isAutoTurnActive()) {
				// 自動整向中は自動整向の制御値をモーターに設定
				if (imuMotorSystem.isLeftMotorConnected()) {
					imuMotorSystem.getLeftMotor().setPower(autoTurnLeftPower);
				}
				if (imuMotorSystem.isRightMotorConnected()) {
					imuMotorSystem.getRightMotor().setPower(autoTurnRightPower);
				}
			} else {
				// A1Taskで直接ドライブ制御を実行
				// 厳密なデッドゾーン設定
				if (Math.abs(rightStickY) < 0.05) rightStickY = 0.0;
				if (Math.abs(rightStickX) < 0.05) rightStickX = 0.0;

				double leftPower = 0.0;
				double rightPower = 0.0;

							// 前後移動制御（右スティックY軸を最優先、gamepad1とgamepad2の両方に対応）
			if (Math.abs(rightStickY) >= 0.05) {
				// gamepad1の右スティックY軸入力がある場合：最優先制御
				System.out.println("A1Task - Gamepad1 right stick Y detected: " + rightStickY);
				leftPower = rightStickY;	// 左モーター（前進で正転）
				rightPower = rightStickY;   // 右モーター（前進で正転）
				
				// 前進時にtop_motorを 속도に 비례하여 정방향으로 구동
				if (rightStickY < 0) {
					double topMotorPower = Math.abs(rightStickY) * 1.2; // 속도의 120%로 top_motor 구동 (50% 더 빠르게)
					System.out.println("A1Task - Gamepad1 Input: " + rightStickY + ", Top motor power: " + topMotorPower);
					
					// top_motor 제어 실행
					if (topSystem.isTopMotorConnected()) {
						topSystem.moveTop(topMotorPower, false);
						System.out.println("A1Task - Forward detected, Top motor " + (topMotorPower * 100) + "% power (speed: " + (Math.abs(rightStickY) * 100) + "%)");
						System.out.println("A1Task - Top motor connected: " + topSystem.isTopMotorConnected() + ", Current power: " + topSystem.getTopMotorPower());
					} else {
						System.out.println("A1Task - ERROR: Top motor not connected!");
					}
				} else {
					// 후진시에는 top_motor 정지
					topSystem.moveTop(0.0, false);
					System.out.println("A1Task - Backward detected, Top motor stopped");
				}
			} else if (Math.abs(gamepad2.right_stick_y) >= 0.05) {
				// gamepad1の右スティックY軸入力がない場合、gamepad2の右スティックY軸で制御
				System.out.println("A1Task - Gamepad2 right stick Y detected: " + gamepad2.right_stick_y);
				double gamepad2RightStickY = gamepad2.right_stick_y;
				leftPower = gamepad2RightStickY;	// 左モーター（前進で正転）
				rightPower = gamepad2RightStickY;   // 右モーター（前進で正転）
				
				// 前進時にtop_motorを 속도에 비례하여 정방향으로 구동
				if (gamepad2RightStickY < 0) {
					double topMotorPower = Math.abs(gamepad2RightStickY) * 1.2; // 속도의 120%로 top_motor 구동 (50% 더 빠르게)
					System.out.println("A1Task - Gamepad2 Input: " + gamepad2RightStickY + ", Top motor power: " + topMotorPower);
					
					// top_motor 제어 실행
					if (topSystem.isTopMotorConnected()) {
						topSystem.moveTop(topMotorPower, false);
						System.out.println("A1Task - Gamepad2 Forward detected, Top motor " + (topMotorPower * 100) + "% power (speed: " + (Math.abs(gamepad2RightStickY) * 100) + "%)");
						System.out.println("A1Task - Top motor connected: " + topSystem.isTopMotorConnected() + ", Current power: " + topSystem.getTopMotorPower());
					} else {
						System.out.println("A1Task - ERROR: Top motor not connected!");
					}
				} else {
					// 후진시에는 top_motor 정지
					topSystem.moveTop(0.0, false);
					System.out.println("A1Task - Gamepad2 Backward detected, Top motor stopped");
				}
			} else {
				// 両方のgamepadの右スティックY軸入力がない 경우：L1ボタン으로 전진
				if (gamepad2.left_bumper && !gamepad2.right_bumper) {
					// L1ボタン이 눌린 경우：전진 제어 (이미 L1 버튼에서 top_motor 제어됨)
					leftPower = -0.3;   // 左モーター（前進で正転）- 30%出力で調整
					rightPower = -0.3;  // 右モーター（前進で正転）- 30%出力で調整
					System.out.println("A1Task - L1 Button Forward Control: -0.3 (30% power)");
				} else {
					// ロボットが停止している時：top_motorを停止
					leftPower = 0.0;
					rightPower = 0.0;
					topSystem.moveTop(0.0, false);
					System.out.println("A1Task - Robot stopped, Top motor stopped");
				}
			}

							// 左右回転制御（右スティックX軸、gamepad1とgamepad2の両方に対応）
			if (Math.abs(rightStickX) >= 0.05) {
				leftPower += -rightStickX;  // 左モーター（右回転で正転）- 符号反転
				rightPower -= -rightStickX; // 右モーター（右回転で逆転）- 符号反転
			} else if (Math.abs(gamepad2.right_stick_x) >= 0.05) {
				// gamepad1の右スティックX軸入力がない場合、gamepad2の右スティックX軸で制御
				double gamepad2RightStickX = gamepad2.right_stick_x;
				leftPower += -gamepad2RightStickX;  // 左モーター（右回転で正転）- 符号反転
				rightPower -= -gamepad2RightStickX; // 右モーター（右回転で逆転）- 符号反転
			}

				// 左右回転制御（L1/R1ボタン）
				if (leftBumper) {
					leftPower += 0.5;   // 左回転（L1）- 左モーター正転
					rightPower -= 0.5;  // 左回転（L1）- 右モーター逆転
				}
				if (rightBumper) {
					leftPower -= 0.5;   // 右回転（R1）- 左モーター逆転
					rightPower += 0.5;  // 右回転（R1）- 右モーター正転
				}
				
				// D-pad 위쪽 버튼으로 천천히 전진
				if (dpadUp) {
					leftPower += -DPAD_UP_FORWARD_POWER;   // 전진 (음수값으로 전진)
					rightPower += -DPAD_UP_FORWARD_POWER;  // 전진 (음수값으로 전진)
					System.out.println("A1Task - D-pad Up: Slow Forward at " + (DPAD_UP_FORWARD_POWER * 100) + "% power");
				}

				// パワー値を-1.0から1.0の範囲に制限
				leftPower = Math.max(-1.0, Math.min(1.0, leftPower));
				rightPower = Math.max(-1.0, Math.min(1.0, rightPower));

							// 安全機能：入力がない場合は確実に0.0に設定（両方のgamepadの入力を考慮）
			boolean hasGamepad1Input = (Math.abs(rightStickY) >= 0.05) || (Math.abs(rightStickX) >= 0.05) || leftBumper || rightBumper || dpadUp;
			boolean hasGamepad2Input = (Math.abs(gamepad2.right_stick_y) >= 0.05) || (Math.abs(gamepad2.right_stick_x) >= 0.05);
			
			if (!hasGamepad1Input && !hasGamepad2Input) {
				leftPower = 0.0;
				rightPower = 0.0;
			}

				// モーターにパワーを設定
				if (imuMotorSystem.isLeftMotorConnected()) {
					imuMotorSystem.getLeftMotor().setPower(leftPower);
				}
				if (imuMotorSystem.isRightMotorConnected()) {
					imuMotorSystem.getRightMotor().setPower(rightPower);
				}
			}
			
			// A1Imuでヘディング記憶機能を更新（P1のSHAREボタン）
			boolean sharePressed = gamepad1.back; // SHARE/BACKボタン
			imuMotorSystem.updateHeadingMemory(sharePressed);
			
			// A1Imuで自動整向機能を更新（P1のOPTIONSボタン）
			boolean optionsPressed = gamepad1.start; // OPTIONS/STARTボタン
			imuMotorSystem.startAutoTurn(optionsPressed);
			
			// R1アーム位置の調整（P1のL1/R1ボタンで調整）
			if (gamepad1.left_bumper && !lastShareButton) {
				// L1ボタンでアーム位置減少
				r1ArmTargetHeight = Math.max(0.0, r1ArmTargetHeight - R1_ARM_HEIGHT_INCREMENT);
				System.out.println("A1Task - R1 Arm Target Height Decreased: " + r1ArmTargetHeight);
			}
			if (gamepad1.right_bumper && !lastOptionsButton) {
				// R1ボタンでアーム位置増加
				r1ArmTargetHeight = Math.min(1.0, r1ArmTargetHeight + R1_ARM_HEIGHT_INCREMENT);
				System.out.println("A1Task - R1 Arm Target Height Increased: " + r1ArmTargetHeight);
			}
			lastShareButton = gamepad1.left_bumper;
			lastOptionsButton = gamepad1.right_bumper;
			
			// アーム自重補正パワーの調整（P1のX/Yボタンで調整）
			if (gamepad1.x && !lastShareButton) {
				// Xボタンで保持パワー減少
				armHoldPower = Math.max(-0.5, armHoldPower - ARM_HOLD_POWER_INCREMENT);
				System.out.println("A1Task - Arm Hold Power Decreased: " + armHoldPower);
			}
			if (gamepad1.y && !lastOptionsButton) {
				// Yボタンで保持パワー増加
				armHoldPower = Math.min(0.0, armHoldPower + ARM_HOLD_POWER_INCREMENT);
				System.out.println("A1Task - Arm Hold Power Increased: " + armHoldPower);
			}
			lastShareButton = gamepad1.x;
			lastOptionsButton = gamepad1.y;
			
			// アーム位置保持 최대出力の調整（P1のX/Yボタンで調整）
			if (gamepad1.x && !lastShareButton) {
				// Xボタンで保持 최대出力 감소
				armHoldMaxPower = Math.max(-0.5, armHoldMaxPower - ARM_HOLD_MAX_POWER_INCREMENT);
				System.out.println("A1Task - Arm Hold Max Power Decreased: " + armHoldMaxPower);
			}
			if (gamepad1.y && !lastOptionsButton) {
				// Yボタンで保持 최대出力 증가
				armHoldMaxPower = Math.min(0.0, armHoldMaxPower + ARM_HOLD_MAX_POWER_INCREMENT);
				System.out.println("A1Task - Arm Hold Max Power Increased: " + armHoldMaxPower);
			}
			

			

			
			// R1ボタンでアーム位置制御 + 리프트 자동 높이 제어 (포텐셔미터 0.57까지)
			if (gamepad2.right_bumper && (Math.abs(rightStickY) < 0.05 || rightStickY <= 0)) {
				// 現在のポテンションメーター値を取得
				double currentArmHeight = potensionSystem.getCurrentHeight();
				
				// 目標位置との差を計算
				double heightError = r1ArmTargetHeight - currentArmHeight;
				
				// 比例制御でアームモーターを制御（ゆっくり）
				double armControlPower = Math.max(-0.3, Math.min(0.3, heightError * 2.0)); // 制限付き比例制御
				
				// ポテンションメーターノーマライズの値に応じてarm_motorの出力を調整（전체적으로 20% 감소）
				double finalArmPower;
				if (currentArmHeight >= 0.45) {
					// ターゲット位置（0.55）に近い場合：50%の出力でしっかり動作（20% 감소 적용）
					finalArmPower = 0.5 * 0.8; // 50%에서 20% 감소 = 40%
					System.out.println("A1Task - Near target position, using 40% power (20% reduced from 50%)");
				} else if (currentArmHeight >= 0.35) {
					// 中間位置の場合：30%の出力（20% 감소 적용）
					finalArmPower = 0.3 * 0.8; // 30%에서 20% 감소 = 24%
					System.out.println("A1Task - Middle position, using 24% power (20% reduced from 30%)");
				} else {
					// 低い位置（0.28など）の場合：比例制御（ほぼ停止可能）（20% 감소 적용）
					finalArmPower = armControlPower * 1.3 * 0.8; // 30%増加 후 20% 감소
					finalArmPower = Math.max(-1.0, Math.min(1.0, finalArmPower)); // 範囲制限
					System.out.println("A1Task - Low position, using proportional control (20% reduced)");
				}
				

				
				// 리프트 자동 높이 제어 (포텐셔미터 0.57까지)
				double targetLiftHeight = 0.57; // 목표 리프트 높이
				double currentLiftHeight = potensionSystem.getCurrentHeight();
				double liftHeightError = targetLiftHeight - currentLiftHeight;
				
				if (currentLiftHeight < targetLiftHeight) {
					// 목표 높이에 도달하지 않은 경우: 최대 파워로 강력하게 상승
					double liftUpPower;
					
					if (currentLiftHeight < 0.5) {
						// 0.5 미만에서는 최대 파워로 강제 상승
						liftUpPower = -1.0; // 100% 파워로 최대 상승
					} else if (currentLiftHeight < 0.55) {
						// 0.5 ~ 0.55에서는 매우 강한 파워로 상승
						liftUpPower = -0.95; // 95% 파워로 강력하게 상승
					} else {
						// 0.55 이상에서는 강한 파워로 정밀하게 상승
						liftUpPower = -0.85; // 85% 파워로 상승
					}
					
					liftSystem.moveLift(liftUpPower);
					System.out.println("A1Task - R1: Lifting to target height " + targetLiftHeight + ", Current: " + currentLiftHeight + ", Power: " + liftUpPower + " (Maximum Power)");
				} else {
					// 목표 높이에 도달한 경우: 자중 지탱 파워로 유지
					liftSystem.moveLift(armHoldPower);
					System.out.println("A1Task - R1: Target height reached, maintaining with hold power: " + armHoldPower);
				}
				
				// 디버그 정보
				System.out.println("A1Task - R1 Button: Arm motor power (+30% increased)");
				System.out.println("A1Task - Current Arm Height: " + currentArmHeight);
				System.out.println("A1Task - Target Arm Height: " + r1ArmTargetHeight);
				System.out.println("A1Task - Current Lift Height: " + currentLiftHeight);
				System.out.println("A1Task - Target Lift Height: " + targetLiftHeight);
				
				telemetry.addData("R1 ARM+LIFT CONTROL", "Arm: Active, Lift: " + (currentLiftHeight < targetLiftHeight ? "Rising" : "Holding") + ", Height: " + currentLiftHeight + "/" + targetLiftHeight);
			}
			
			// R2ボタンでリフト制御（left stick Yの値をシミュレート）
			if (gamepad2.right_bumper) {
				// gamepad2のleft joystick Y入力がある場合はそれを優先
				if (Math.abs(gamepad2.left_stick_y) > 0.1) {
					// left joystick Y入力がある場合：その値で制御
					double leftStickPower = potensionSystem.getAdjustedLiftPower(gamepad2.left_stick_y);
					liftSystem.moveLift(leftStickPower);
					
					// デバッグ情報
					System.out.println("A1Task - R2 Button + Left Stick Y Active");
					System.out.println("A1Task - Left Stick Y Input: " + gamepad2.left_stick_y);
					System.out.println("A1Task - Adjusted Power: " + leftStickPower);
					System.out.println("A1Task - Lift Motor Connected: " + liftSystem.isLiftMotorConnected());
					
					telemetry.addData("R2 LIFT CONTROL", "Left Stick Y Priority: " + gamepad2.left_stick_y + ", Power: " + leftStickPower);
				} else {
					// left joystick Y入力がない場合：R2ボタンの固定値で制御
					// left stick Yにr2LiftValueが入ってきたかのように制御
					// 段階的制限を適用したパワーを取得（r2LiftValueをleft stick Yとして渡す）
					double r2LiftPower = potensionSystem.getAdjustedLiftPower(r2LiftValue);
					
					// トルクを20%上げる（1.2倍）
					double boostedPower = r2LiftPower * 1.2;
					// パワーの範囲を-1.0〜1.0に制限
					boostedPower = Math.max(-1.0, Math.min(1.0, boostedPower));
					
					liftSystem.moveLift(boostedPower);
					
					// デバッグ情報
					System.out.println("A1Task - R2 Button Pressed (Fixed Value)");
					System.out.println("A1Task - Simulated Left Stick Y: " + r2LiftValue);
					System.out.println("A1Task - Original Adjusted Power: " + r2LiftPower);
					System.out.println("A1Task - Boosted Power (20% up): " + boostedPower);
					System.out.println("A1Task - Lift Motor Connected: " + liftSystem.isLiftMotorConnected());
					
					telemetry.addData("R2 LIFT CONTROL", "Fixed Value: " + r2LiftValue + ", Original: " + r2LiftPower + ", Boosted: " + boostedPower);
				}
			}
			
			// ===== 서보모터 제어 정리 =====
			// Triangle 버튼 제어 제거됨 - top_motor와 servo는 독립적으로 제어
			
			// D-pad 제어 (servo1, servo2 동시 제어)
			servoSystem.controlServosWithDpad(gamepad2.dpad_left, gamepad2.dpad_right, gamepad2.dpad_up, gamepad2.dpad_down);
		
			// A/B 버튼 제어
			if (gamepad2.a) {
				servoSystem.moveBothServosToMin(); // A: 최소 위치
				telemetry.addData("SERVO CONTROL", "A Button - Both Servos to MIN");
			}
			if (gamepad2.b) {
				servoSystem.moveBothServosToMax(); // B: 최대 위치
				telemetry.addData("SERVO CONTROL", "B Button - Both Servos to MAX");
			}
		
			// 버튼 매핑 디버깅을 위한 추가 로그
			if (gamepad2.square) {
				System.out.println("A1Task - Square button detected: " + gamepad2.square);
			}
			if (gamepad2.circle) {
				System.out.println("A1Task - Circle button detected: " + gamepad2.circle);
			}
			
			// Share/Options 버튼 제어
			if (gamepad2.back) { // Share 버튼
				servoSystem.setServo1Position(0.25);
				servoSystem.setServo2Position(0.85);
				System.out.println("A1Task - Share Button: Servo1(0.25) + Servo2(0.85)");
				telemetry.addData("SERVO CONTROL", "Share Button - Servo1(0.25) + Servo2(0.85)");
			} else if (gamepad2.start) { // Options 버튼
				servoSystem.setServo1Position(0.7);
				servoSystem.setServo2Position(0.5);
				System.out.println("A1Task - Options Button: Servo1(0.7) + Servo2(0.5)");
				telemetry.addData("SERVO CONTROL", "Options Button - Servo1(0.7) + Servo2(0.5)");
			}
			
			// A1Top2にトップモーター制御を委譲（R2トリガーでM1コンボ機能付き、△ボタンで正回転）
			if (gamepad2.right_trigger > 0.05) {
				// R2トリガーが押されている場合
				if (gamepad2.triangle) {
					// R2トリガー + △ボタンが同時に押された場合：正回転
					topSystem.moveTopWithM1Combo(gamepad2.right_trigger);
					System.out.println("A1Task - R2 + Triangle: Top motor normal rotation with power: " + gamepad2.right_trigger);
				} else {
					// R2トリガーのみ：逆回転
					topSystem.moveTopWithM1ComboReverse(gamepad2.right_trigger);
					System.out.println("A1Task - R2 only: Top motor reverse rotation with power: " + gamepad2.right_trigger);
				}
				
				// R2 트리거로 top_motor 구동 시에는 servo 제어하지 않음
				// top_motor와 servo는 독립적으로 제어되어야 함
				System.out.println("A1debug - R2 trigger: Top Motor control ONLY - NO SERVO CONTROL");
			} else {
				// R2が押されていない場合：top_motor即座に停止
				if (Math.abs(rightStickY) < 0.05 || rightStickY >= 0) {
					topSystem.moveTopWithM1Combo(0.0);
				}
				// servo 제어가 제거되었으므로 타이머 리셋 불필요
			}
			
			// デバッグ情報
			System.out.println("A1Task - Gamepad2 R2 Trigger: " + gamepad2.right_trigger);
			System.out.println("A1Task - Gamepad2 Triangle Button: " + gamepad2.triangle);
			System.out.println("A1Task - Top Motor Power: " + topSystem.getTopMotorPower());
			System.out.println("A1Task - Top Motor Connected: " + topSystem.isTopMotorConnected());
			

			
			// A1ImuでIMUデータを更新
			// imuMotorSystem.updateImuData(); // この行は削除
			

			
			// === ARM MOTOR 제어 로직 ===
			// L2 트리거 제어
			if (gamepad2.left_trigger > 0.05) {
				if (armSystem.isArmMotorConnected()) {
					armSystem.getArmMotor().setPower(1.0); // 100% 출력
					System.out.println("A1Task - L2 Trigger: Arm motor 100% power");
				}
			}
			// R1 버튼 제어 - 30% 출력 증가
			else if (gamepad2.right_bumper && (Math.abs(rightStickY) < 0.05 || rightStickY <= 0)) {
				// 현재의 포텐셔미터 값에 따른 비례 제어
				double currentArmHeight = potensionSystem.getCurrentHeight();
				double heightError = r1ArmTargetHeight - currentArmHeight;
				double armControlPower = Math.max(-0.3, Math.min(0.3, heightError * 2.0));
				
				double finalArmPower;
				if (currentArmHeight >= 0.45) {
					finalArmPower = 0.4 * 1.3; // 40% → 52% (30% 증가)
				} else if (currentArmHeight >= 0.35) {
					finalArmPower = 0.24 * 1.3; // 24% → 31.2% (30% 증가)
				} else {
					finalArmPower = armControlPower * 1.3 * 0.8 * 1.3; // 비례 제어도 30% 증가
				}
				finalArmPower = Math.max(-1.0, Math.min(1.0, finalArmPower));
				
				if (armSystem.isArmMotorConnected()) {
					armSystem.getArmMotor().setPower(finalArmPower);
				}
				System.out.println("A1Task - R1 Button: Arm motor power (+30% increased)");
			}
			// 후진 제어
			else if (rightStickY > 0 || gamepad2.right_stick_y > 0) {
				if (armSystem.isArmMotorConnected()) {
					armSystem.getArmMotor().setPower(0.4); // 40% 출력
				}
				System.out.println("A1Task - Backward: Arm motor 40% power");
			}
			// arm_motor 정지
			else if (armSystem.isArmMotorConnected()) {
				armSystem.getArmMotor().setPower(0.0);
			}
			
			// arm_motor 상태 정보
			System.out.println("A1Task - Arm Motor Connected: " + armSystem.isArmMotorConnected());
			System.out.println("A1Task - Arm Motor Power: " + armSystem.getArmMotorPower());
			
			// A1Liftにリフト制御を委譲（gamepad2のleft stick）
			double currentHeight = potensionSystem.getCurrentHeight();
			
			// 段階的制限を適用したパワーを取得（gamepad2のleft stick Y軸）
			double adjustedLiftPower = potensionSystem.getAdjustedLiftPower(gamepad2.left_stick_y);
			
			// L1前進スピードの調整（P1のL1/R1ボタンで調整）
			if (gamepad1.left_bumper && !lastShareButton) {
				// L1ボタンでスピード減少
				triangleForwardSpeed = Math.max(0.0, triangleForwardSpeed - TRIANGLE_SPEED_INCREMENT);
				System.out.println("A1Task - L1 Forward Speed Decreased: " + triangleForwardSpeed);
			}
			if (gamepad1.right_bumper && !lastOptionsButton) {
				// R1ボタンでスピード増加
				triangleForwardSpeed = Math.min(1.0, triangleForwardSpeed + TRIANGLE_SPEED_INCREMENT);
				System.out.println("A1Task - L1 Forward Speed Increased: " + triangleForwardSpeed);
			}
			lastShareButton = gamepad1.left_bumper;
			lastOptionsButton = gamepad1.right_bumper;
			
			// gamepad2のL1ボタン制御（lift_motor + top_motor + 아주 약한 전진）
			if (gamepad2.left_bumper && !gamepad2.right_bumper) {
				// L1ボタン이 눌린 경우：lift_motor 제어（gamepad1 joystick과 무관）
				double l1LiftPower = potensionSystem.getAdjustedLiftPower(-0.800);
				liftSystem.moveLift(l1LiftPower);
				
				// L1 버튼으로 top_motor 구동 시에는 servo 제어하지 않음
				// top_motor와 servo는 독립적으로 제어되어야 함
				System.out.println("A1debug - L1 button pressed: Lift + Top Motor + Forward ONLY - NO SERVO CONTROL");
				
				// top_motorを動かす（M1コンボ機能）- gamepad1 joystick과 무관
				
				// top_motorを動かす（M1コンボ機能）- gamepad1 joystick과 무관
				topSystem.moveTopM1Combo();
				
				// L1 버튼으로 약한 전진 추가 (30% 파워)
				double l1ForwardPower = 0.3; // 약한 전진 파워
				if (imuMotorSystem.isLeftMotorConnected()) {
					imuMotorSystem.getLeftMotor().setPower(-l1ForwardPower); // 전진 (음수)
				}
				if (imuMotorSystem.isRightMotorConnected()) {
					imuMotorSystem.getRightMotor().setPower(-l1ForwardPower); // 전진 (음수)
				}
				
				// デバッグ情報
				System.out.println("A1Task - L1 Button Pressed (Lift+Top+Servo+Forward Control)");
				System.out.println("A1Task - Top Motor Power: " + topSystem.getTopMotorPower());
				System.out.println("A1Task - Top Motor Connected: " + topSystem.isTopMotorConnected());
				System.out.println("A1Task - L1 Top Motor Power: -1.0");
				System.out.println("A1Task - L1 Forward Power: " + l1ForwardPower + " (30%)");
				
				telemetry.addData("LIFT+TOP+FORWARD COMBO", "L1 Button - Lift: -0.800 (Adjusted: " + l1LiftPower + "), Top: -1.0, Forward: " + l1ForwardPower);
			}
			// gamepad2のSquareボタン制御（top_motor만 제어）
			else if (gamepad2.square) {
				// Squareボタン이 눌린 경우：top_motor만 제어
				topSystem.moveTopM1Combo(); // 100% 정방향
				
				// デバッグ情報
				System.out.println("A1Task - Square Button Pressed (Top Motor Control)");
				System.out.println("A1Task - Top Motor Power: " + topSystem.getTopMotorPower());
				System.out.println("A1Task - Top Motor Connected: " + topSystem.isTopMotorConnected());
				System.out.println("A1Task - Square Top Motor Power: 1.0");
				
				telemetry.addData("TOP MOTOR CONTROL", "Square Button - Top: 1.0 (100%)");
			}
			// gamepad2のCircleボタン制御（top_motor만 제어 - 역방향, 서보 제어 완전 차단）
			else if (gamepad2.circle) {
				// Circleボタン이 눌린 경우：top_motor만 제어 (역방향)
				topSystem.moveTopM1ComboReverse(); // 100% 역방향
				
				// Circle 버튼이 눌렸을 때 서보 제어 완전 차단
				// 어떤 다른 로직에서도 서보를 제어하지 않도록 명시적으로 차단
				System.out.println("A1debug - Circle Button: BLOCKING ALL SERVO CONTROL");
				
				// デバッ그情報
				System.out.println("=== CIRCLE BUTTON DEBUG ===");
				System.out.println("A1debug - Circle Button Pressed - Starting Top Motor Reverse Control ONLY");
				System.out.println("A1debug - Before moveTopM1ComboReverse() - Top Motor Power: " + topSystem.getTopMotorPower());
				
				topSystem.moveTopM1ComboReverse(); // 100% 역방향
				
				System.out.println("A1debug - After moveTopM1ComboReverse() - Top Motor Power: " + topSystem.getTopMotorPower());
				System.out.println("A1debug - Top Motor Connected: " + topSystem.isTopMotorConnected());
				System.out.println("A1debug - Circle Top Motor Power: -1.0");
				System.out.println("A1debug - Circle Button: NO SERVO CONTROL - TOP MOTOR ONLY");
				System.out.println("=== END CIRCLE BUTTON DEBUG ===");
				
				telemetry.addData("TOP MOTOR CONTROL", "Circle Button - Top: -1.0 (100% Reverse) - NO SERVO");
			} else if (!gamepad2.right_bumper) {
				// L1ボタンが押されていない場合：通常の制御（段階的制限を適用）
				// R2ボタンが押されている場合は通常制御をスキップ
				
				// gamepad2의 left joystick Y 입력이 있는 경우：일반 제어
				if (Math.abs(gamepad2.left_stick_y) > 0.1) {
					liftSystem.moveLift(adjustedLiftPower);
					System.out.println("A1Task - Normal Lift Control: " + adjustedLiftPower);
				} else {
					// gamepad2의 left joystick Y 입력이 없는 경우：암 위치 유지 제어
					double holdPower = calculateArmHoldPower(currentHeight);
					liftSystem.moveLift(holdPower);
					System.out.println("A1Task - Arm Position Hold Control: Height=" + currentHeight + ", HoldPower=" + holdPower);
				}
			}
			

			
			// Flask機能は無効化（ロボットとFlaskを分離）

			// テレメトリ表示
			telemetry.addData("=== GAMEPAD INPUT VALUES ===", "");
			telemetry.addData("Left Stick X", "%.3f", leftStickX);
			telemetry.addData("Left Stick Y", "%.3f", leftStickY);
			telemetry.addData("Right Stick X", "%.3f", rightStickX);
			telemetry.addData("Right Stick Y", "%.3f", rightStickY);
			
			telemetry.addData("Left Trigger", "%.3f", leftTrigger);
			telemetry.addData("Right Trigger", "%.3f", rightTrigger);
			
			telemetry.addData("Left Bumper", leftBumper ? "PRESSED" : "Released");
			telemetry.addData("Right Bumper", rightBumper ? "PRESSED" : "Released");
			
			// 크랩 드라이브 상태 표시
			telemetry.addData("=== CRAB DRIVE STATUS ===", "");
			telemetry.addData("Crab Drive Active", crabDriveActive ? "YES" : "NO");
			telemetry.addData("Normal Drive Input", hasNormalDriveInput ? "YES (Priority)" : "NO");
			telemetry.addData("Left Stick X", "%.3f", leftStickX);
			if (crabDriveActive) {
				telemetry.addData("Target Heading", "%.2f°", crabDriveTargetHeading);
				telemetry.addData("Current Heading", "%.2f°", imuMotorSystem.getYaw());
				telemetry.addData("Heading Error", "%.2f°", crabDriveTargetHeading - imuMotorSystem.getYaw());
				long elapsedTime = System.currentTimeMillis() - crabDriveStartTime;
				telemetry.addData("Elapsed Time", "%d ms", elapsedTime);
				if (elapsedTime < CRAB_DRIVE_DURATION) {
					telemetry.addData("Phase", "1 - BACKWARD FIRST (800ms)");
					telemetry.addData("Backward Power", "%.1f%%", CRAB_DRIVE_BACKWARD_POWER * 100);
					if (leftStickX > 0) {
						telemetry.addData("Backward Direction", "LEFT backward (오른쪽 이동을 위해)");
					} else {
						telemetry.addData("Backward Direction", "RIGHT backward (왼쪽 이동을 위해)");
					}
				} else if (elapsedTime < CRAB_DRIVE_DURATION + 500) {
					telemetry.addData("Phase", "2 - FORWARD SECOND (500ms)");
					double forwardPower = Math.abs(leftStickX) * CRAB_DRIVE_FORWARD_POWER;
					telemetry.addData("Forward Power", "%.1f%%", forwardPower * 100);
					telemetry.addData("Forward Purpose", "Return to original position with heading correction");
					
					// Heading 보정 상태 상세 표시
					double currentHeading = imuMotorSystem.getYaw();
					double headingError = crabDriveTargetHeading - currentHeading;
					while (headingError > 180) headingError -= 360;
					while (headingError < -180) headingError += 360;
					
					// 오버슈팅 방지 정보 추가
					double headingStopThreshold = 2.0 * CRAB_DRIVE_HEADING_STOP_RATIO; // 1.4도
					telemetry.addData("Overshoot Prevention", "%.1f%% (%.2f°)", CRAB_DRIVE_HEADING_STOP_RATIO * 100, headingStopThreshold);
					
					telemetry.addData("Heading Correction", "%.2f° → %.2f° (Error: %.2f°)", currentHeading, crabDriveTargetHeading, headingError);
					if (Math.abs(headingError) < headingStopThreshold) {
						telemetry.addData("Heading Status", "TARGET REACHED (Overshoot Prevention) ✓");
					} else if (Math.abs(headingError) < 5.0) {
						telemetry.addData("Heading Status", "APPROACHING TARGET");
					} else {
						telemetry.addData("Heading Status", "CORRECTING...");
					}
				} else {
					telemetry.addData("Phase", "Completed");
				}
				if (leftStickX > 0) {
					telemetry.addData("Direction", "RIGHT (오른쪽으로 이동)");
				} else {
					telemetry.addData("Direction", "LEFT (왼쪽으로 이동)");
				}
			}
			
			telemetry.addData("A Button", aButton ? "PRESSED" : "Released");
			telemetry.addData("B Button", bButton ? "PRESSED" : "Released");
			telemetry.addData("X Button", xButton ? "PRESSED" : "Released");
			telemetry.addData("Y Button", yButton ? "PRESSED" : "Released");
			
			telemetry.addData("D-pad Up", dpadUp ? "PRESSED" : "Released");
			telemetry.addData("D-pad Down", dpadDown ? "PRESSED" : "Released");
			telemetry.addData("D-pad Left", dpadLeft ? "PRESSED" : "Released");
			telemetry.addData("D-pad Right", dpadRight ? "PRESSED" : "Released");
			
			// Gamepad2の入力値も表示
			telemetry.addData("=== GAMEPAD2 INPUT VALUES ===", "");
			telemetry.addData("Gamepad2 Left Stick X", "%.3f", gamepad2.left_stick_x);
			telemetry.addData("Gamepad2 Left Stick Y", "%.3f", gamepad2.left_stick_y);
			telemetry.addData("Gamepad2 Right Stick X", "%.3f", gamepad2.right_stick_x);
			telemetry.addData("Gamepad2 Right Stick Y", "%.3f", gamepad2.right_stick_y);
			telemetry.addData("Gamepad2 Left Trigger", "%.3f", gamepad2.left_trigger);
			telemetry.addData("Gamepad2 Right Trigger", "%.3f", gamepad2.right_trigger);
			telemetry.addData("Gamepad2 Left Bumper", gamepad2.left_bumper ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 Right Bumper", gamepad2.right_bumper ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 A Button", gamepad2.a ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 B Button", gamepad2.b ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 X Button", gamepad2.x ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 Y Button", gamepad2.y ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 Square Button", gamepad2.square ? "PRESSED - Arm Motor Control" : "Released");
			telemetry.addData("Gamepad2 Circle Button", gamepad2.circle ? "PRESSED - Top Motor Reverse Control" : "Released");
			telemetry.addData("Gamepad2 Triangle Button", gamepad2.triangle ? "PRESSED - Top Motor Control" : "Released");
			telemetry.addData("Gamepad2 D-pad Up", gamepad2.dpad_up ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 D-pad Down", gamepad2.dpad_down ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 D-pad Left", gamepad2.dpad_left ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 D-pad Right", gamepad2.dpad_right ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 Share (Back)", gamepad2.back ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 Options (Start)", gamepad2.start ? "PRESSED" : "Released");
			
			telemetry.addData("=== DRIVE CONTROL ===", "");
			telemetry.addData("Right Stick Y", "%.3f", rightStickY);
			telemetry.addData("Right Stick X", "%.3f", rightStickX);
			telemetry.addData("Left Bumper (L1)", leftBumper ? "PRESSED" : "Released");
			telemetry.addData("Right Bumper (R1)", rightBumper ? "PRESSED" : "Released");
			telemetry.addData("Left Motor Power", "%.3f", imuMotorSystem.getLeftMotorPower());
			telemetry.addData("Right Motor Power", "%.3f", imuMotorSystem.getRightMotorPower());
			telemetry.addData("Left Motor Connected", imuMotorSystem.isLeftMotorConnected());
			telemetry.addData("Right Motor Connected", imuMotorSystem.isRightMotorConnected());
			
			// 入力がない場合の確認
			boolean noInput = (Math.abs(rightStickY) < 0.05) && (Math.abs(rightStickX) < 0.05) && !leftBumper && !rightBumper;
			telemetry.addData("No Input Detected", noInput ? "YES - Motors should be stopped" : "NO - Input detected");
			
			telemetry.addData("=== ARM CONTROL ===", "");
			telemetry.addData("Arm Motor Power", "%.3f", armSystem.getArmMotorPower());
			telemetry.addData("Arm Square Button (Gamepad2)", gamepad2.square ? "PRESSED - Fixed Power 0.442 (Simple5MotorTest Style)" : "Released");
			telemetry.addData("R1 Arm Control Active", gamepad2.right_bumper ? "YES - Auto Position Control" : "NO");
			telemetry.addData("R1 Arm Target Height", "%.2f (P1 L1/R1 to adjust)", r1ArmTargetHeight);
			telemetry.addData("R2 Button Lift Control", gamepad2.right_bumper ? "YES - Simulated Left Stick Y: " + r2LiftValue : "NO");
			telemetry.addData("Arm Hold Power", "%.2f (P1 X/Y to adjust)", armHoldPower);
			telemetry.addData("Arm Position Hold Max Power", "%.2f (P1 X/Y to adjust)", armHoldMaxPower);
			telemetry.addData("Arm Position Hold Active", currentHeight >= ARM_HOLD_MIN_HEIGHT ? "YES" : "NO");
			telemetry.addData("Arm Position Hold Height", "%.3f (Min: %.2f, Max: %.2f)", currentHeight, ARM_HOLD_MIN_HEIGHT, ARM_HOLD_MAX_HEIGHT);
			telemetry.addData("Arm Motor Connected", armSystem.isArmMotorConnected());
			
			telemetry.addData("=== TOP MOTOR CONTROL ===", "");
			telemetry.addData("Top Motor Power", "%.3f", topSystem.getTopMotorPower());
			telemetry.addData("Top Command (Gamepad2 R2)", "%.3f", gamepad2.right_trigger);
			telemetry.addData("Top M1 Combo Active", gamepad2.right_trigger > 0.1 ? "YES" : "NO");
			telemetry.addData("Top Current Power", "%.3f", topSystem.getCurrentTopPower());
			telemetry.addData("Top Target Power", "%.3f", topSystem.getTargetTopPower());
			telemetry.addData("Top Motor Connected", topSystem.isTopMotorConnected());
			telemetry.addData("Triangle Button (Gamepad2)", gamepad2.triangle ? "PRESSED - Reverse Rotation" : "Released");
			telemetry.addData("Triangle Top Motor Power", "%.3f", gamepad2.triangle ? -1.0 : 0.0);
			telemetry.addData("Gamepad2 R2 Active", gamepad2.right_trigger > 0.05 ? "YES" : "NO");
			telemetry.addData("R2 + Triangle Combo", (gamepad2.right_trigger > 0.05 && gamepad2.triangle) ? "ACTIVE - Reverse" : "Inactive");
			telemetry.addData("Top Rotation Direction", (gamepad2.right_trigger > 0.05 && gamepad2.triangle) ? "REVERSE" : (gamepad2.right_trigger > 0.05 ? "NORMAL" : "STOPPED"));
			
			// IMUデータ表示
			telemetry.addData("=== IMU DATA ===", "");
			if (imuMotorSystem.isInitialized()) {
				telemetry.addData("IMU Status", "Connected");
				telemetry.addData("Yaw", "%.2f°", imuMotorSystem.getYaw());
				telemetry.addData("Pitch", "%.2f°", imuMotorSystem.getPitch());
				telemetry.addData("Roll", "%.2f°", imuMotorSystem.getRoll());
				telemetry.addData("Heading", "%.2f°", imuMotorSystem.getHeading());
				telemetry.addData("Angular Velocity Z", "%.2f°/s", imuMotorSystem.getAngularVelocityZ());
				
				// ヘディング記憶情報を表示
				if (imuMotorSystem.hasStoredHeading()) {
					telemetry.addData("Stored Heading", "%.2f° (%s)", imuMotorSystem.getStoredHeading(), 
						imuMotorSystem.getHeadingDirection(imuMotorSystem.getStoredHeading()));
				} else {
					telemetry.addData("Stored Heading", "NONE - Press SHARE/BACK to store");
				}
				
				// 現在のヘディング方向を表示
				telemetry.addData("Current Direction", "%s (%.2f°)", 
					imuMotorSystem.getHeadingDirection(imuMotorSystem.getYaw()), imuMotorSystem.getYaw());
				
				// 自動整向の状態を表示
				telemetry.addData("Auto Turn Active", imuMotorSystem.isAutoTurnActive() ? "YES" : "NO");
				telemetry.addData("Heading Locked", imuMotorSystem.isHeadingLocked() ? "YES" : "NO");
				if (imuMotorSystem.isAutoTurnActive()) {
					double[] autoTurnControlTelemetry = imuMotorSystem.calculateAutoTurnControl();
					telemetry.addData("Auto Turn Left Power", "%.3f", autoTurnControlTelemetry[0]);
					telemetry.addData("Auto Turn Right Power", "%.3f", autoTurnControlTelemetry[1]);
				}
			} else {
				telemetry.addData("IMU Status", "Not Connected");
				if (imuMotorSystem.hasError()) {
					telemetry.addData("IMU Error", imuMotorSystem.getLastError());
				}
			}
			
			telemetry.addData("=== LIFT CONTROL ===", "");
			telemetry.addData("Lift Motor Power", "%.3f", liftSystem.getLiftMotorPower());
			telemetry.addData("Lift Command (Gamepad2 Left Stick Y)", "%.3f", gamepad2.left_stick_y);
			telemetry.addData("Lift Motor Connected", liftSystem.isLiftMotorConnected());
			telemetry.addData("L1 Combo Active", gamepad2.left_bumper ? "YES - Lift+Top+Forward Simultaneous" : "NO - Normal Control");
			telemetry.addData("L1 Forward Speed", "%.2f (P1 L1/R1 to adjust)", triangleForwardSpeed);
			
			telemetry.addData("=== POTENTIOMETER ===", "");
			telemetry.addData("Potentiometer Voltage", "%.3f V", potensionSystem.getPotentiometerVoltage());
			telemetry.addData("Potentiometer Normalized", "%.3f", currentHeight);
			telemetry.addData("Current Height", "%.3f", currentHeight);
			telemetry.addData("Height Limit (Upper)", "%.3f", potensionSystem.getMaxHeight());
			telemetry.addData("Reduction Factor", "%.3f", potensionSystem.getReductionFactor());
			telemetry.addData("Adjusted Lift Power", "%.3f", adjustedLiftPower);
			telemetry.addData("Original Lift Power (Gamepad2 Left Stick Y)", "%.3f", gamepad2.left_stick_y);
			telemetry.addData("Auto Height Control", potensionSystem.isAutoHeightControlEnabled() ? "ON" : "OFF");
			telemetry.addData("Target Height", "%.3f", potensionSystem.getTargetHeight());
			telemetry.addData("Height Error", "%.3f", potensionSystem.getHeightError());
			telemetry.addData("Potentiometer Connected", potensionSystem.isPotentiometerConnected());
			
			telemetry.addData("=== SERVO CONTROL ===", "");
			telemetry.addData("Servo1 Position", "%.3f", servoSystem.getServo1Position());
			telemetry.addData("Servo2 Position", "%.3f", servoSystem.getServo2Position());
			telemetry.addData("Servo1 Connected", servoSystem.isServo1Connected());
			telemetry.addData("Servo2 Connected", servoSystem.isServo2Connected());
			telemetry.addData("Servo Control (Gamepad2)", "D-pad: Left/Right=Servo1, Up/Down=Servo2");
			telemetry.addData("Servo Button Control (Gamepad2)", "A=Servo1(0.250)+Servo2(0.500), B=Servo1(0.700)+Servo2(0.900)");
			telemetry.addData("Servo Triangle Control (Gamepad2)", "Triangle=Servo1(0.700)+Servo2(0.500)");
			
			// Flaskステータス表示（Flaskが利用可能な場合のみ）
			if (flaskSystem != null) {
				telemetry.addData("=== FLASK STATUS ===", "");
				telemetry.addData("Send Count", flaskSystem.getSendCount());
				telemetry.addData("Success Count", flaskSystem.getSuccessCount());
				telemetry.addData("Error Count", flaskSystem.getErrorCount());
				telemetry.addData("Success Rate", "%.1f%%", flaskSystem.getSuccessRate());
				telemetry.addData("Last Error", flaskSystem.getLastError());
				telemetry.addData("Server URL", flaskSystem.getServerURL());
			} else {
				telemetry.addData("=== FLASK STATUS ===", "DISABLED - Flask system not available");
			}
			
			telemetry.addData("=== SYSTEM INFO ===", "");
			telemetry.addData("Update Rate", "20 Hz (50ms)");
			telemetry.addData("Status", "Running");
			
			// Servo1 위치 변경 추적
			double currentServo1Position = servoSystem.getServo1Position();
			if (Math.abs(currentServo1Position - lastServo1Position) > 0.01) {
				telemetry.addData("*** SERVO1 POSITION CHANGE ***", "%.3f -> %.3f", lastServo1Position, currentServo1Position);
				telemetry.addData("*** CHANGE DETECTED ***", "Check Logcat for details");
				lastServo1Position = currentServo1Position;
			} else {
				telemetry.addData("Servo1 Position Stable", "%.3f", currentServo1Position);
			}
			
			// M2 버튼 상태 표시
			telemetry.addData("M2 Button Status", gamepad2.square ? "PRESSED" : "Released");
			telemetry.addData("Circle Button Status", gamepad2.circle ? "PRESSED - Top Motor Reverse" : "Released");
			
			// 현재 servo 제어 상태 표시
			if (gamepad2.square) {
				telemetry.addData("*** SQUARE ACTIVE ***", "Top Motor Forward");
			} else if (gamepad2.circle) {
				telemetry.addData("*** CIRCLE ACTIVE ***", "Top Motor Reverse");
			} else {
				telemetry.addData("*** NO SQUARE/CIRCLE ***", "Other controls active");
			}
			
			// === SERVO1 POSITION TRACKING ===
			telemetry.addData("=== SERVO1 POSITION TRACKING ===", "");
			
			// 현재 servo1 위치
			telemetry.addData("Current Servo1 Position", "%.3f", currentServo1Position);
			telemetry.addData("Last Servo1 Position", "%.3f", lastServo1Position);
			
			// 위치 변경 감지
			if (Math.abs(currentServo1Position - lastServo1Position) > 0.01) {
				telemetry.addData("*** POSITION CHANGED ***", "YES");
				telemetry.addData("Change Amount", "%.3f", currentServo1Position - lastServo1Position);
			} else {
				telemetry.addData("*** POSITION CHANGED ***", "NO");
			}
			
			// 버튼 상태 상세 정보
			telemetry.addData("Gamepad2 Square (Top Forward)", gamepad2.square ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 Circle (Top Reverse)", gamepad2.circle ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 A", gamepad2.a ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 B", gamepad2.b ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 D-pad Left", gamepad2.dpad_left ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 D-pad Right", gamepad2.dpad_right ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 D-pad Up", gamepad2.dpad_up ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 D-pad Down", gamepad2.dpad_down ? "PRESSED" : "Released");
			telemetry.addData("Gamepad2 R2 Trigger", "%.3f", gamepad2.right_trigger);
			telemetry.addData("Gamepad2 L1 Bumper", gamepad2.left_bumper ? "PRESSED" : "Released");
			
			// 버튼 상태 디버깅
			telemetry.addData("=== BUTTON DEBUG ===", "");
			telemetry.addData("Square (Top Forward) Raw Value", gamepad2.square);
			telemetry.addData("Circle (Top Reverse) Raw Value", gamepad2.circle);
			telemetry.addData("Triangle Raw Value", gamepad2.triangle);
			telemetry.addData("Cross (X) Raw Value", gamepad2.x);
			telemetry.addData("A Button Raw Value", gamepad2.a);
			telemetry.addData("B Button Raw Value", gamepad2.b);
			telemetry.addData("Both Square and Circle Pressed", (gamepad2.square && gamepad2.circle) ? "YES" : "NO");
			telemetry.addData("Neither Square nor Circle Pressed", (!gamepad2.square && !gamepad2.circle) ? "YES" : "NO");
			
			// 버튼 매핑 확인
			telemetry.addData("=== BUTTON MAPPING CHECK ===", "");
			if (gamepad2.square) telemetry.addData("Square (Top Forward) Detected", "YES");
			if (gamepad2.circle) telemetry.addData("Circle (Top Reverse) Detected", "YES");
			if (gamepad2.triangle) telemetry.addData("Triangle Detected", "YES");
			if (gamepad2.x) telemetry.addData("Cross (X) Detected", "YES");
			if (gamepad2.a) telemetry.addData("A Button Detected", "YES");
			if (gamepad2.b) telemetry.addData("B Button Detected", "YES");
			
			// 제어 우선순위 상태
			if (gamepad2.square) {
				telemetry.addData("*** SQUARE PRIORITY ***", "ACTIVE - Top Motor Forward");
				telemetry.addData("Square Override Status", "Top motor forward 100% power");
			} else if (gamepad2.circle) {
				telemetry.addData("*** CIRCLE PRIORITY ***", "ACTIVE - Top Motor Reverse");
				telemetry.addData("Circle Override Status", "Top motor reverse 100% power");
			} else {
				telemetry.addData("*** NORMAL CONTROL ***", "ACTIVE");
				telemetry.addData("D-pad Control", "Enabled");
				telemetry.addData("A/B Button Control", "Enabled");
				telemetry.addData("R2 Servo Control", "DISABLED - Top Motor Only");
				telemetry.addData("L1 Servo Control", "Enabled");
			}
			
			// 예상 servo1 위치 (수정된 버전)
			if (gamepad2.back) {
				telemetry.addData("Expected Servo1 Position", "0.250 (Share Button)");
			} else if (gamepad2.start) {
				telemetry.addData("Expected Servo1 Position", "0.700 (Options Button)");
			} else if (gamepad2.a) {
				telemetry.addData("Expected Servo1 Position", "0.250 (A Button)");
			} else if (gamepad2.b) {
				telemetry.addData("Expected Servo1 Position", "0.700 (B Button)");
			} else if (gamepad2.dpad_left) {
				telemetry.addData("Expected Servo1 Position", "Decreasing (D-pad Left)");
			} else if (gamepad2.dpad_right) {
				telemetry.addData("Expected Servo1 Position", "Increasing (D-pad Right)");
			} else if (gamepad2.right_trigger > 0.05) {
				telemetry.addData("Expected Servo1 Position", "0.700 (R2 Trigger - 3s delay)");
			} else if (gamepad2.left_bumper) {
				telemetry.addData("Expected Servo1 Position", "0.700 (L1 Button)");
			} else {
				telemetry.addData("Expected Servo1 Position", "No change expected");
			}
			
			// 위치 불일치 감지 (Circle 버튼은 이제 servo 제어 안함)
			telemetry.addData("Position Match", "OK - Circle button no longer controls servos");
			
			// === CIRCLE BUTTON SERVO DEBUG ===
			telemetry.addData("=== CIRCLE BUTTON SERVO DEBUG ===", "");
			if (gamepad2.circle) {
				telemetry.addData("*** CIRCLE BUTTON ACTIVE ***", "Top Motor Reverse Control ONLY");
				telemetry.addData("Circle Button Status", "PRESSED - ALL SERVO CONTROL BLOCKED");
				telemetry.addData("Servo1 Position During Circle", "%.3f (BLOCKED - NO CHANGE)", currentServo1Position);
				telemetry.addData("Servo2 Position During Circle", "%.3f (BLOCKED - NO CHANGE)", servoSystem.getServo2Position());
				telemetry.addData("Top Motor Power During Circle", "%.3f (Should be -1.0)", topSystem.getTopMotorPower());
				telemetry.addData("Circle Servo Block Status", "ACTIVE - L1 servo control BLOCKED (R2 servo control removed)");
			} else {
				telemetry.addData("Circle Button Status", "Released - Servo control allowed");
				telemetry.addData("Servo1 Position", "%.3f", currentServo1Position);
				telemetry.addData("Servo2 Position", "%.3f", servoSystem.getServo2Position());
			}
			
			telemetry.update();
			
			// 50ms待機
			sleep(50);
		}
	}
} 