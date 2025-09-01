package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name="SimpleServoTest", group="Test")
public class simpleServoTest extends LinearOpMode {

	// 서보모터 객체
	private Servo servo1;
	private Servo servo2;
	
	// 서보 각도 변수
	private double currentPosition = 0.5; // 중간 위치 (0.0 ~ 1.0)
	private double currentPosition2 = 0.5; // servo2 중간 위치 (0.0 ~ 1.0)
	private static final double POSITION_INCREMENT = 0.05; // 5%씩 이동
	private static final double MIN_POSITION = 0.0; // 최소 위치
	private static final double MAX_POSITION = 1.0; // 최대 위치

	@Override
	public void runOpMode() {
		// 서보모터 초기화
		servo1 = hardwareMap.get(Servo.class, "servo1");
		servo2 = hardwareMap.get(Servo.class, "servo2");
		
		if (servo1 != null) {
			// 서보를 중간 위치로 설정
			servo1.setPosition(currentPosition);
			telemetry.addData("Servo1 Status", "Initialized Successfully");
		} else {
			telemetry.addData("Servo1 Status", "Initialization Failed");
		}
		
		if (servo2 != null) {
			// 서보2를 중간 위치로 설정
			servo2.setPosition(currentPosition2);
			telemetry.addData("Servo2 Status", "Initialized Successfully");
		} else {
			telemetry.addData("Servo2 Status", "Initialization Failed");
		}
		
		telemetry.addData("Status", "Ready for START");
		telemetry.addData("Servo1 Controls", "Left/Right Arrow Keys");
		telemetry.addData("Servo2 Controls", "Up/Down Arrow Keys");
		telemetry.addData("Position Range", "0.0 ~ 1.0");
		telemetry.update();
		
		// 시작 대기
		waitForStart();
		
		while (opModeIsActive()) {
			// 화살표 키 입력 확인
			boolean leftArrow = gamepad1.dpad_left;
			boolean rightArrow = gamepad1.dpad_right;
			boolean upArrow = gamepad1.dpad_up;
			boolean downArrow = gamepad1.dpad_down;
			
			// servo1 위치 조정 (좌우 화살표)
			if (leftArrow && currentPosition > MIN_POSITION) {
				// 왼쪽 화살표: 서보를 왼쪽으로 이동
				currentPosition = Math.max(MIN_POSITION, currentPosition - POSITION_INCREMENT);
				if (servo1 != null) {
					servo1.setPosition(currentPosition);
				}
			} else if (rightArrow && currentPosition < MAX_POSITION) {
				// 오른쪽 화살표: 서보를 오른쪽으로 이동
				currentPosition = Math.min(MAX_POSITION, currentPosition + POSITION_INCREMENT);
				if (servo1 != null) {
					servo1.setPosition(currentPosition);
				}
			}
			
			// servo2 위치 조정 (상하 화살표)
			if (upArrow && currentPosition2 < MAX_POSITION) {
				// 위쪽 화살표: 서보를 위쪽으로 이동
				currentPosition2 = Math.min(MAX_POSITION, currentPosition2 + POSITION_INCREMENT);
				if (servo2 != null) {
					servo2.setPosition(currentPosition2);
				}
			} else if (downArrow && currentPosition2 > MIN_POSITION) {
				// 아래쪽 화살표: 서보를 아래쪽으로 이동
				currentPosition2 = Math.max(MIN_POSITION, currentPosition2 - POSITION_INCREMENT);
				if (servo2 != null) {
					servo2.setPosition(currentPosition2);
				}
			}
			
			// Telemetry 표시
			telemetry.addData("=== SERVO1 CONTROL ===", "");
			telemetry.addData("Servo1 Name", "servo1");
			telemetry.addData("Servo1 Position", "%.3f", currentPosition);
			telemetry.addData("Servo1 Position %", "%.1f%%", currentPosition * 100);
			
			telemetry.addData("=== SERVO2 CONTROL ===", "");
			telemetry.addData("Servo2 Name", "servo2");
			telemetry.addData("Servo2 Position", "%.3f", currentPosition2);
			telemetry.addData("Servo2 Position %", "%.1f%%", currentPosition2 * 100);
			
			telemetry.addData("=== POSITION RANGE ===", "");
			telemetry.addData("Min Position", "%.1f", MIN_POSITION);
			telemetry.addData("Max Position", "%.1f", MAX_POSITION);
			telemetry.addData("Increment", "%.2f", POSITION_INCREMENT);
			
			telemetry.addData("=== CONTROLS ===", "");
			telemetry.addData("Left Arrow (Servo1)", leftArrow ? "PRESSED" : "Released");
			telemetry.addData("Right Arrow (Servo1)", rightArrow ? "PRESSED" : "Released");
			telemetry.addData("Up Arrow (Servo2)", upArrow ? "PRESSED" : "Released");
			telemetry.addData("Down Arrow (Servo2)", downArrow ? "PRESSED" : "Released");
			telemetry.addData("Instructions", "D-pad Left/Right for Servo1, Up/Down for Servo2");
			
			telemetry.addData("=== STATUS ===", "");
			if (servo1 != null) {
				telemetry.addData("Servo1 Connected", "YES");
				telemetry.addData("Servo1 Actual Position", "%.3f", servo1.getPosition());
			} else {
				telemetry.addData("Servo1 Connected", "NO");
				telemetry.addData("Servo1 Error", "Check Robot Configuration");
			}
			
			if (servo2 != null) {
				telemetry.addData("Servo2 Connected", "YES");
				telemetry.addData("Servo2 Actual Position", "%.3f", servo2.getPosition());
			} else {
				telemetry.addData("Servo2 Connected", "NO");
				telemetry.addData("Servo2 Error", "Check Robot Configuration");
			}
			
			telemetry.update();
			sleep(50); // 20Hz 업데이트
		}
	}
} 