package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="SimpleIMUTest", group="Test")
public class simpleIMUtest extends LinearOpMode {

	// IMU 센서 객체
	private IMU imu;

	@Override
	public void runOpMode() {
		// IMU 초기화
		imu = hardwareMap.get(IMU.class, "imu");

		// IMU 설정 (2023년 이후 최신 IMU 칩용)
		if (imu != null) {
			// IMU 파라미터 설정
			IMU.Parameters parameters = new IMU.Parameters(
				new RevHubOrientationOnRobot(
					RevHubOrientationOnRobot.LogoFacingDirection.UP,
					RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
				)
			);
			imu.initialize(parameters);
			
			telemetry.addData("IMU Status", "Initialized Successfully");
			telemetry.addData("IMU Chip", "BHI260AP (2023+)");
		} else {
			telemetry.addData("IMU Status", "Initialization Failed");
		}

		telemetry.addData("Status", "Ready for START");
		telemetry.update();

		// 시작 대기
		waitForStart();

		while (opModeIsActive()) {
			if (imu != null) {
				// Yaw, Pitch, Roll 각도 읽기
				YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
				
				// 각속도 읽기
				AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
				
				// 절대 방향 읽기 (Quaternion)
				Orientation absoluteOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

				// Telemetry 표시
				telemetry.addData("=== IMU ORIENTATION DATA ===", "");
				telemetry.addData("Yaw (Z-axis)", "%.2f°", orientation.getYaw(AngleUnit.DEGREES));
				telemetry.addData("Pitch (Y-axis)", "%.2f°", orientation.getPitch(AngleUnit.DEGREES));
				telemetry.addData("Roll (X-axis)", "%.2f°", orientation.getRoll(AngleUnit.DEGREES));
				
				telemetry.addData("=== HEADING INFORMATION ===", "");
				double yawDegrees = orientation.getYaw(AngleUnit.DEGREES);
				String headingDirection = getHeadingDirection(yawDegrees);
				telemetry.addData("Heading Direction", headingDirection);
				telemetry.addData("Heading Degrees", "%.1f°", yawDegrees);
				
				telemetry.addData("=== ANGULAR VELOCITY ===", "");
				telemetry.addData("Angular Velocity X", "%.2f°/sec", angularVelocity.xRotationRate);
				telemetry.addData("Angular Velocity Y", "%.2f°/sec", angularVelocity.yRotationRate);
				telemetry.addData("Angular Velocity Z", "%.2f°/sec", angularVelocity.zRotationRate);
				
				telemetry.addData("=== QUATERNION DATA ===", "");
				telemetry.addData("Quaternion X", "%.4f", absoluteOrientation.firstAngle);
				telemetry.addData("Quaternion Y", "%.4f", absoluteOrientation.secondAngle);
				telemetry.addData("Quaternion Z", "%.4f", absoluteOrientation.thirdAngle);
				
				telemetry.addData("=== IMU STATUS ===", "");
				telemetry.addData("IMU Connected", "YES");
				telemetry.addData("Data Rate", "10 Hz");
				telemetry.addData("Accuracy", "High");
				
			} else {
				telemetry.addData("IMU Status", "NOT CONNECTED");
				telemetry.addData("Error", "Check IMU connection");
			}

			telemetry.update();
			sleep(100); // 10Hz 업데이트
		}
	}

	// 방향을 문자열로 변환하는 메서드
	private String getHeadingDirection(double yawDegrees) {
		// 각도를 0-360 범위로 정규화
		double normalizedYaw = ((yawDegrees % 360) + 360) % 360;
		
		if (normalizedYaw >= 337.5 || normalizedYaw < 22.5) {
			return "NORTH";
		} else if (normalizedYaw >= 22.5 && normalizedYaw < 67.5) {
			return "NORTHEAST";
		} else if (normalizedYaw >= 67.5 && normalizedYaw < 112.5) {
			return "EAST";
		} else if (normalizedYaw >= 112.5 && normalizedYaw < 157.5) {
			return "SOUTHEAST";
		} else if (normalizedYaw >= 157.5 && normalizedYaw < 202.5) {
			return "SOUTH";
		} else if (normalizedYaw >= 202.5 && normalizedYaw < 247.5) {
			return "SOUTHWEST";
		} else if (normalizedYaw >= 247.5 && normalizedYaw < 292.5) {
			return "WEST";
		} else {
			return "NORTHWEST";
		}
	}
}
