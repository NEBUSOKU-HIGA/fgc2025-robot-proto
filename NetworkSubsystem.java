package org.firstinspires.ftc.teamcode.subsystems;

import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.concurrent.atomic.AtomicBoolean;

public class NetworkSubsystem {
	private ServerSocket serverSocket;
	private final TopMotorSubsystem topMotor;
	private final AtomicBoolean serverRunning = new AtomicBoolean(false);
	private final int port;

	public NetworkSubsystem(int port, TopMotorSubsystem topMotor) {
		this.port = port;
		this.topMotor = topMotor;
	}

	public void startServer() {
		try {
			serverSocket = new ServerSocket(port);
			serverRunning.set(true);
			
			// サーバーを別スレッドで開始
			Thread serverThread = new Thread(() -> {
				while (serverRunning.get()) {
					try {
						Socket clientSocket = serverSocket.accept();
						handleClient(clientSocket);
					} catch (Exception e) {
						if (serverRunning.get()) {
							// エラーログ（必要に応じて）
						}
					}
				}
			});
			serverThread.start();
		} catch (Exception e) {
			// 初期化エラー
		}
	}

	public void stopServer() {
		serverRunning.set(false);
		try {
			if (serverSocket != null && !serverSocket.isClosed()) {
				serverSocket.close();
			}
		} catch (Exception e) {
			// 終了エラー
		}
	}

	private void handleClient(Socket clientSocket) {
		try (BufferedReader in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
			 PrintWriter out = new PrintWriter(clientSocket.getOutputStream(), true)) {
			
			String inputLine;
			while ((inputLine = in.readLine()) != null) {
				// シンプルなコマンド処理
				if (inputLine.startsWith("MOTOR:")) {
					try {
						double power = Double.parseDouble(inputLine.substring(6));
						topMotor.moveTop(power);
						out.println("OK");
					} catch (NumberFormatException e) {
						out.println("ERROR: Invalid power value");
					}
				} else {
					out.println("UNKNOWN_COMMAND");
				}
			}
		} catch (Exception e) {
			// クライアント処理エラー
		}
	}

	public void handleIncoming() {
		// メインループからの呼び出し（必要に応じて）
		// 現在は別スレッドで処理しているため、ここでは何もしない
	}
}