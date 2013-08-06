package edu.oregonstate.robotics;

import java.net.*;
import java.io.*;
import java.nio.ByteBuffer;
import android.util.Log;

public class RosSensorIO implements Runnable {
	Socket client;
	DataOutputStream outStream;
	String serverName;
	int port;	
	
	RosSensorIO(String serverName, int port) {
		this.serverName = serverName;
		this.port = port;
	}
	
	public void sendRPY(float roll, float pitch, float yaw) {
		try {			
			// this can be deserialized in python with struct.unpack('>3f', data)
			ByteBuffer bb = ByteBuffer.allocate(12);
			bb.putFloat(-roll).putFloat(pitch).putFloat(-yaw+180);
			this.outStream.write(bb.array());
		} catch (IOException e) {
			Log.w("RosSensorIO", "IO Exception: trying to reconnect");
			try {
				this.client.close();
				this.run();
			} catch (IOException e1) {
				e1.printStackTrace();
			}
			
		} catch (Exception e) {
			@SuppressWarnings("unused")
			int a = 5;
		}
	}

	public void run() {
		try {
			this.client = new Socket(this.serverName, this.port);
			OutputStream outToServer = client.getOutputStream();
			outStream = new DataOutputStream(outToServer);
		} catch (UnknownHostException e) {
			Log.e("RosSensorIO", "UnknownHostException", e);
		} catch (IOException e) {
			Log.e("RosSensorIO", "IOException", e);
		}
	}

}
