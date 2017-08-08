package pl.edu.pw.meil.seekurJrRC;

import java.io.IOException;
import java.io.OutputStream;
import java.net.InetAddress;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.util.Timer;
import java.util.TimerTask;
import java.util.zip.CRC32;

import android.app.Activity;
import android.app.AlertDialog;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.SharedPreferences;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.preference.PreferenceManager;
import android.view.KeyEvent;
import android.view.View;
import android.widget.ImageView;
import android.widget.TextView;
import android.widget.VideoView;

public class SeekurJrRCActivity
	extends Activity
	implements
		SensorEventListener,
		DialogInterface.OnKeyListener
	{
	
	private SensorManager mSensorManager;
    private Sensor mAccelerometer;	
	private boolean mLocked;
	private AlertDialog mLockedDialog;
	private Socket robotSocket;
	private Timer mSocketTimer;
	private Boolean mSocketGood = false;

	
	public SeekurJrRCActivity() {
		mSensorManager = null;
        mAccelerometer = null;
   	}

	
    /** Called when the activity is first created. */
    @Override
    public void onCreate(Bundle savedInstanceState) {
    	PreferenceManager.setDefaultValues(this, R.xml.app_preferences, false);  
    	super.onCreate(savedInstanceState);
    	setContentView(R.layout.main);
    	
        mLockedDialog = new AlertDialog.Builder(this)
        .setOnKeyListener(this) // now ain't that a dirty hack? :)
        .create();
        mLockedDialog.setTitle("Blokada!");
        mLockedDialog.setMessage("Aplikacja jest zablokowana, robot jest zatrzymany. Aby jeździć robotem, trzymaj wciśnięty przycisk migawki");
        // lock the app
        lock();

        mSensorManager = (SensorManager)getSystemService(SENSOR_SERVICE);
        mAccelerometer = mSensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        
    	mSocketTimer = new Timer();
    	mSocketTimer.schedule(new TimerTask() {
	    		@Override
	    		public void run() {
	    			// will periodically check if socket is good
	    			if (!mSocketGood) {
	    				connectToServer();
	    			}
	    		}
    		}, 0, 200);
    }
    
    /**
     * Gdy zostaniemy przywróceni, włączamy akcelerometr
     */
    protected void onResume() {
        super.onResume();
        mSensorManager.registerListener(this, mAccelerometer, SensorManager.SENSOR_DELAY_GAME);
    }

    /**
     * Przy uśpieniu (generalnie po straceniu focusu) wyłączamy akcelerometr - żeby nie wyduldać baterii
     */
    protected void onPause() {
        super.onPause();
        lock();
        mSensorManager.unregisterListener(this);
    }
    
    private void connectToServer() {
       	try {   
        	// 10.42.43.1:1025 for local testing 
        	// 173.194.67.103:80 for Google 
       		// 192.168.1.110:1025 for testing @ H69
       		SharedPreferences appPreferences = PreferenceManager.getDefaultSharedPreferences(this);        
        	robotSocket = new Socket(
        			InetAddress.getByAddress(
        					IPAddressUtil.textToNumericFormatV4(
        							appPreferences.getString("controlURL", "192.168.1.110")
        							)
        					), 
        					Integer.parseInt(appPreferences.getString("controlPort", "1025"))
        			);
        	robotSocket.setTcpNoDelay(true);
        	mSocketGood = true;
        } catch (IOException e) {
        	mSocketGood = false;
			TextView upperText = (TextView) findViewById(R.id.upper_text);
			upperText.setText("[ERROR] connecting to server message: ".concat(e.getMessage()));
        } 
    }
    
	
	/**
	 * może kiedyś się to do czegoś przyda
	 */
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    /**
     * główne miejsce, w którym reagujemy na wskazania akcelerometru
     */
    public void onSensorChanged(SensorEvent event) {
    	// just to make sure
    	if (event.sensor.getType() == Sensor.TYPE_ACCELEROMETER) {
    		ByteBuffer pre_crc32_message_buffer = ByteBuffer.allocate(16);
    		// STOP/START
    		pre_crc32_message_buffer.position(0);
    		if (mLocked == true)
    			pre_crc32_message_buffer.put((byte)0x00);
    		else
    			pre_crc32_message_buffer.put((byte)0xff);
    		
    		// Steering model code
    		pre_crc32_message_buffer.position(1);
    		SharedPreferences appPreferences = PreferenceManager.getDefaultSharedPreferences(this);        
        	pre_crc32_message_buffer.put(Byte.parseByte(appPreferences.getString("controlCode", "00"), 16));
        	
    		// x,y,z values
    		pre_crc32_message_buffer.putInt(4, Float.floatToRawIntBits(event.values[2]));
    		pre_crc32_message_buffer.putInt(8, Float.floatToRawIntBits(event.values[1]));
    		pre_crc32_message_buffer.putInt(12, Float.floatToRawIntBits(event.values[0]));
    		
    		byte[] pre_crc32_message_bytes = pre_crc32_message_buffer.array();
    		
    		// calculate crc32
    		CRC32 sum = new CRC32();
    		sum.update(pre_crc32_message_bytes);
    		int crc32_sum = (int) sum.getValue();
    		
    		ByteBuffer post_crc32_message_buffer = ByteBuffer.allocate(20);
    		post_crc32_message_buffer.put(pre_crc32_message_bytes);
    		post_crc32_message_buffer.putInt(16, crc32_sum);
    		
    		byte[] post_crc32_message_bytes = post_crc32_message_buffer.array(); 

    		try {
    			if (robotSocket == null)
    				throw new IllegalStateException("socket object unitialized");
    			OutputStream outStream = robotSocket.getOutputStream();    			
				outStream.write(post_crc32_message_bytes);
				outStream.flush();
				
		    	TextView upperText = (TextView) findViewById(R.id.upper_text);
	    		String textToPlace = "OK";
	    		upperText.setText(textToPlace);
	    		mSocketGood = true;
	    		
			} catch (IOException e) {
				try {
					robotSocket.close();
					robotSocket = null;
				} catch (IOException e1) {
					
				}
				robotSocket = null;
				TextView upperText = (TextView) findViewById(R.id.upper_text);
				upperText.setText("[ERROR] sending message: ".concat(e.getMessage()));
			} catch (IllegalStateException e) {
				mSocketGood = false;
				// connectToServer();
				TextView upperText = (TextView) findViewById(R.id.upper_text);
				upperText.setText("[ERROR] sending message: ".concat(e.getMessage()));
			}
    	}
    }

    public boolean onKey(DialogInterface dialog, int keyCode, KeyEvent event) {
    	if (event.getAction() == KeyEvent.ACTION_DOWN) {
    		return onKeyDown(keyCode, event);
    	} else if (event.getAction() == KeyEvent.ACTION_UP) {
    		return onKeyUp(keyCode, event);
    	} else {
    		return false;
    	}
    }
    
    public boolean onKeyDown(int keyCode, KeyEvent event) {
    	if (keyCode == KeyEvent.KEYCODE_CAMERA) {
    		unlock();
    		return true;
    	} else if (keyCode == KeyEvent.KEYCODE_DPAD_CENTER) {
    		Intent settingsActivity = new Intent(getBaseContext(), SeekurJrPreferenceActivity.class);
    		startActivity(settingsActivity);
        	return true;
    	} else if (keyCode == KeyEvent.KEYCODE_BACK && mLocked == true) {
    		return true;
    	}
    	return false;
    }
    
    public boolean onKeyUp(int keyCode, KeyEvent event) {
    	if (keyCode == KeyEvent.KEYCODE_CAMERA) {
    		lock();
    		return true;
    	}
    	return false;
    }
    
    /**
     * blokujemy aplikację, nie pozwalamy na żaden ruch
     */
    private void lock() {
    	mLocked = true;
    	mLockedDialog.show();
    }

    /**
     * odblokowujemy aplikację, pozwalamy na ruch robota
     */
    private void unlock() {
    	mLocked = false;
    	mLockedDialog.hide();
    }

}