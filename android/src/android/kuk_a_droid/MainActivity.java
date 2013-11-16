package android.kuk_a_droid;

import android.app.Activity;
import android.app.PendingIntent;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbManager;
import android.kuk_a_droid.accessory_service.AccessoryEngine;
import android.kuk_a_droid.accessory_service.AccessoryService;
import android.kuk_a_droid.log.L;
import android.os.Bundle;
import android.os.Handler;
import android.os.ResultReceiver;
import android.view.Menu;
import android.view.MenuItem;
import android.widget.TextView;

public final class MainActivity extends Activity {

	private ResultReceiver mResultReceiver = null;
	private TextView mStatusView = null;

	@Override
	protected void onCreate(Bundle savedInstanceState) {
		super.onCreate(savedInstanceState);
		mResultReceiver = new ResultReceiver(new Handler()) {
			protected void onReceiveResult(int resultCode, Bundle resultData) {
				onHandleResult(resultCode, resultData);
			};
		};

		setContentView(R.layout.activity_main);

		mStatusView = (TextView) findViewById(R.id.tv_status);

		handleIntent(getIntent());
	}

	@Override
	protected void onNewIntent(Intent intent) {
		handleIntent(intent);
		super.onNewIntent(intent);// accessory connected and app already in fg
	}

	@Override
	protected void onStop() {
		Intent serviceIntent = new Intent(AccessoryService.ACTION_STOP_SERVICE);
		serviceIntent.setClass(this, AccessoryService.class);
		startService(serviceIntent);
		super.onStop();
	}

	@Override
	protected void onDestroy() {
		mResultReceiver = null;
		super.onDestroy();
	}

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		getMenuInflater().inflate(R.menu.main, menu);
		return true;
	}

	@Override
	public boolean onOptionsItemSelected(MenuItem item) {
		switch (item.getItemId()) {
		case R.id.action_connect:
			connectAccessory();
			return true;
		default:
			break;
		}
		return super.onOptionsItemSelected(item);
	}

	private void handleIntent(Intent intent) {
		L.d("handling intent");
		if (intent.hasExtra(UsbManager.EXTRA_ACCESSORY)) {
			connectAccessory();
		}
		
		if(intent.getBooleanExtra(UsbManager.EXTRA_PERMISSION_GRANTED, false)){
			connectAccessory();
		}
	}

	private void onHandleResult(int resultCode, Bundle resultData) {
		switch (resultCode) {
		case AccessoryEngine.ERROR_OTHER:
			setStatusText("There was a generic error in the accessory engine");
			break;
		case AccessoryEngine.ERROR_NO_PERMISSION:
			requestPermission();
			break;
		case AccessoryEngine.ERROR_DEVICE_NOT_SUPPORTED:
			break;
		case AccessoryEngine.ERROR_DEVICE_NOT_FOUND:
			setStatusText("No Kuka-Bot found");
			break;
		case AccessoryEngine.ERROR_WRITER_ALREADY_STARTED:
			break;
		case AccessoryEngine.ERROR_WRITER_TASK_ABORTED:
			setStatusText("There was an error in the Accessory Thread (Writer)");
			break;
		case AccessoryEngine.ERROR_READER_TASK_ABORTED:
			setStatusText("There was an error in the Accessory Thread (Reader)");
			break;
		case AccessoryService.RESULT_KUKA_CONNECTED:
			setStatusText("Kuka-Bot is connected");
			break;
		default:
			break;
		}
	}

	private void setStatusText(final String message) {
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				mStatusView.setText(message);
			}
		});
	}

	private void connectAccessory() {
		Intent serviceIntent = new Intent(AccessoryService.ACTION_START_SERVICE);
		serviceIntent.setClass(this, AccessoryService.class);
		serviceIntent.putExtra(AccessoryService.EXTRA_RESULT_RECEIVER,
				mResultReceiver);
		startService(serviceIntent);
	}
	
	private void requestPermission(){
		try{
			final PendingIntent pi = PendingIntent.getActivity(this, 0, new Intent(this, MainActivity.class), 0);
			final UsbManager usbman = (UsbManager)getSystemService(USB_SERVICE);
			usbman.requestPermission(usbman.getAccessoryList()[0], pi);	
		}catch(Exception e){
			L.d("failed to request permission on accessory");
		}
	}

}
