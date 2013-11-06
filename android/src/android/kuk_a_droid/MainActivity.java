package android.kuk_a_droid;

import android.app.Activity;
import android.content.Intent;
import android.kuk_a_droid.accessory_service.AccessoryEngine;
import android.kuk_a_droid.accessory_service.AccessoryService;
import android.kuk_a_droid.log.L;
import android.os.Bundle;
import android.os.Handler;
import android.os.ResultReceiver;
import android.view.Menu;
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

	private void handleIntent(Intent intent) {
		L.d("handling intent");

		Intent serviceIntent = new Intent(AccessoryService.ACTION_START_SERVICE);
		serviceIntent.setClass(this, AccessoryService.class);
		serviceIntent.putExtra(AccessoryService.EXTRA_RESULT_RECEIVER,
				mResultReceiver);
		startService(serviceIntent);
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

	private void onHandleResult(int resultCode, Bundle resultData) {
		switch (resultCode) {
		case AccessoryEngine.ERROR_OTHER:
			setStatusText("There was a generic error in the accessory engine");
			break;
		case AccessoryEngine.ERROR_NO_PERMISSION:
			break;
		case AccessoryEngine.ERROR_DEVICE_NOT_SUPPORTED:
			break;
		case AccessoryEngine.ERROR_DEVICE_NOT_FOUND:
			setStatusText("No Kuka-Bot found");
			break;
		case AccessoryEngine.ERROR_WRITER_ALREADY_STARTED:
			break;
		case AccessoryEngine.ERROR_WRITER_TASK_ABORTED:
			break;
		default:
			break;
		}
	}
	
	private void setStatusText(final String message){
		runOnUiThread(new Runnable() {
			@Override
			public void run() {
				mStatusView.setText(message);
			}
		});
	}
	

	@Override
	public boolean onCreateOptionsMenu(Menu menu) {
		getMenuInflater().inflate(R.menu.main, menu);
		return true;
	}
}
