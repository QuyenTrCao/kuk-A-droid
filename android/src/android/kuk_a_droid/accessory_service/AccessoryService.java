package android.kuk_a_droid.accessory_service;

import android.app.Notification;
import android.app.Service;
import android.content.Intent;
import android.kuk_a_droid.R;
import android.kuk_a_droid.accessory_service.AccessoryEngine.IAccessoryEngineCallback;
import android.kuk_a_droid.log.L;
import android.os.Bundle;
import android.os.IBinder;
import android.os.ResultReceiver;

public final class AccessoryService extends Service {

	private ResultReceiver mResultReceiver = null;
	private Notification mServiceNotification = null;

	private static final int NOTIFICATION_ID = 1;

	public static final String ACTION_START_SERVICE = "android.kuk_a_droid.accessory_service.ACTION_START_SERVICE";
	public static final String ACTION_STOP_SERVICE = "android.kuk_a_droid.accessory_service.ACTION_STOP_SERVICE";

	public static final String EXTRA_RESULT_RECEIVER = "android.kuk_a_droid.accessory_service.RESULT_RECEIVER";
	
	private static final int RESULT_CODE_DATA = 1;
	private AccessoryEngine mEngine = null;

	@Override
	public int onStartCommand(Intent intent, int flags, int startId) {
		if (intent != null)
			handleIntent(intent, startId);

		return START_STICKY;
	}

	private void handleIntent(Intent intent, int startId) {
		if (intent.hasExtra(EXTRA_RESULT_RECEIVER)) {
			mResultReceiver = intent.getParcelableExtra(EXTRA_RESULT_RECEIVER);
		}

		if (ACTION_START_SERVICE.equals(intent.getAction())) {
			L.d("action start");
			startForeground(NOTIFICATION_ID, setupNotification());
			
			if(mEngine == null){
				 mEngine = new AccessoryEngine(this, mAccessoryEngineCallback);
			}
			
			mEngine.start();
		} else if (ACTION_STOP_SERVICE.equals(intent.getAction())) {
			L.d("action stop");
			if(stopSelfResult(startId)){
				stopForeground(true);
				System.exit(0);
			}
		}
	}

	@Override
	public IBinder onBind(Intent intent) {
		throw new UnsupportedOperationException(
				"Do not bind the service, use startService for communicaton");
	}

	private Notification setupNotification() {
		if (mServiceNotification != null)
			return mServiceNotification;
		return new Notification.Builder(this)
				.setContentTitle("Kuk-A-Droid Service")
				.setContentText("Service is running")
				.setSmallIcon(R.drawable.ic_launcher).build();
	}
	
	private final Bundle mData = new Bundle();
	private final IAccessoryEngineCallback mAccessoryEngineCallback = new IAccessoryEngineCallback() {
		@Override
		public void onNewData(byte[] data) {
			if (mResultReceiver != null) {
				mResultReceiver.send(RESULT_CODE_DATA, mData);
			}
		}

		@Override
		public void onError(int errorCode) {
			if (mResultReceiver != null) {
				mResultReceiver.send(errorCode, mData);
				stopForeground(true);
				System.exit(1);
			}
		}
	};
}
