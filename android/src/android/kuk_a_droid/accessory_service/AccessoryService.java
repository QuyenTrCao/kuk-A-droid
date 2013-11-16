package android.kuk_a_droid.accessory_service;

import java.lang.ref.WeakReference;
import java.lang.reflect.InvocationTargetException;

import android.app.Notification;
import android.app.PendingIntent;
import android.app.Service;
import android.content.Intent;
import android.kuk_a_droid.MainActivity;
import android.kuk_a_droid.R;
import android.kuk_a_droid.accessory_service.AccessoryEngine.IAccessoryEngineCallback;
import android.kuk_a_droid.log.L;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import android.os.ResultReceiver;

public final class AccessoryService extends Service {

	private ResultReceiver mResultReceiver = null;
	private Notification mServiceNotification = null;

	private static final int NOTIFICATION_ID = 1;

	public static final String ACTION_START_SERVICE = "android.kuk_a_droid.accessory_service.ACTION_START_SERVICE";
	public static final String ACTION_STOP_SERVICE = "android.kuk_a_droid.accessory_service.ACTION_STOP_SERVICE";

	public static final String EXTRA_RESULT_RECEIVER = "android.kuk_a_droid.accessory_service.RESULT_RECEIVER";
	
	public static final int RESULT_CODE_DATA = 1;
	public static final int RESULT_KUKA_CONNECTED = 2;
	
	private AccessoryEngine mEngine = null;
	private IncomingHandler mHandler = null;

	@Override
	public int onStartCommand(Intent intent, int flags, int startId) {
		if (intent != null)
			handleIntent(intent, startId);

		return START_STICKY;
	}

	private void handleIntent(Intent intent, int startId) {
		if (mHandler  == null) {
			mHandler = new IncomingHandler(this);
		}

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
			
			if(mEngine.isOngoing()){
				mResultReceiver.send(RESULT_KUKA_CONNECTED, mData);
			}
			
			mHandler.removeMessages(MSG_SEND_DATA);
			mHandler.sendEmptyMessage(MSG_SEND_DATA);
		} else if (ACTION_STOP_SERVICE.equals(intent.getAction())) {
			L.d("action stop");
			if(mEngine != null){
				if(mEngine.isOngoing())
					return;
			}
			
			stopForeground(true);
			stopSelf();
			System.exit(0);
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
		final PendingIntent pending = PendingIntent.getActivity(this, 0, new Intent(this, MainActivity.class), 0);
		return new Notification.Builder(this)
				.setContentTitle("Kuk-A-Droid Service")
				.setContentText("Service is running")
				.setContentIntent(pending)
				.setSmallIcon(R.drawable.ic_stat_service).build();
	}
	
	private static final int MSG_SEND_DATA = 0;
	
	
	
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

		@Override
		public void onAccessoryConnect() {
			if (mResultReceiver != null) {
				mResultReceiver.send(RESULT_KUKA_CONNECTED, mData);
			}
		}
	};
	
	private void handleMessage(Message message) {
		switch (message.what){
		case MSG_SEND_DATA:
			mEngine.send(new byte[]{1,2,3,4});
			mHandler.sendEmptyMessageDelayed(MSG_SEND_DATA, 1000);
			break;
		default:
			break;
		}
	}
	
	private static class IncomingHandler extends Handler {
		private final WeakReference<AccessoryService> mService;

		IncomingHandler(AccessoryService service) {
			mService = new WeakReference<AccessoryService>(service);
		}

		@Override
		public void handleMessage(Message msg) {
			AccessoryService service = mService.get();
			if (service != null) {
				service.handleMessage(msg);
			}else{
				L.e("service is null, seems dead object");
			}
		}
	}

}
