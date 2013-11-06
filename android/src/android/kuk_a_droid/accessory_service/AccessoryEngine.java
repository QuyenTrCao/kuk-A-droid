package android.kuk_a_droid.accessory_service;

import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;

import android.content.Context;
import android.hardware.usb.UsbAccessory;
import android.hardware.usb.UsbManager;
import android.kuk_a_droid.log.L;
import android.os.ParcelFileDescriptor;

public final class AccessoryEngine {

	private final Context mContext;
	private final IAccessoryEngineCallback mCallback;

	public static final int ERROR_OTHER = -1;
	public static final int ERROR_NO_PERMISSION = -2;
	public static final int ERROR_DEVICE_NOT_SUPPORTED = -3;
	public static final int ERROR_DEVICE_NOT_FOUND = -4;
	public static final int ERROR_WRITER_ALREADY_STARTED = -5;
	public static final int ERROR_WRITER_TASK_ABORTED = -6;

	protected interface IAccessoryEngineCallback {
		void onNewData(byte[] data);

		void onError(int errorCode);
	}

	public AccessoryEngine(Context context, IAccessoryEngineCallback callback) {
		mContext = context;
		mCallback = callback;
	}

	public void start() {
		if (mEngineThread == null) {
			mEngineThread = new Thread(mEngineRunnable, "Engine Thread");
			mEngineThread.start();
		} else {
			L.d("engine already started");
		}
	}

	public void send(final byte[] data) {
		mWriterLock.lock();
		mWriterBuffer.put(data);
		mWriterCondition.signal();
		mWriterLock.unlock();
	}

	private static Thread mEngineThread = null;
	private final Runnable mEngineRunnable = new Runnable() {
		@Override
		public void run() {
			try {
				engineTask();
			} catch (Exception e) {
				mCallback.onError(ERROR_OTHER);
				e.printStackTrace();
			}
			mEngineThread = null;
		}
	};

	private static final int BUFFER_SIZE = 10 * 1024;
	private final ByteBuffer mBuffer = ByteBuffer.allocate(BUFFER_SIZE);
	private FileOutputStream mOutputStream = null;

	private void engineTask() throws Exception {
		final UsbManager usbman = (UsbManager) mContext
				.getSystemService(Context.USB_SERVICE);
		final UsbAccessory[] accessories = usbman.getAccessoryList();
		if (accessories == null || accessories.length == 0) {
			mCallback.onError(ERROR_DEVICE_NOT_FOUND);
			return;
		}

		final UsbAccessory accessory = accessories[0];
		if (!usbman.hasPermission(accessory)) {
			mCallback.onError(ERROR_NO_PERMISSION);
			return;
		}

		final ParcelFileDescriptor parcelDescriptor = usbman
				.openAccessory(accessory);
		FileInputStream inputStream = new FileInputStream(
				parcelDescriptor.getFileDescriptor());
		FileOutputStream mOutputStream = new FileOutputStream(
				parcelDescriptor.getFileDescriptor());

		if (sWriterThread == null) {
			sWriterThread = new Thread(mWriterRunnable,
					"Accessory Writer Thread");
			sWriterThread.start();
		} else {
			L.e("Could not start writer thread! This should not happen!");
			mCallback.onError(ERROR_WRITER_ALREADY_STARTED);
		}

		try {
			for (;;) {
				int len = inputStream.read(mBuffer.array());
				mBuffer.position(0);
				mBuffer.limit(len);
				mCallback.onNewData(mBuffer.slice().array());
			}
		} catch (IOException e) {
			L.e("ioexecption in accessory engine thread");
		}

		inputStream.close();
		mOutputStream.close();
	}

	private static final int WRITER_BUFFER = 1024 * 1000;
	private final ByteBuffer mWriterBuffer = ByteBuffer.allocate(WRITER_BUFFER);
	private final byte[] mWriterData = new byte[WRITER_BUFFER];

	private final ReentrantLock mWriterLock = new ReentrantLock();
	private final Condition mWriterCondition = mWriterLock.newCondition();

	private Thread sWriterThread = null;
	private final Runnable mWriterRunnable = new Runnable() {
		@Override
		public void run() {
			try {
				writerTask();
			} catch (Exception e) {
				L.e("error during writer task");
			}
			sWriterThread = null;
		}
	};

	private void writerTask() throws Exception {
		for (;;) {
			mWriterLock.lock();
			mWriterCondition.await();
			int pos = mWriterBuffer.position();
			mWriterBuffer.position(0);
			mWriterBuffer.get(mWriterData, 0, pos);
			mWriterLock.unlock();

			mOutputStream.write(mWriterData, 0, pos);
		}
	}

}
