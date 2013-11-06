package android.kuk_a_droid.log;

import android.kuk_a_droid.BuildConfig;
import android.util.Log;

public final class L {
	private static final boolean SHUT_UP = false;
	private static final String TAG = ">==< KUK-A-DROID >==<"; 

	public static void d(Object o){
		if(BuildConfig.DEBUG && !SHUT_UP)
			Log.d(TAG, String.valueOf(o));
	}
	public static void d(String s, Object ... args){
		if(BuildConfig.DEBUG && !SHUT_UP)
			Log.d(TAG, String.format(s,args));
	}
	
	public static void e(Object o){
		if(BuildConfig.DEBUG && !SHUT_UP)
			Log.e(TAG, String.valueOf(o));
	}
}




