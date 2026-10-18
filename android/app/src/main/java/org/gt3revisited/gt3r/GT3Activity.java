package org.gt3revisited.gt3r;

import android.content.Intent;
import android.net.Uri;
import android.os.Build;
import android.util.Log;
import android.view.DisplayCutout;
import android.view.RoundedCorner;
import android.view.View;
import android.view.WindowInsets;

import org.libsdl.app.SDLActivity;

/** Hosts the native engine. SDL is statically linked into libmain.so. */
public class GT3Activity extends SDLActivity {
    @Override
    protected String[] getLibraries() {
        return new String[] { "main" };
    }

    // The disc image comes from the system file picker as a content:// URI.
    // Keep read access across app restarts so the remembered disc (and the
    // recent-discs list) still opens next launch.
    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        if (resultCode == RESULT_OK && data != null && data.getData() != null
                && (data.getFlags() & Intent.FLAG_GRANT_PERSISTABLE_URI_PERMISSION) != 0) {
            Uri uri = data.getData();
            try {
                getContentResolver().takePersistableUriPermission(uri, Intent.FLAG_GRANT_READ_URI_PERMISSION);
            } catch (SecurityException e) {
                Log.w("GT3R", "Could not keep access to " + uri, e);
            }
        }
        super.onActivityResult(requestCode, resultCode, data);
    }

    /**
     * Called from native code (src/platform/display_insets.cpp): the display
     * cutout per edge and the largest rounded-corner radius, in pixels, as
     * {left, top, right, bottom, radius}; null when not known yet. Unlike
     * SDL's safe area this leaves out the system-gesture zones.
     */
    public int[] displayInsets() {
        if (Build.VERSION.SDK_INT < 28) return null;
        View decor = getWindow() != null ? getWindow().getDecorView() : null;
        WindowInsets insets = decor != null ? decor.getRootWindowInsets() : null;
        if (insets == null) return null;
        int[] out = new int[5];
        DisplayCutout cut = insets.getDisplayCutout();
        if (cut != null) {
            out[0] = cut.getSafeInsetLeft();
            out[1] = cut.getSafeInsetTop();
            out[2] = cut.getSafeInsetRight();
            out[3] = cut.getSafeInsetBottom();
        }
        if (Build.VERSION.SDK_INT >= 31) {
            int[] positions = {RoundedCorner.POSITION_TOP_LEFT, RoundedCorner.POSITION_TOP_RIGHT,
                               RoundedCorner.POSITION_BOTTOM_LEFT, RoundedCorner.POSITION_BOTTOM_RIGHT};
            for (int p : positions) {
                RoundedCorner c = insets.getRoundedCorner(p);
                if (c != null) out[4] = Math.max(out[4], c.getRadius());
            }
        }
        return out;
    }
}
