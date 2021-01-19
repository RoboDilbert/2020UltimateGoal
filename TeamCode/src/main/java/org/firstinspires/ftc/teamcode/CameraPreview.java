package org.firstinspires.ftc.teamcode;


import android.content.Context;
import android.hardware.Camera;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * A basic Camera preview class, largely from the example:
 * https://developer.android.com/guide/topics/media/camera#sample
 * */
class CameraPreview extends SurfaceView implements SurfaceHolder.Callback {
    private static final String TAG = "CameraPreview";

    private final SurfaceHolder holder;
    private final Camera camera;
    private final AtomicBoolean cameraReleased;

    public CameraPreview(Context context, Camera camera, AtomicBoolean cameraReleased) {
        super(context);
        this.camera = camera;
        this.cameraReleased = cameraReleased;

        // Install a SurfaceHolder.Callback so we get notified when the
        // underlying surface is created and destroyed.
        holder = getHolder();
        holder.addCallback(this);
        // deprecated setting, but required on Android versions prior to 3.0
        holder.setType(SurfaceHolder.SURFACE_TYPE_PUSH_BUFFERS);
    }

    private void startCamera(SurfaceHolder holder) {
        try {
            if (!cameraReleased.get()) {
                camera.setPreviewDisplay(holder);
                camera.startPreview();
            }
        } catch (IOException e) {
            Log.e(TAG, "Error setting up camera preview", e);
        }
    }

    private void stopCamera() {
        try {
            if (!cameraReleased.get()) {
                camera.stopPreview();
            }
        } catch (Exception e) {
            // Doesn't matter, camera is stopped.
        }
    }

    public void surfaceCreated(SurfaceHolder holder) {
        // The Surface has been created, now tell the camera where to draw the preview.
        startCamera(holder);
    }

    public void surfaceDestroyed(SurfaceHolder holder) {
        // empty. Take care of releasing the Camera preview in your activity.
        stopCamera();
    }

    public void surfaceChanged(SurfaceHolder holder, int format, int w, int h) {
        // If your preview can change or rotate, take care of those events here.
        // Make sure to stop the preview before resizing or reformatting it.

        if (this.holder.getSurface() == null){
            // preview surface does not exist
            return;
        }

        stopCamera();
        startCamera(holder);
    }

    public void hide() {
        this.setVisibility(GONE);
        this.postInvalidate();
    }
}
