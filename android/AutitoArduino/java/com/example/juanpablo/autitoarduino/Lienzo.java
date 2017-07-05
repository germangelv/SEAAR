package com.example.juanpablo.autitoarduino;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Path;
import android.support.annotation.Nullable;
import android.util.AttributeSet;
import android.view.MotionEvent;
import android.view.View;

/**
 * Created by Juan Pablo on 28/6/2017.
 */

public class Lienzo extends View {

    //Path que utilizaré para ir pintando las lineas, va guardando el trazo por el cual vas pintando
    private Path drawPath;
    //Paint de dibujar, es como un pincel
    private static Paint drawPaint;
    //otro pincel
    private Paint canvasPaint;
    //Color Inicial rojo
    private static int paintColor = 0xFFFF0000;
    //canvas, el lugar sobre vas a pitnar ,el lienzo, el fondo
    private Canvas drawCanvas;
    //canvas para guardar
    private Bitmap canvasBitmap;


    public Lienzo(Context context, AttributeSet attrs) {
        super(context, attrs);
        setupDrawing();
    }

    private void setupDrawing(){

        //Configuración del area sobre la que pintar

        drawPath = new Path();
        drawPaint = new Paint();
        drawPaint.setColor(paintColor);
        drawPaint.setAntiAlias(true);

        //setTamanyoPunto(20);

        drawPaint.setStrokeWidth(10);
        drawPaint.setStyle(Paint.Style.STROKE);
        drawPaint.setStrokeJoin(Paint.Join.ROUND);
        drawPaint.setStrokeCap(Paint.Cap.ROUND);
        canvasPaint = new Paint(Paint.DITHER_FLAG);


    }

    //Tamaño asignado a la vista
    @Override
    protected void onSizeChanged(int w, int h, int oldw, int oldh) {
        super.onSizeChanged(w, h, oldw, oldh);
        canvasBitmap = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888);
        drawCanvas = new Canvas(canvasBitmap);
        drawCanvas.translate(300,600);
    }

    //Pinta la vista.
    @Override
    protected void onDraw(Canvas canvas) {
        canvas.drawBitmap(canvasBitmap, 0, 0, canvasPaint);
        canvas.drawPath(drawPath, drawPaint);
    }

    public void dibujar(float x, float y){
        drawCanvas.drawPoint(x,y,drawPaint);
        invalidate();
    }

}
