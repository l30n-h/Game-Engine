package com.brocorporation.gameengine;

import org.lwjgl.opengl.Display;

public abstract class GameEngine implements IUpdateInfo {

	protected boolean isFixedTimeSteps;
	protected boolean isRunning, isPaused;
	protected int maxFPS;
	protected int maxFramesSkipped;
	protected int framePeriod;
	protected float inverseRate;
	protected float rate;
	protected float halfRate;
	protected boolean isPreRendering = true;
	protected long startTime;
	protected long sleepTime;

	public GameEngine() {
		isFixedTimeSteps = false;
	}

	public GameEngine(final int pFPS, final int pMaxFramesSkipped) {
		maxFPS = pFPS;
		maxFramesSkipped = pMaxFramesSkipped;
		framePeriod = 1000 / maxFPS;
		setRate(1F / maxFPS);
		isFixedTimeSteps = true;
	}

	protected abstract void create();

	protected abstract void load();

	protected abstract void init();

	protected abstract void onSurfaceChanged(int pWidth, int pHeight);

	// protected abstract void onSurfaceCreated();

	protected abstract void update(final IUpdateInfo uInfo);

	protected abstract void render();

	public final void runVariableTimeSteps() {
		startTime = System.nanoTime();
		isPreRendering(true);
		while (isRunning && !Display.isCloseRequested()) {
			if (Display.wasResized()) {
				onSurfaceChanged(Display.getWidth(), Display.getHeight());
			}
			final long lastTime = startTime;
			startTime = System.nanoTime();
			setRate((float) ((startTime - lastTime) * 1E-9));
			update(this);
			render();
			Display.update();
		}
		Display.destroy();
	}

	public final void runFixedTimeSteps() {
		startTime = System.nanoTime();
		while (isRunning && !Display.isCloseRequested()) {
			if (Display.wasResized()) {
				onSurfaceChanged(Display.getWidth(), Display.getHeight());
			}
			final long timeDiff = (System.nanoTime() - startTime) / 1000000L;
			sleepTime += framePeriod - timeDiff;
			if (sleepTime > 0) {
				try {
					Thread.sleep(sleepTime);
				} catch (InterruptedException e) {
				}
				sleepTime = 0;
			} else {
				int framesSkipped = 0;
				isPreRendering(false);
				while (sleepTime <= -framePeriod
						&& framesSkipped < maxFramesSkipped) {
					update(this);
					sleepTime += framePeriod;
					framesSkipped++;
				}
				isPreRendering(true);
			}
			startTime = System.nanoTime();
			update(this);
			render();
			Display.update();
		}
		Display.destroy();
	}

	public void start() {
		if (!isRunning) {
			create();
			load();
			init();
			onSurfaceChanged(Display.getWidth(), Display.getHeight());
			isRunning = true;
			resume();
			if (isFixedTimeSteps) {
				runFixedTimeSteps();
			} else {
				runVariableTimeSteps();
			}
		}
	}

	@Override
	public boolean isPaused() {
		return isPaused;
	}

	@Override
	public void pause() {
		isPaused = true;
	}

	@Override
	public void resume() {
		isPaused = false;
	}

	public void stop() {
		pause();
		isRunning = false;
	}

	public void setRate(final float dRate) {
		rate = dRate;
		halfRate = rate / 2;
		inverseRate = 1 / rate;
	}

	public void isPreRendering(final boolean pPreRendering) {
		isPreRendering = pPreRendering;
	}

	@Override
	public float getInverseRate() {
		return inverseRate;
	}

	@Override
	public float getRate() {
		return rate;
	}

	@Override
	public float getHalfRate() {
		return halfRate;
	}

	@Override
	public boolean isPreRendering() {
		return isPreRendering;
	}
}
