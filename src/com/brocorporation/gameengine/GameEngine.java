package com.brocorporation.gameengine;

import org.lwjgl.opengl.Display;

public abstract class GameEngine implements IUpdateInfo {

	protected boolean isFixedTimeSteps;
	protected boolean isRunning, isPaused;
	protected double framePeriod;
	protected float inverseRate;
	protected float rate;
	protected float halfRate;
	protected boolean isPreRendering = true;

	public GameEngine() {
		isFixedTimeSteps = false;
	}

	public GameEngine(final int pUPS) {
		framePeriod = 1000d / pUPS;
		setRate(1f / pUPS);
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
		long startTime = System.nanoTime();
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

	public void runFixedTimeSteps() {
		double lastTime = System.nanoTime();
		double lag = 0;
		isPreRendering(true);
		boolean doRendering = false;
		while (isRunning && !Display.isCloseRequested()) {
			double currentTime = System.nanoTime();
			double timeDiff = (currentTime - lastTime) / 1000000;
			lastTime = currentTime;
			lag += timeDiff;
			doRendering = lag >= framePeriod;
			if (doRendering) {
				isPreRendering(false);
				while (true) {
					lag -= framePeriod;
					if (lag < framePeriod) {
						isPreRendering(true);
						update(this);
						break;
					}
					update(this);
				}
			}
			if (Display.wasResized()) {
				onSurfaceChanged(Display.getWidth(), Display.getHeight());
				doRendering = true;
			}
			if (doRendering) {
				render();
				Display.update();
			} else {
				long sleep = Math.round(framePeriod - lag);
				if (sleep == 0)
					sleep = 1;
				try {
					Thread.sleep(sleep);
				} catch (InterruptedException e) {
				}
			}
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
