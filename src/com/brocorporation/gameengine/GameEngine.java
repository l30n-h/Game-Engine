package com.brocorporation.gameengine;

import org.lwjgl.opengl.Display;

public abstract class GameEngine implements IUpdateInfo {

	protected boolean isRunning, isPaused;
	protected double framePeriod;
	protected int inverseRate;
	protected float rate;
	protected float halfRate;
	protected boolean isPreRendering = true;

	public GameEngine() {
		this(60);
	}

	public GameEngine(final int pUPS) {
		setRate(pUPS);
	}

	protected abstract void create();

	protected abstract void load();

	protected abstract void init();

	protected abstract void onSurfaceChanged(int pWidth, int pHeight);

	// protected abstract void onSurfaceCreated();

	protected abstract void update(final IUpdateInfo uInfo);

	protected abstract void render();

	public void runFixedTimeSteps() {
		long lastTime = System.nanoTime();
		double lag = 0;
		isPreRendering(true);
		boolean doRendering = false;
		while (isRunning && !Display.isCloseRequested()) {
			long currentTime = System.nanoTime();
			double timeDiff = (currentTime - lastTime) / 1000000d;
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
			runFixedTimeSteps();
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

	public void setRate(final int pUPS) {
		framePeriod = 1000d / pUPS;
		rate = 1f / pUPS;
		halfRate = rate / 2;
		inverseRate = pUPS;
	}

	public void isPreRendering(final boolean pPreRendering) {
		isPreRendering = pPreRendering;
	}

	@Override
	public int getInverseRate() {
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
