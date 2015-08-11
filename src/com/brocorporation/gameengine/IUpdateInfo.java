package com.brocorporation.gameengine;

public interface IUpdateInfo {

	public int getInverseRate();
	
	public float getRate();
	
	public float getHalfRate();

	public boolean isPreRendering();
	
	public boolean isPaused();
	
	public void pause();
	
	public void resume();
}
