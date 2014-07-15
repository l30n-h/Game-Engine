package com.brocorporation.gameengine.elements.opengl;

import org.lwjgl.opengl.GL15;

public abstract class GLShape {

	protected final static byte BYTES_PER_SHORT = 2;
	protected final static byte BYTES_PER_FLOAT = 4;

	public abstract void initBuffer();

	public abstract int[] getBuffer();

	public void release() {
		final int[] buffer = getBuffer();
		if (buffer != null) {
			for (int i = buffer.length - 1; i >= 0; i--) {
				GL15.glDeleteBuffers(buffer[i]);
			}
		}
	}
}