package com.brocorporation.gameengine.parser;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.ShortBuffer;
import java.util.ArrayList;

import com.brocorporation.gameengine.elements.opengl.GLMesh;

public class WavefrontObject {

	private final ArrayList<GLMesh> meshList = new ArrayList<GLMesh>();
	private ArrayList<float[]> vboList = new ArrayList<float[]>();
	private ArrayList<Short> iboList = new ArrayList<Short>();
	private ShortBuffer ibo;
	private FloatBuffer vbo;

	public void addIndex(final Short pIndex) {
		iboList.add(pIndex);
	}

	public void addVBO(final float[] vertex) {
		vboList.add(vertex);
	}

	public void addMesh(final GLMesh pMesh) {
		meshList.add(pMesh);
	}

	public ShortBuffer getIBO() {
		if (ibo == null && iboList != null) {
			final ShortBuffer buffer = ByteBuffer
					.allocateDirect(iboList.size() * 2)
					.order(ByteOrder.nativeOrder()).asShortBuffer();
			for (final short index : iboList) {
				buffer.put(index);
			}
			buffer.flip();
			ibo = buffer;
			iboList.clear();
			iboList = null;
		}
		return ibo;
	}

	public FloatBuffer getVBO() {
		if (vbo == null && vboList != null) {
			final int size = vboList.size() / 3;
			final FloatBuffer buffer = ByteBuffer
					.allocateDirect(size * 32)
					.order(ByteOrder.nativeOrder()).asFloatBuffer();
			for (final float[] vertex : vboList) {
				for (final float v : vertex) {
					buffer.put(v);
				}
			}
			buffer.flip();
			vbo = buffer;
			vboList.clear();
			vboList = null;
		}
		return vbo;
	}

	public GLMesh[] getMesh() {
		return meshList.toArray(new GLMesh[meshList.size()]);
	}
}
