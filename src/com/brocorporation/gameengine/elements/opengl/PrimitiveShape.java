package com.brocorporation.gameengine.elements.opengl;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL20;

import com.brocorporation.gameengine.elements.collision.AABB;
import com.brocorporation.gameengine.utils.Vector3f;
import com.brocorporation.gameengine.utils.Vector4f;

public class PrimitiveShape extends GLShape {

	protected final static byte COORDS_PER_VERTEX = 3;
	protected final static byte STRIDE = COORDS_PER_VERTEX * BYTES_PER_FLOAT;

	protected Vector4f color = new Vector4f(0, 0, 0, 1);
	protected boolean colorChanged = true;
	protected final FloatBuffer pointBuffer;
	protected final FloatBuffer linesBuffer;
	protected final FloatBuffer trianglesBuffer;
	protected int pointsCount, linesPointsCount, trianglesPointsCount,
			maxPoints = 30000, maxLinesPoints = 60000,
			maxTrianglesPoints = 300;

	protected final PrimitiveShader shader;

	public PrimitiveShape(final PrimitiveShader glShader) {
		shader = glShader;
		pointBuffer = ByteBuffer.allocateDirect(maxPoints * STRIDE)
				.order(ByteOrder.nativeOrder()).asFloatBuffer();
		linesBuffer = ByteBuffer.allocateDirect(maxLinesPoints * STRIDE)
				.order(ByteOrder.nativeOrder()).asFloatBuffer();
		trianglesBuffer = ByteBuffer
				.allocateDirect(maxTrianglesPoints * STRIDE)
				.order(ByteOrder.nativeOrder()).asFloatBuffer();
	}

	public void setColor(final float pR, final float pG, final float pB,
			final float pA) {
		color.set(pR, pG, pB, pA);
		colorChanged = true;
	}

	public void addPoint(float x1, float y1, float z1) {
		if (pointsCount < maxPoints) {
			pointBuffer.put(x1).put(y1).put(z1);
			pointsCount += 1;
		}
	}

	public void addPoint(Vector3f p) {
		if (pointsCount < maxPoints) {
			pointBuffer.put(p.x).put(p.y).put(p.z);
			pointsCount += 1;
		}
	}

	public void addLine(float x1, float y1, float z1, float x2, float y2,
			float z2) {
		if (linesPointsCount < maxLinesPoints) {
			linesBuffer.put(x1).put(y1).put(z1);
			linesBuffer.put(x2).put(y2).put(z2);
			linesPointsCount += 2;
		}
	}

	public void addLine(Vector3f from, Vector3f to) {
		if (linesPointsCount < maxLinesPoints) {
			linesBuffer.put(from.x).put(from.y).put(from.z);
			linesBuffer.put(to.x).put(to.y).put(to.z);
			linesPointsCount += 2;
		}
	}

	public void addTriangle(float x1, float y1, float z1, float x2, float y2,
			float z2, float x3, float y3, float z3) {
		if (trianglesPointsCount < maxTrianglesPoints) {
			trianglesBuffer.put(x1).put(y1).put(z1);
			trianglesBuffer.put(x2).put(y2).put(z2);
			trianglesBuffer.put(x3).put(y3).put(z3);
			trianglesPointsCount += 3;
		}
	}

	public void addAABB(AABB bounds) {
		final Vector3f p = bounds.getPosition();
		final Vector3f h = bounds.getHalfsize();
		final float r = p.x + h.x;
		final float l = p.x - h.x;
		final float u = p.y + h.y;
		final float d = p.y - h.y;
		final float f = p.z + h.z;
		final float b = p.z - h.z;
		addLine(r, u, f, r, d, f);
		addLine(l, u, f, l, d, f);
		addLine(r, u, b, r, d, b);
		addLine(l, u, b, l, d, b);
		addLine(r, u, f, l, u, f);
		addLine(r, u, b, l, u, b);
		addLine(r, u, f, r, u, b);
		addLine(l, u, f, l, u, b);
		addLine(r, d, f, l, d, f);
		addLine(r, d, b, l, d, b);
		addLine(r, d, f, r, d, b);
		addLine(l, d, f, l, d, b);
	}

	public void reset() {
		pointBuffer.clear();
		linesBuffer.clear();
		trianglesBuffer.clear();
		pointsCount = 0;
		linesPointsCount = 0;
		trianglesPointsCount = 0;
	}

	public void render(final float[] mvpMatrix) {
		final int[] aHandle = shader.aHandle;
		shader.setMVPMatrix(mvpMatrix);
		if (colorChanged) {
			shader.setColor(color);
			colorChanged = false;
		}

		final int positionHandle = aHandle[PrimitiveShader.a_Position];
		GL20.glEnableVertexAttribArray(positionHandle);
		if (pointsCount > 0) {
			pointBuffer.flip();
			GL20.glVertexAttribPointer(positionHandle, COORDS_PER_VERTEX,
					false, STRIDE, pointBuffer);
			GL11.glDrawArrays(GL11.GL_POINTS, 0, pointsCount);
		}
		if (linesPointsCount > 1) {
			linesBuffer.flip();
			GL20.glVertexAttribPointer(positionHandle, COORDS_PER_VERTEX,
					false, STRIDE, linesBuffer);
			GL11.glDrawArrays(GL11.GL_LINES, 0, linesPointsCount);
		}
		if (trianglesPointsCount > 2) {
			trianglesBuffer.flip();
			GL20.glVertexAttribPointer(positionHandle, COORDS_PER_VERTEX,
					false, STRIDE, trianglesBuffer);
			GL11.glDrawArrays(GL11.GL_TRIANGLES, 0, trianglesPointsCount);
		}

		GL20.glDisableVertexAttribArray(positionHandle);

		reset();
	}

	@Override
	public int[] getBuffer() {
		return null;
	}

	@Override
	public void initBuffer() {
	}
}
