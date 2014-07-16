package com.brocorporation.gameengine.elements.opengl;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.nio.ShortBuffer;

import org.lwjgl.BufferUtils;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL15;
import org.lwjgl.opengl.GL20;

import com.brocorporation.gameengine.utils.Vector3f;
import com.brocorporation.gameengine.utils.Vector4f;

public class FontShape extends GLShape {
	protected final static byte COORDS_PER_POSITION = 2;
	protected final static byte COORDS_PER_TEXTURE = 2;
	protected final static byte COORDS_PER_UNIT = COORDS_PER_POSITION
			+ COORDS_PER_TEXTURE;
	protected final static byte STRIDE = COORDS_PER_UNIT * BYTES_PER_FLOAT;
	protected final static byte EDGES_PER_CHAR = 4;
	protected final static byte VERTICES_PER_CHAR = EDGES_PER_CHAR
			* COORDS_PER_UNIT;
	protected final static byte INDICES_PER_CHAR = 6;
	protected final static byte BYTES_PER_VERTICES_PER_CHAR = VERTICES_PER_CHAR
			* BYTES_PER_FLOAT;
	protected final static byte BYTES_PER_INDICES_PER_CHAR = INDICES_PER_CHAR
			* BYTES_PER_SHORT;
	protected final byte VBO = 0;
	protected final byte IBO = 1;

	protected int glyphWidth, glyphHeight, maxChars, columns = 1, count;
	protected float startX, startY, stepX, stepY, scaledGlyphWidth,
			scaledGlyphHeight;
	protected String texture;
	protected final GLTexture glTexture;
	protected Vector3f position = new Vector3f();
	protected Vector4f color = new Vector4f(0, 0, 0, 1);
	protected float size = 1;
	protected boolean positionChanged = true, colorChanged = true,
			sizeChanged = true;
	protected final FloatBuffer vBuffer;
	protected final ShortBuffer iBuffer;
	protected final int buffer[] = new int[2];
	protected final FontShader shader;

	public FontShape(final FontShader glShader, final GLTexture pGLTexture) {
		shader = glShader;
		glTexture = pGLTexture;
		maxChars = 200;
		vBuffer = ByteBuffer
				.allocateDirect(maxChars * BYTES_PER_VERTICES_PER_CHAR)
				.order(ByteOrder.nativeOrder()).asFloatBuffer();

		iBuffer = ByteBuffer
				.allocateDirect(maxChars * BYTES_PER_INDICES_PER_CHAR)
				.order(ByteOrder.nativeOrder()).asShortBuffer();
	}

	public void initBuffer() {
		final IntBuffer intBuffer = BufferUtils.createIntBuffer(2);
		GL15.glGenBuffers(intBuffer);
		buffer[VBO] = intBuffer.get(VBO);
		buffer[IBO] = intBuffer.get(IBO);
		GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, buffer[VBO]);
		GL15.glBufferData(GL15.GL_ARRAY_BUFFER, vBuffer, GL15.GL_DYNAMIC_DRAW);
		GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, 0);
		GL15.glBindBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, buffer[IBO]);
		GL15.glBufferData(GL15.GL_ELEMENT_ARRAY_BUFFER, iBuffer,
				GL15.GL_DYNAMIC_DRAW);
		GL15.glBindBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, 0);
	}

	public void setTexture(final String pTexture, final int pStartX,
			final int pStartY, final int pGlyphWidth, final int pGlyphHeight,
			final int pPaddingX, final int pPaddingY, final int pSize) {
		texture = pTexture;
		glyphWidth = pGlyphWidth;
		glyphHeight = pGlyphHeight;
		final int sX = glyphWidth + pPaddingX;
		columns = (pSize - pStartX + pPaddingX) / sX;
		final float inverseTexureSize = 1F / pSize;
		startX = pStartX * inverseTexureSize;
		startY = pStartY * inverseTexureSize;
		stepX = sX * inverseTexureSize;
		stepY = (glyphHeight + pPaddingY) * inverseTexureSize;
		scaledGlyphWidth = glyphWidth * inverseTexureSize;
		scaledGlyphHeight = glyphHeight * inverseTexureSize;
	}

	public void setPosition(final float pX, final float pY, final float pZ) {
		position.set(pX, pY, pZ);
		positionChanged = true;
	}

	public void setColor(final float pR, final float pG, final float pB,
			final float pA) {
		color.set(pR, pG, pB, pA);
		colorChanged = true;
	}

	public void setSize(final float s) {
		size = s / glyphHeight;
		sizeChanged = true;
	}

	public void setText(final String text, final int offset) {
		final int length = text.length();
		count = Math.max(length, length + offset);
		float positionX = offset * glyphWidth;
		float positionY = 0;
		short indicesOffset = (short) (offset * EDGES_PER_CHAR);
		vBuffer.clear();
		iBuffer.clear();
		for (int i = 0; i < length; i++) {
			final char c = text.charAt(i);
			if (c == '\n') {
				// positionX = offset * glyphWidth;
				// positionY -= glyphHeight;
				continue;
			}
			final int col;
			final int row;
			if (c >= ' ' && c <= '~') {
				final int charIndex = c - ' ';
				col = charIndex % columns;
				row = charIndex / columns;
			} else {
				col = columns - 1;
				row = col;
			}

			final float s1 = (startX + col * stepX);
			final float t1 = (startY + row * stepY);
			final float s2 = s1 + scaledGlyphWidth;
			final float t2 = t1 + scaledGlyphHeight;
			final float x2 = positionX + glyphWidth;
			final float y2 = positionY - glyphHeight;
			vBuffer.put(positionX);
			vBuffer.put(positionY);
			vBuffer.put(s1);
			vBuffer.put(t1);

			vBuffer.put(positionX);
			vBuffer.put(y2);
			vBuffer.put(s1);
			vBuffer.put(t2);

			vBuffer.put(x2);
			vBuffer.put(y2);
			vBuffer.put(s2);
			vBuffer.put(t2);

			vBuffer.put(x2);
			vBuffer.put(positionY);
			vBuffer.put(s2);
			vBuffer.put(t1);

			positionX = x2;

			final short i2 = (short) (indicesOffset + 2);
			iBuffer.put(indicesOffset);
			iBuffer.put((short) (indicesOffset + 1));
			iBuffer.put(i2);
			iBuffer.put(indicesOffset);
			iBuffer.put(i2);
			iBuffer.put((short) (indicesOffset + 3));
			indicesOffset += EDGES_PER_CHAR;
		}
		vBuffer.flip();
		iBuffer.flip();

		GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, buffer[VBO]);
		GL15.glBufferSubData(GL15.GL_ARRAY_BUFFER, offset
				* BYTES_PER_VERTICES_PER_CHAR, vBuffer);
		GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, 0);
		GL15.glBindBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, buffer[IBO]);
		GL15.glBufferSubData(GL15.GL_ELEMENT_ARRAY_BUFFER, offset
				* BYTES_PER_INDICES_PER_CHAR, iBuffer);
		GL15.glBindBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, 0);
	}

	public void setLength(int count) {
		count = Math.max(0, Math.min(count, maxChars));
	}

	public void render() {
		final int[] aHandle = shader.aHandle;
		if (positionChanged) {
			shader.setPosition(position);
			positionChanged = false;
		}
		if (colorChanged) {
			shader.setColor(color);
			colorChanged = false;
		}
		if (sizeChanged) {
			shader.setSize(size);
			sizeChanged = false;
		}
		glTexture.bindTexture(texture);
		final int positionHandle = aHandle[FontShader.a_Position];
		final int uvHandle = aHandle[FontShader.a_UV];
		GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, buffer[VBO]);
		GL20.glEnableVertexAttribArray(positionHandle);
		GL20.glEnableVertexAttribArray(uvHandle);
		GL20.glVertexAttribPointer(positionHandle, COORDS_PER_POSITION,
				GL11.GL_FLOAT, false, STRIDE, 0);
		GL20.glVertexAttribPointer(uvHandle, COORDS_PER_TEXTURE, GL11.GL_FLOAT,
				false, STRIDE, 8);
		GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, 0);
		GL15.glBindBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, buffer[IBO]);
		GL11.glDrawElements(GL11.GL_TRIANGLES, count * INDICES_PER_CHAR,
				GL11.GL_UNSIGNED_SHORT, 0);
		GL15.glBindBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, 0);
		GL20.glDisableVertexAttribArray(positionHandle);
		GL20.glDisableVertexAttribArray(uvHandle);
	}

	public float getWidth() {
		return count * glyphWidth * size;
	}

	@Override
	public int[] getBuffer() {
		return buffer;
	}
}
