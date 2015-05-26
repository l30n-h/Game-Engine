package com.brocorporation.gameengine.elements.opengl;

import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.nio.ShortBuffer;
import java.util.ArrayList;
import java.util.List;

import org.lwjgl.BufferUtils;
import org.lwjgl.opengl.GL11;
import org.lwjgl.opengl.GL15;
import org.lwjgl.opengl.GL20;

import com.brocorporation.gameengine.elements.collision.Frustum;
import com.brocorporation.gameengine.parser.WavefrontObject;
import com.brocorporation.gameengine.utils.AffineTransform;
import com.brocorporation.gameengine.utils.Matrix;
import com.brocorporation.gameengine.utils.MatrixExt;

public class MainShape extends GLShape {

	public final static byte VBO = 0;
	public final static byte IBO = 1;
	public final static byte COORDS_PER_VERTEX = 3;
	public final static byte NORMALS_PER_VERTEX = 3;
	public final static byte TEXTURE_PER_VERTEX = 2;
	public final byte STRIDE = (COORDS_PER_VERTEX + TEXTURE_PER_VERTEX + NORMALS_PER_VERTEX)
			* BYTES_PER_FLOAT;

	private final static float[] normalMatrix = new float[9];

	private final FloatBuffer vBuffer;
	private final ShortBuffer iBuffer;
	private final int buffer[] = new int[2];

	private final GLTexture glTexture;
	private final GLMesh[] meshes;

	protected static GLMaterial lastMaterial;

	protected final BlinnPhongShader shader;
	protected int drawMode = GL11.GL_TRIANGLES;

	protected final List<AffineTransform> transformableList = new ArrayList<AffineTransform>();

	public MainShape(final BlinnPhongShader pGLShader,
			final GLTexture pGLTexture, final WavefrontObject pWavefrontObject) {
		shader = pGLShader;
		glTexture = pGLTexture;
		meshes = pWavefrontObject.getMesh();
		vBuffer = pWavefrontObject.getVBO();
		iBuffer = pWavefrontObject.getIBO();
	}

	@Override
	public void initBuffer() {
		final IntBuffer intBuffer = BufferUtils.createIntBuffer(2);
		GL15.glGenBuffers(intBuffer);
		buffer[VBO] = intBuffer.get(VBO);
		buffer[IBO] = intBuffer.get(IBO);
		GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, buffer[VBO]);
		GL15.glBufferData(GL15.GL_ARRAY_BUFFER, vBuffer, GL15.GL_STATIC_DRAW);
		GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, 0);
		GL15.glBindBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, buffer[IBO]);
		GL15.glBufferData(GL15.GL_ELEMENT_ARRAY_BUFFER, iBuffer,
				GL15.GL_STATIC_DRAW);
		GL15.glBindBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, 0);
	}

	Frustum f;

	public void setFrustum(Frustum pF) {
		f = pF;
	}
	
	public void drawWireframe(boolean t){
		drawMode = t?GL11.GL_LINES:GL11.GL_TRIANGLES;
	}

	public void render(final AffineTransform b) {
		transformableList.add(b);
	}

	public void render(final float[] modelMatrix, final float[] viewMatrix,
			final float[] projectionMatrix, final float[] mvpMatrix) {
		final int[] aHandle = shader.aHandle;
		final int positionHandle = aHandle[BlinnPhongShader.a_Position];
		final int uvHandle = aHandle[BlinnPhongShader.a_UV];
		final int normalHandle = aHandle[BlinnPhongShader.a_Normal];

		Matrix.multiplyMM(mvpMatrix, 0, viewMatrix, 0, modelMatrix, 0);
		shader.setMVMatrix(mvpMatrix);
		MatrixExt.castM3(normalMatrix, mvpMatrix);
		MatrixExt.invertM3(normalMatrix, normalMatrix);
		MatrixExt.transposeM3(normalMatrix, normalMatrix);

		shader.setNMatrix(normalMatrix);
		Matrix.multiplyMM(mvpMatrix, 0, projectionMatrix, 0, mvpMatrix, 0);
		shader.setMVPMatrix(mvpMatrix);

		GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, buffer[VBO]);
		GL20.glEnableVertexAttribArray(positionHandle);
		GL20.glEnableVertexAttribArray(uvHandle);
		GL20.glEnableVertexAttribArray(normalHandle);

		GL20.glVertexAttribPointer(positionHandle, COORDS_PER_VERTEX,
				GL11.GL_FLOAT, false, STRIDE, 0);
		GL20.glVertexAttribPointer(uvHandle, TEXTURE_PER_VERTEX, GL11.GL_FLOAT,
				false, STRIDE, 12);
		GL20.glVertexAttribPointer(normalHandle, NORMALS_PER_VERTEX,
				GL11.GL_FLOAT, false, STRIDE, 20);

		GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, 0);
		GL15.glBindBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, buffer[IBO]);
		render();
		GL15.glBindBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, 0);
		GL20.glDisableVertexAttribArray(positionHandle);
		GL20.glDisableVertexAttribArray(uvHandle);
		GL20.glDisableVertexAttribArray(normalHandle);
	}

	public void render(final float[] viewMatrix,
			final float[] projectionMatrix, final float[] mvpMatrix) {
		if (!transformableList.isEmpty()) {
			final int[] aHandle = shader.aHandle;
			final int positionHandle = aHandle[BlinnPhongShader.a_Position];
			final int uvHandle = aHandle[BlinnPhongShader.a_UV];
			final int normalHandle = aHandle[BlinnPhongShader.a_Normal];

			GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, buffer[VBO]);
			GL20.glEnableVertexAttribArray(positionHandle);
			GL20.glEnableVertexAttribArray(uvHandle);
			GL20.glEnableVertexAttribArray(normalHandle);

			GL20.glVertexAttribPointer(positionHandle, COORDS_PER_VERTEX,
					GL11.GL_FLOAT, false, STRIDE, 0);
			GL20.glVertexAttribPointer(uvHandle, TEXTURE_PER_VERTEX,
					GL11.GL_FLOAT, false, STRIDE, 12);
			GL20.glVertexAttribPointer(normalHandle, NORMALS_PER_VERTEX,
					GL11.GL_FLOAT, false, STRIDE, 20);

			GL15.glBindBuffer(GL15.GL_ARRAY_BUFFER, 0);
			GL15.glBindBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, buffer[IBO]);

			for (final AffineTransform b : transformableList) {
				b.getTransformationMatrix(mvpMatrix);
				Matrix.multiplyMM(mvpMatrix, 0, viewMatrix, 0, mvpMatrix, 0);
				shader.setMVMatrix(mvpMatrix);
				MatrixExt.castM3(normalMatrix, mvpMatrix);
				MatrixExt.invertM3(normalMatrix, normalMatrix);
				MatrixExt.transposeM3(normalMatrix, normalMatrix);

				shader.setNMatrix(normalMatrix);
				Matrix.multiplyMM(mvpMatrix, 0, projectionMatrix, 0, mvpMatrix,
						0);
				shader.setMVPMatrix(mvpMatrix);
				render();
			}
			GL15.glBindBuffer(GL15.GL_ELEMENT_ARRAY_BUFFER, 0);
			GL20.glDisableVertexAttribArray(positionHandle);
			GL20.glDisableVertexAttribArray(uvHandle);
			GL20.glDisableVertexAttribArray(normalHandle);

			transformableList.clear();
		}
	}

	protected void render() {
		boolean draw = false;
		int offset = 0;
		int indiceslength = 0;
		for (final GLMesh mesh : meshes) {
			if (f == null || f.intersects(mesh.getAABB())) {
				final GLMaterial material = mesh.getMaterial();
				if (lastMaterial != material) {
					lastMaterial = material;
					if (draw) {
						GL11.glDrawElements(drawMode, indiceslength,
								GL11.GL_UNSIGNED_SHORT, offset
										* BYTES_PER_SHORT);
						offset += indiceslength;
						indiceslength = 0;
					}
					shader.setMaterialAmbient(material.getAmbientColor());
					shader.setMaterialDiffuse(material.getDiffuseColor());
					shader.setMaterialSpecular(material.getSpecularColor());
					shader.setMaterialShininess(material.getShininess());
				}
				if (mesh.hasUVs()) {
					final String texture = material.getTexture();
					if (texture != null) {
						glTexture.bindTexture(texture);
						shader.setMaterialHasTexture(true);
					} else {
						shader.setMaterialHasTexture(false);
					}
				} else {
					shader.setMaterialHasTexture(false);
				}
				draw = true;
			} else {
				if (draw) {
					GL11.glDrawElements(drawMode, indiceslength,
							GL11.GL_UNSIGNED_SHORT, offset * BYTES_PER_SHORT);
					offset += indiceslength;
					indiceslength = 0;
					draw = false;
				}
			}
			indiceslength += mesh.getIndicesLength();
		}
		if (draw) {
			GL11.glDrawElements(drawMode, indiceslength,
					GL11.GL_UNSIGNED_SHORT, offset * BYTES_PER_SHORT);
		}
	}

	@Override
	public int[] getBuffer() {
		return buffer;
	}

}