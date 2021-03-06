package com.brocorporation.gameengine.utils;

public class AffineTransform {

	protected Vector3f translation = new Vector3f();
	protected Quaternion orientation = new Quaternion();

	public AffineTransform() {

	}

	public void setTranslation(float x, float y, float z) {
		translation.set(x, y, z);
	}

	public void setTranslation(Vector3f pTranslation) {
		translation.set(pTranslation);
	}

	public void translate(float x, float y, float z) {
		translation.add(x, y, z);
	}

	public void translate(Vector3f pTranslation) {
		translation.add(pTranslation);
	}

	public void setRotation(float x, float y, float z) {
		orientation.getQuaternionEuler(x, y, z);
	}

	public void setRotation(Vector3f pRotation) {
		orientation.getQuaternionEuler(pRotation);
	}

	public void rotate(float x, float y, float z) {
		orientation.addRotationEuler(x, y, z);
	}

	public void rotate(Vector3f pRotation) {
		orientation.addRotationEuler(pRotation);
	}

	public Vector3f getTranslation() {
		return translation;
	}
	
	public Quaternion getOrientation() {
		return orientation;
	}

	public Vector3f toWorld(Vector3f result, Vector3f vector) {
		return orientation.rotateV(result, vector).add(translation);
	}

	public Vector3f toLocal(Vector3f result, Vector3f vector) {
		return orientation.rotateInverseV(result,
				result.setSubtract(vector, translation));
	}

	public float[] getTransformationMatrix(float[] result) {
		MatrixExt.setIdentityM(result, 0);
		MatrixExt.translateM(result, 0, translation.x, translation.y,
				translation.z);
		orientation.multiplyMQ(result, 0, result, 0);
		return result;
	}

	public interface IAffineTransform {
		public AffineTransform getAffineTransform();
	}
}
