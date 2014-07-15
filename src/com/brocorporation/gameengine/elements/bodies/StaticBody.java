package com.brocorporation.gameengine.elements.bodies;

import com.brocorporation.gameengine.IUpdateInfo;
import com.brocorporation.gameengine.elements.collision.Collidable;
import com.brocorporation.gameengine.elements.collision.IShape;
import com.brocorporation.gameengine.elements.collision.Material;
import com.brocorporation.gameengine.elements.opengl.GLShape;
import com.brocorporation.gameengine.utils.AffineTransform;
import com.brocorporation.gameengine.utils.Quaternion;
import com.brocorporation.gameengine.utils.Vector3f;

public class StaticBody extends Collidable implements
		AffineTransform.IAffineTransform {

	protected final AffineTransform affineTransform = new AffineTransform();
	protected boolean updateTranslation;
	protected boolean updateOrientation;
	protected Material material = Material.DEFAULT;

	protected GLShape glShape;// TODO

	public void setGLShape(final GLShape shape) {
		glShape = shape;
	}

	public GLShape getGLShape() {
		return glShape;
	}

	public StaticBody(final IShape shape) {
		super(shape);
	}

	public void setMaterial(final Material m) {
		material = m;
	}

	public Material getMaterial() {
		return material;
	}

	public void setPosition(final float x, final float y, final float z) {
		affineTransform.setTranslation(x, y, z);
		updateTranslation = true;
	}

	public Vector3f getPosition() {
		return shape.getPosition();
	}

	public void setRotation(final float x, final float y, final float z) {
		affineTransform.setRotation(x, y, z);
		updateOrientation = true;
	}
	

	public Quaternion getOrientation() {
		return affineTransform.getOrientation();
	}

	public void rotate(final float degreeX, final float degreeY,
			final float degreeZ) {
		if (degreeX != 0 || degreeY != 0 || degreeZ != 0) {
			affineTransform.rotate(degreeX, degreeY, degreeZ);
			updateOrientation = true;
		}
	}

	public void prepareUpdatePosition(final IUpdateInfo uInfo) {

	}

	public void updatePosition(final IUpdateInfo uInfo) {
	}

	public void updateBounds() {
		shape.updateBounds(getAffineTransform(), updateTranslation,
				updateOrientation);
		updateTranslation = false;
		updateOrientation = false;
	}

	@Override
	public AffineTransform getAffineTransform() {
		return affineTransform;
	};
}
