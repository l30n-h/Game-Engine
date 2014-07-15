package com.brocorporation.gameengine.elements.bodies;

import com.brocorporation.gameengine.elements.opengl.GLShape;
import com.brocorporation.gameengine.utils.AffineTransform;
import com.brocorporation.gameengine.utils.Vector3f;

public class Body implements AffineTransform.IAffineTransform {

	protected GLShape glShape;

	protected final Vector3f position = new Vector3f();
	protected final AffineTransform affineTransform = new AffineTransform();
	protected boolean visible = true;

	public Body() {
	}

	public void setPosition(final float x, final float y, final float z) {
		position.set(x, y, z);
	}

	public Vector3f getPosition() {
		return position;
	}

	public void setShape(final GLShape shape) {
		glShape = shape;
	}

	public GLShape getShape() {
		return glShape;
	}

	public void isVisible(boolean pVisible) {
		visible = pVisible;
	}

	public boolean isVisible() {
		return visible;
	}

	@Override
	public AffineTransform getAffineTransform() {
		affineTransform.setTranslation(getPosition());
		return affineTransform;
	}

}
