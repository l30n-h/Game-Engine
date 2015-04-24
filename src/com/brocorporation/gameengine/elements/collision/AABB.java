package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.utils.AffineTransform;
import com.brocorporation.gameengine.utils.Quaternion;
import com.brocorporation.gameengine.utils.Vector3f;

public class AABB implements IShape {

	protected final static Vector3f temp = new Vector3f();

	protected final Vector3f baseHalfsize = new Vector3f();

	protected final Vector3f position = new Vector3f();
	protected final Vector3f halfsize = new Vector3f();
	protected boolean hasChanged;
	protected byte frustumType;

	public AABB() {
	}

	public AABB(float hx, float hy, float hz) {
		setHalfsize(hx, hy, hz);
	}

	@Override
	public Vector3f getPosition() {
		return position;
	}

	@Override
	public Vector3f getMaxAlongDirection(final Vector3f result,
			final Vector3f dir) {
		final Vector3f h1 = getHalfsize();
		final Vector3f p1 = getPosition();
		return result.set(dir.x >= 0 ? p1.x + h1.x : p1.x - h1.x,
				dir.y >= 0 ? p1.y + h1.y : p1.y - h1.y, dir.z >= 0 ? p1.z
						+ h1.z : p1.z - h1.z);
	}

	@Override
	public Vector3f getMinAlongDirection(final Vector3f result,
			final Vector3f dir) {
		final Vector3f h1 = getHalfsize();
		final Vector3f p1 = getPosition();
		return result.set(dir.x < 0 ? p1.x + h1.x : p1.x - h1.x,
				dir.y < 0 ? p1.y + h1.y : p1.y - h1.y, dir.z < 0 ? p1.z + h1.z
						: p1.z - h1.z);
	}

	@Override
	public AABB getAABB() {
		return this;
	}

	@Override
	public void updateBounds(AffineTransform affineTransform,
			boolean updateTranslation, boolean updateOrientation) {
		hasChanged = updateTranslation || updateOrientation;
		if (hasChanged) {
			if (updateOrientation) {
				rotate(affineTransform.getOrientation());
			}
			if (updateTranslation) {
				position.set(affineTransform.getTranslation());
			}
		}
	}

	@Override
	public void setFrustumType(byte type) {
		frustumType = type;
	}

	@Override
	public byte getFrustumType() {
		return frustumType;
	}

	@Override
	public boolean hasChanged() {
		return hasChanged;
	}

	@Override
	public void getInverseInertiaTensor(float[] inverseInertiaTensor,
			float inverseMass) {
		final float ww = baseHalfsize.x * baseHalfsize.x;
		final float hh = baseHalfsize.y * baseHalfsize.y;
		final float dd = baseHalfsize.z * baseHalfsize.z;
		final float s = 3 * inverseMass;
		// inverseInertiaTensor.set(s / (hh + dd), s / (ww + dd), s / (ww +
		// hh));
		inverseInertiaTensor[1] = inverseInertiaTensor[2] = inverseInertiaTensor[3] = inverseInertiaTensor[5] = inverseInertiaTensor[6] = inverseInertiaTensor[7] = 0;
		inverseInertiaTensor[0] = s / (hh + dd);
		inverseInertiaTensor[4] = s / (ww + dd);
		inverseInertiaTensor[8] = s / (ww + hh);
	}

	public Vector3f getBaseHalfsize() {
		return baseHalfsize;
	}

	public Vector3f getHalfsize() {
		return halfsize;
	}

	public void setHalfsize(float x, float y, float z) {
		baseHalfsize.set(x, y, z);
		halfsize.set(x, y, z);
	}

	public void setHalfsize(Vector3f v) {
		baseHalfsize.set(v);
		halfsize.set(v);
	}

	public void setPosition(float x, float y, float z) {
		position.set(x, y, z);
	}

	public void setPosition(Vector3f v) {
		position.set(v);
	}

	public void rotate(Quaternion q) {
		q.rotateHalfsize(halfsize, baseHalfsize);
	}

	public boolean contains(AABB aabb) {
		final Vector3f h1 = getHalfsize();
		final Vector3f h2 = aabb.getHalfsize();
		final Vector3f p1, p2;
		return h1.x >= h2.x
				&& h1.y >= h2.y
				&& h1.z >= h2.z
				&& Math.abs((p2 = aabb.getPosition()).x
						- (p1 = getPosition()).x) <= h1.x - h2.x
				&& Math.abs(p2.y - p1.y) <= h1.y - h2.y
				&& Math.abs(p2.z - p1.z) <= h1.z - h2.z;
	}

	public boolean contains(Vector3f vertex) {
		final Vector3f h1 = getHalfsize();
		final Vector3f p1 = getPosition();
		return Math.abs(vertex.x - p1.x) <= h1.x
				&& Math.abs(vertex.y - p1.y) <= h1.y
				&& Math.abs(vertex.z - p1.z) <= h1.z;
	}

	public boolean intersects(AABB aabb) {
		final Vector3f h1 = getHalfsize();
		final Vector3f h2 = aabb.getHalfsize();
		final Vector3f p1 = getPosition();
		final Vector3f p2 = aabb.getPosition();
		return Math.abs(p2.x - p1.x) < h2.x + h1.x
				&& Math.abs(p2.y - p1.y) < h2.y + h1.y
				&& Math.abs(p2.z - p1.z) < h2.z + h1.z;
	}

	public boolean intersectsRay(RaycastHit hit, Vector3f fromPoint,
			Vector3f dir, float min, float max) {
		final Vector3f p1 = getPosition();
		if (contains(fromPoint)) {
			hit.getPoint().set(p1);
			hit.setScalar(0);
			return min <= 0;
		}
		final Vector3f h1 = getHalfsize();
		float tmin, tmax, tminyz, tmaxyz;
		final float dirfracX = 1.0f / dir.x;
		final float dx = p1.x - fromPoint.x;
		if (dir.x >= 0) {
			tmin = (dx - h1.x) * dirfracX;
			tmax = (dx + h1.x) * dirfracX;
		} else {
			tmin = (dx + h1.x) * dirfracX;
			tmax = (dx - h1.x) * dirfracX;
		}
		final float dirfracY = 1.0f / dir.y;
		final float dy = p1.y - fromPoint.y;
		if (dir.y >= 0) {
			tminyz = (dy - h1.y) * dirfracY;
			tmaxyz = (dy + h1.y) * dirfracY;
		} else {
			tminyz = (dy + h1.y) * dirfracY;
			tmaxyz = (dy - h1.y) * dirfracY;
		}
		if ((tmin > tmaxyz) || (tminyz > tmax))
			return false;
		if (tminyz > tmin)
			tmin = tminyz;
		if (tmaxyz < tmax)
			tmax = tmaxyz;
		final float dirfracZ = 1.0f / dir.z;
		final float dz = p1.z - fromPoint.z;
		if (dir.z >= 0) {
			tminyz = (dz - h1.z) * dirfracZ;
			tmaxyz = (dz + h1.z) * dirfracZ;
		} else {
			tminyz = (dz + h1.z) * dirfracZ;
			tmaxyz = (dz - h1.z) * dirfracZ;
		}
		if ((tmin > tmaxyz) || (tminyz > tmax))
			return false;
		if (tminyz > tmin)
			tmin = tminyz;
		if (tmaxyz < tmax)
			tmax = tmaxyz;
		if (!Float.isInfinite(tmin) && tmin >= min && tmin <= max) {
			final Vector3f v = hit.getPoint();
			v.setAddScaled(fromPoint, dir, tmin);
			hit.setScalar(tmin);
			return true;
		}
		return false;
	}

	public boolean intersectsRay(RaycastHit hit, Vector3f halfsizeOffset,
			Vector3f fromPoint, Vector3f dir, float min, float max) {
		temp.setAdd(getHalfsize(), halfsizeOffset);
		final Vector3f p1 = getPosition();
		if (AABB.contains(position, temp, fromPoint)) {
			hit.getPoint().set(p1);
			hit.setScalar(0);
			return min <= 0;
		}
		float tmin, tmax, tminyz, tmaxyz;
		final float dirfracX = 1.0f / dir.x;
		final float dx = p1.x - fromPoint.x;
		if (dir.x >= 0) {
			tmin = (dx - temp.x) * dirfracX;
			tmax = (dx + temp.x) * dirfracX;
		} else {
			tmin = (dx + temp.x) * dirfracX;
			tmax = (dx - temp.x) * dirfracX;
		}
		final float dirfracY = 1.0f / dir.y;
		final float dy = p1.y - fromPoint.y;
		if (dir.y >= 0) {
			tminyz = (dy - temp.y) * dirfracY;
			tmaxyz = (dy + temp.y) * dirfracY;
		} else {
			tminyz = (dy + temp.y) * dirfracY;
			tmaxyz = (dy - temp.y) * dirfracY;
		}
		if ((tmin > tmaxyz) || (tminyz > tmax))
			return false;
		if (tminyz > tmin)
			tmin = tminyz;
		if (tmaxyz < tmax)
			tmax = tmaxyz;
		final float dirfracZ = 1.0f / dir.z;
		final float dz = p1.z - fromPoint.z;
		if (dir.z >= 0) {
			tminyz = (dz - temp.z) * dirfracZ;
			tmaxyz = (dz + temp.z) * dirfracZ;
		} else {
			tminyz = (dz + temp.z) * dirfracZ;
			tmaxyz = (dz - temp.z) * dirfracZ;
		}
		if ((tmin > tmaxyz) || (tminyz > tmax))
			return false;
		if (tminyz > tmin)
			tmin = tminyz;
		if (tmaxyz < tmax)
			tmax = tmaxyz;
		if (!Float.isInfinite(tmin) && tmin >= min && tmin <= max) {
			final Vector3f v = hit.getPoint();
			v.setAddScaled(fromPoint, dir, tmin);
			hit.setScalar(tmin);
			return true;
		}
		return false;
	}

	public static boolean contains(Vector3f position1, Vector3f halfsize1,
			Vector3f position2, Vector3f halfsize2) {
		return halfsize1.x >= halfsize2.x
				&& halfsize1.y >= halfsize2.y
				&& halfsize1.z >= halfsize2.z
				&& Math.abs(position2.x - position1.x) <= halfsize1.x
						- halfsize2.x
				&& Math.abs(position2.y - position1.y) <= halfsize1.y
						- halfsize2.y
				&& Math.abs(position2.z - position1.z) <= halfsize1.z
						- halfsize2.z;
	}

	public static boolean contains(Vector3f position, Vector3f halfsize,
			Vector3f vertex) {
		return Math.abs(vertex.x - position.x) <= halfsize.x
				&& Math.abs(vertex.y - position.y) <= halfsize.y
				&& Math.abs(vertex.z - position.z) <= halfsize.z;
	}

	public static boolean intersects(Vector3f position1, Vector3f halfsize1,
			Vector3f position2, Vector3f halfsize2) {
		return Math.abs(position2.x - position1.x) < halfsize2.x + halfsize1.x
				&& Math.abs(position2.y - position1.y) < halfsize2.y
						+ halfsize1.y
				&& Math.abs(position2.z - position1.z) < halfsize2.z
						+ halfsize1.z;
	}

	public static boolean intersectsRay(RaycastHit hit, Vector3f position,
			Vector3f halfsize, Vector3f fromPoint, Vector3f dir, float min,
			float max) {
		if (AABB.contains(position, halfsize, fromPoint)) {
			hit.getPoint().set(position);
			hit.setScalar(0);
			return min <= 0;
		}
		float tmin, tmax, tminyz, tmaxyz;
		final float dirfracX = 1.0f / dir.x;
		final float dx = position.x - fromPoint.x;
		if (dir.x >= 0) {
			tmin = (dx - halfsize.x) * dirfracX;
			tmax = (dx + halfsize.x) * dirfracX;
		} else {
			tmin = (dx + halfsize.x) * dirfracX;
			tmax = (dx - halfsize.x) * dirfracX;
		}
		final float dirfracY = 1.0f / dir.y;
		final float dy = position.y - fromPoint.y;
		if (dir.y >= 0) {
			tminyz = (dy - halfsize.y) * dirfracY;
			tmaxyz = (dy + halfsize.y) * dirfracY;
		} else {
			tminyz = (dy + halfsize.y) * dirfracY;
			tmaxyz = (dy - halfsize.y) * dirfracY;
		}
		if ((tmin > tmaxyz) || (tminyz > tmax))
			return false;
		if (tminyz > tmin)
			tmin = tminyz;
		if (tmaxyz < tmax)
			tmax = tmaxyz;
		final float dirfracZ = 1.0f / dir.z;
		final float dz = position.z - fromPoint.z;
		if (dir.z >= 0) {
			tminyz = (dz - halfsize.z) * dirfracZ;
			tmaxyz = (dz + halfsize.z) * dirfracZ;
		} else {
			tminyz = (dz + halfsize.z) * dirfracZ;
			tmaxyz = (dz - halfsize.z) * dirfracZ;
		}
		if ((tmin > tmaxyz) || (tminyz > tmax))
			return false;
		if (tminyz > tmin)
			tmin = tminyz;
		if (tmaxyz < tmax)
			tmax = tmaxyz;
		if (!Float.isInfinite(tmin) && tmin >= min && tmin <= max) {
			final Vector3f v = hit.getPoint();
			v.setAddScaled(fromPoint, dir, tmin);
			hit.setScalar(tmin);
			return true;
		}
		return false;
	}

	public static boolean intersectsRay(RaycastHit hit, Vector3f position,
			Vector3f halfsize, Vector3f halfsizeOffset, Vector3f fromPoint,
			Vector3f dir, float min, float max) {
		temp.setAdd(halfsize, halfsizeOffset);
		if (AABB.contains(position, temp, fromPoint)) {
			hit.getPoint().set(position);
			hit.setScalar(0);
			return min <= 0;
		}
		float tmin, tmax, tminyz, tmaxyz;
		final float dirfracX = 1.0f / dir.x;
		final float dx = position.x - fromPoint.x;
		if (dir.x >= 0) {
			tmin = (dx - temp.x) * dirfracX;
			tmax = (dx + temp.x) * dirfracX;
		} else {
			tmin = (dx + temp.x) * dirfracX;
			tmax = (dx - temp.x) * dirfracX;
		}
		final float dirfracY = 1.0f / dir.y;
		final float dy = position.y - fromPoint.y;
		if (dir.y >= 0) {
			tminyz = (dy - temp.y) * dirfracY;
			tmaxyz = (dy + temp.y) * dirfracY;
		} else {
			tminyz = (dy + temp.y) * dirfracY;
			tmaxyz = (dy - temp.y) * dirfracY;
		}
		if ((tmin > tmaxyz) || (tminyz > tmax))
			return false;
		if (tminyz > tmin)
			tmin = tminyz;
		if (tmaxyz < tmax)
			tmax = tmaxyz;
		final float dirfracZ = 1.0f / dir.z;
		final float dz = position.z - fromPoint.z;
		if (dir.z >= 0) {
			tminyz = (dz - temp.z) * dirfracZ;
			tmaxyz = (dz + temp.z) * dirfracZ;
		} else {
			tminyz = (dz + temp.z) * dirfracZ;
			tmaxyz = (dz - temp.z) * dirfracZ;
		}
		if ((tmin > tmaxyz) || (tminyz > tmax))
			return false;
		if (tminyz > tmin)
			tmin = tminyz;
		if (tmaxyz < tmax)
			tmax = tmaxyz;
		if (!Float.isInfinite(tmin) && tmin >= min && tmin <= max) {
			final Vector3f v = hit.getPoint();
			v.setAddScaled(fromPoint, dir, tmin);
			hit.setScalar(tmin);
			return true;
		}
		return false;
	}

	public static void maxPointAlongDirection(Vector3f result,
			Vector3f position, Vector3f halfsize, Vector3f dir) {
		result.set(dir.x >= 0 ? position.x + halfsize.x : position.x
				- halfsize.x, dir.y >= 0 ? position.y + halfsize.y : position.y
				- halfsize.y, dir.z >= 0 ? position.z + halfsize.z : position.z
				- halfsize.z);
	}

	public static void minPointAlongDirection(Vector3f result,
			Vector3f position, Vector3f halfsize, Vector3f dir) {
		result.set(dir.x < 0 ? position.x + halfsize.x : position.x
				- halfsize.x, dir.y < 0 ? position.y + halfsize.y : position.y
				- halfsize.y, dir.z < 0 ? position.z + halfsize.z : position.z
				- halfsize.z);
	}

	public static boolean contains(AABB bounds1, AABB bounds2) {
		return contains(bounds1.getPosition(), bounds1.getHalfsize(),
				bounds2.getPosition(), bounds2.getHalfsize());
	}

	public static boolean contains(AABB bounds, Vector3f vertex) {
		return contains(bounds.getPosition(), bounds.getHalfsize(), vertex);
	}

	public static boolean intersects(AABB bounds1, AABB bounds2) {
		return intersects(bounds1.getPosition(), bounds1.getHalfsize(),
				bounds2.getPosition(), bounds2.getHalfsize());
	}

	public static boolean intersectsRay(RaycastHit hit, AABB bounds,
			Vector3f fromPoint, Vector3f dir, float min, float max) {
		return intersectsRay(hit, bounds.getPosition(), bounds.getHalfsize(),
				fromPoint, dir, min, max);
	}

	public static boolean intersectsRay(RaycastHit hit, AABB bounds,
			Vector3f halfsizeOffset, Vector3f fromPoint, Vector3f dir,
			float min, float max) {
		return intersectsRay(hit, bounds.getPosition(), bounds.getHalfsize(),
				halfsizeOffset, fromPoint, dir, min, max);
	}

	public static void maxPointAlongDirection(Vector3f result, AABB bounds,
			Vector3f dir) {
		maxPointAlongDirection(result, bounds.getPosition(),
				bounds.getHalfsize(), dir);
	}

	public static void minPointAlongDirection(Vector3f result, AABB bounds,
			Vector3f dir) {
		minPointAlongDirection(result, bounds.getPosition(),
				bounds.getHalfsize(), dir);
	}

	public static float getDistance(Vector3f outNormal, AABB bounds1,
			AABB bounds2) {
		final float distX = bounds2.getPosition().x - bounds1.getPosition().x;
		final float distY = bounds2.getPosition().y - bounds1.getPosition().y;
		final float distZ = bounds2.getPosition().z - bounds1.getPosition().z;
		final float absX = Math.abs(distX)
				- (bounds1.getHalfsize().x + bounds2.getHalfsize().x);
		final float absY = Math.abs(distY)
				- (bounds1.getHalfsize().y + bounds2.getHalfsize().y);
		final float absZ = Math.abs(distZ)
				- (bounds1.getHalfsize().z + bounds2.getHalfsize().z);
		outNormal.set(0, 0, 0);
		if (absX > absY) {
			if (absX > absZ) {
				outNormal.x = Math.signum(distX);
				return absX;
			}
		} else {
			if (absY > absZ) {
				outNormal.y = Math.signum(distY);
				return absY;
			}
		}
		outNormal.z = Math.signum(distZ);
		return absZ;
	}

	public static void getBounds(AABB aabb, Vector3f[] vertices) {
		if (vertices != null) {
			float minX = Float.POSITIVE_INFINITY;
			float minY = Float.POSITIVE_INFINITY;
			float minZ = Float.POSITIVE_INFINITY;
			float maxX = Float.NEGATIVE_INFINITY;
			float maxY = Float.NEGATIVE_INFINITY;
			float maxZ = Float.NEGATIVE_INFINITY;
			for (final Vector3f vertex : vertices) {
				if (vertex.x < minX) {
					minX = vertex.x;
				} else if (vertex.x > maxX) {
					maxX = vertex.x;
				}
				if (vertex.y < minY) {
					minY = vertex.y;
				} else if (vertex.y > maxY) {
					maxY = vertex.y;
				}
				if (vertex.z < minZ) {
					minZ = vertex.z;
				} else if (vertex.z > maxZ) {
					maxZ = vertex.z;
				}
			}
			final float hx = Math.abs(maxX - minX) * 0.5F;
			final float hy = Math.abs(maxY - minY) * 0.5F;
			final float hz = Math.abs(maxZ - minZ) * 0.5F;
			aabb.setPosition(maxX - hx, maxY - hy, maxZ - hz);
			aabb.setHalfsize(hx + 0.001f, hy + 0.001f, hz + 0.001f);
		}
	}

	public interface IBounds {

		public AABB getAABB();
	}
}
