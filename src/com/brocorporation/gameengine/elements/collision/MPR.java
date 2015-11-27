package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.elements.collision.Simplex.Element;
import com.brocorporation.gameengine.utils.Distance;
import com.brocorporation.gameengine.utils.Vector3f;

public class MPR {

	protected final static int MAX_ITERATIONS = 34;
	protected final static float CollideEpsilon = 1e-4f;
	protected final static float CollideEpsilon_2 = CollideEpsilon * CollideEpsilon;

	protected static Vector3f v1v0 = new Vector3f(), v2v0 = new Vector3f();
	protected final static Vector3f v2v1 = new Vector3f(), v3v1 = new Vector3f(), v4v1 = new Vector3f();
	protected final static Vector3f temp = new Vector3f();
	protected final static Simplex simplex = new Simplex();
	protected final static Element e0 = new Element();

	protected static Element e1, e2, e3, e4;

	protected static Vector3f dir = new Vector3f();

	public static boolean intersects(IShape shape1, IShape shape2) {
		initElementsFromSimplex(simplex);
		final int res = discoverPortal(shape1, shape2);
		if (res < 0)
			return false;
		if (res > 0)
			return true;
		return refinePortal(shape1, shape2);
	}

	public static boolean intersects(Contact contact, IShape shape1, IShape shape2) {
		try {
			initElementsFromSimplex(simplex);
			final int res = discoverPortal(shape1, shape2);
			if (res < 0) {
				return false;
			} else if (res == 1) {
				findPenetrationTouch(contact, shape1, shape2);
			} else if (res == 2) {
				findPenetrationSegment(contact);
			} else if (res == 0) {
				if (!refinePortal(shape1, shape2)) {
					return false;
				}
				findPenetration(contact, shape1, shape2);
			}
			return true;
		} finally {
			setDir = false;
		}
	}

	public static void initElementsFromSimplex(Simplex s) {
		e1 = s.get(0);
		e2 = s.get(1);
		e3 = s.get(2);
		e4 = s.get(3);
	}

	public final static Vector3f DIR = new Vector3f();
	public static boolean setDir = false;

	public static void setDir(boolean set) {
		setDir = set;
	}

	protected static void getOriginRayDirection(IShape shape1, IShape shape2) {

		if (setDir) {
			e0.v.set(DIR);
		} else {
			e0.v.setSubtract(e0.pA.set(shape1.getPosition()), e0.pB.set(shape2.getPosition()));
		}
		if (e0.v.isZero()) {
			e0.v.set(0.00001f, 0, 0);
		}
	}

	protected static byte discoverPortal(IShape shape1, IShape shape2) {
		getOriginRayDirection(shape1, shape2);
		MinkowskiDifference.getMinSupport(e1, shape1, shape2, e0.v);
		if (e1.v.dot(e0.v) >= 0)
			return -1;
		dir.setCross(e0.v, e1.v);
		if (dir.isZero()) {
			return 2;
		}
		MinkowskiDifference.getMaxSupport(e2, shape1, shape2, dir);
		if (e2.v.dot(dir) <= 0)
			return -1;
		v1v0.setSubtract(e1.v, e0.v);
		v2v0.setSubtract(e2.v, e0.v);
		dir.setCross(v1v0, v2v0);
		if (dir.dot(e0.v) > 0) {
			final Element e = e1;
			e1 = e2;
			e2 = e;
			dir.invert();
			Vector3f s = v1v0;
			v1v0 = v2v0;
			v2v0 = s;
		}
		while (true) {
			MinkowskiDifference.getMaxSupport(e3, shape1, shape2, dir);
			if (e3.v.dot(dir) <= 0)
				return -1;
			if (temp.setCross(e1.v, e3.v).dot(e0.v) < 0) {
				set(e2, e3);
				v2v0.setSubtract(e2.v, e0.v);
			} else if (temp.setCross(e3.v, e2.v).dot(e0.v) < 0) {
				set(e1, e3);
				v1v0.setSubtract(e1.v, e0.v);
			} else {
				return 0;
			}
			dir.setCross(v1v0, v2v0);
		}
	}

	protected static boolean refinePortal(IShape shape1, IShape shape2) {
		v2v1.setSubtract(e2.v, e1.v);
		v3v1.setSubtract(e3.v, e1.v);
		while (true) {
			dir.setCross(v2v1, v3v1);
			if (dir.dot(e1.v) >= 0)
				return true;
			MinkowskiDifference.getMaxSupport(e4, shape1, shape2, dir);
			if (dir.dot(e4.v) < 0 || portalReachTolerance(dir)) {
				return false;
			}
			expandPortal(e4);
		}
	}

	protected static void findPenetration(Contact contact, IShape shape1, IShape shape2) {
		int iterations = 0;
		while (true) {
			dir.setCross(v2v1, v3v1);
			MinkowskiDifference.getMaxSupport(e4, shape1, shape2, dir);
			if (iterations > MAX_ITERATIONS || portalReachTolerance(dir)) {
				break;
			}
			expandPortal(e4);// schwachpunkt wenn knapp falsche normale

			iterations++;
		}
		final Vector3f n = contact.getNormal();
		n.setNorm(dir);
		float d = e1.v.dot(n);
		if (d <= CollideEpsilon/* _2 */) {
			findPenetrationTouch(contact, shape1, shape2);
		} else {
			// d = (float) Math.sqrt(d);
			contact.setDistance(-d);
			// n.norm();
			findPos(contact, shape1, shape2, n);
		}
	}

	protected static void findPenetrationS(Contact c, IShape shape1, IShape shape2) {
		for (int i = 0; i < MAX_ITERATIONS; i++) {
			dir.setCross(v2v1, v3v1);
			MinkowskiDifference.getMaxSupport(e4, shape1, shape2, dir);
			if (portalReachTolerance(dir))
				break;
			expandPortal(e4);
		}
		final Vector3f n = c.getNormal();
		float d = Distance.distanceToTriangle(n, e1.v, e2.v, e3.v);
		if (d <= CollideEpsilon_2) {
			findPenetrationTouch(c, shape1, shape2);
		} else {
			d = (float) Math.sqrt(d);
			c.setDistance(-d);
			n.norm();
			findPos(c, shape1, shape2, n);
		}
		return;
	}

	protected static void findPenetrationTouch(Contact c, IShape shape1, IShape shape2) {
		c.setDistance(0);
		c.getPointA().setAdd(e1.pA, e1.pB).scale(0.5f);
		c.getPointB().set(c.getPointA());
	}

	protected static void findPenetrationSegment(Contact c) {
		c.getPointA().setAdd(e1.pA, e1.pB).scale(0.5f);
		c.getNormal().set(e1.v);
		c.setDistance(-c.getNormal().normLength());
		c.getPointB().set(c.getPointA());
	}

	static float s, t;

	private static float closestPointToTriangle(Vector3f result) {
		final Vector3f A = simplex.getV(2);
		final Vector3f B = simplex.getV(1);
		final Vector3f C = simplex.getV(0);
		Vector3f AB = v2v1;
		Vector3f AC = v3v1;
		AB.setSubtract(B, A);
		AC.setSubtract(C, A);
		final float aoab = -A.dot(AB);
		final float aoac = -A.dot(AC);
		if (aoab <= 0 && aoac <= 0) {
			simplex.removeB();
			simplex.removeC();
			result.set(A);
			return result.dot();
		}
		final float boab = -B.dot(AB);
		final float boac = -B.dot(AC);
		if (boab >= 0 && boac <= boab) {
			simplex.removeA();
			simplex.removeC();
			result.set(B);
			return result.dot();
		}
		final float vc = aoab * boac - boab * aoac;
		if (vc <= 0 && aoab >= 0 && boab <= 0) {
			simplex.removeC();
			s = aoab / (aoab - boab);
			result.setAddScaled(A, AB, s);
			return result.dot();
		}
		final float coab = -C.dot(AB);
		final float coac = -C.dot(AC);
		if (coac >= 0 && coab <= coac) {
			simplex.removeA();
			simplex.removeB();
			result.set(C);
			return result.dot();
		}
		final float vb = coab * aoac - aoab * coac;
		if (vb <= 0 && aoac >= 0 && coac <= 0) {
			simplex.removeB();
			s = aoac / (aoac - coac);
			result.setAddScaled(A, AC, s);
			return result.dot();
		}
		final float va = boab * coac - coab * boac;
		final float bacmbab, cabmcac;
		if (va <= 0 && (bacmbab = boac - boab) >= 0 && (cabmcac = coab - coac) >= 0) {
			simplex.removeA();
			s = bacmbab / (bacmbab + cabmcac);
			result.setAddScaled(B, AB.setSubtract(C, B), s);
			return result.dot();
		}
		final float denom = 1 / (va + vb + vc);
		s = vb * denom;
		t = vc * denom;
		result.setAddScaled(A, AB, s).addScaled(AC, t);
		return result.dot();
	}

	static void Barycentric(Contact c, Element e1, Element e2, Element e3) {
		float d00 = v2v1.dot();
		float d01 = v2v1.dot(v3v1);
		float d11 = v3v1.dot();
		float d20 = -v2v1.dot(e1.v);
		float d21 = -v3v1.dot(e1.v);
		float denom = d00 * d11 - d01 * d01;
		float v = (d11 * d20 - d01 * d21) / denom;
		float w = (d00 * d21 - d01 * d20) / denom;
		float u = 1.0f - v - w;
		c.getPointA().setScale(e1.pA, u).addScaled(e2.pA, v).addScaled(e3.pA, w).addScaled(c.getNormal(),
				c.getDistance() / 2);
		c.getPointB().set(c.getPointA());
		System.out.println(c.getNormal());
		System.out.println(c.getPointA());
	}

	public static void getIntersectsClosestPoints(Contact c, Simplex simplex, Vector3f normal) {
		if (simplex.size() == 1) {
			final Element e0 = simplex.get(0);
			c.getPointA().set(e0.pA);
			c.getPointB().set(c.getPointA());
		} else if (simplex.size() == 2) {
			final Element eA = simplex.get(0);
			final Element eB = simplex.get(1);
			float r2 = 1 - s;
			c.getPointA().setScale(eA.pA, s).addScaled(eB.pA, r2);
			c.getPointB().set(c.getPointA().subtractScaled(normal, 0.5f));
		} else if (simplex.size() == 3) {
			final Element eA = simplex.get(0);
			final Element eB = simplex.get(1);
			final Element eC = simplex.get(2);
			float r3 = 1 - t - s;
			c.getPointA().setScale(eA.pA, t).addScaled(eB.pA, s).addScaled(eC.pA, r3);
			c.getPointB().set(c.getPointA().subtractScaled(normal, 0.5f));
		}
	}

	protected static void findPos(Contact c, IShape shape1, IShape shape2, Vector3f normedNormal) {
		simplex.clear();
		simplex.addElement();
		simplex.addElement();
		simplex.addElement();
		c.setDistance(closestPointToTriangle(c.getNormal()));
		getIntersectsClosestPoints(c, simplex, c.getNormal());
		c.setDistance(-c.getNormal().normLength());
		if (true)
			return;

		Barycentric(c, e1, e2, e3);
		if (true)
			return;
		float b0 = v2v1.setCross(e1.v, e2.v).dot(e3.v);
		float b1 = v3v1.setCross(e3.v, e2.v).dot(e0.v);
		float b2 = temp.setCross(e0.v, e1.v).dot(e3.v);
		float b3 = -v2v1.dot(e0.v);

		float sum = b0 + b1 + b2 + b3;
		if (sum <= 0) {
			b1 = -v3v1.dot(normedNormal);
			b2 = temp.setCross(e3.v, e1.v).dot(normedNormal);
			b3 = v2v1.dot(normedNormal);
			sum = b1 + b2 + b3;
			final float invHalfSum = 0.5f / sum;
			c.getPointA()
					.set(((e1.pA.x + e1.pB.x) * b1 + (e2.pA.x + e2.pB.x) * b2 + (e3.pA.x + e3.pB.x) * b3) * invHalfSum,
							((e1.pA.y + e1.pB.y) * b1 + (e2.pA.y + e2.pB.y) * b2 + (e3.pA.y + e3.pB.y) * b3)
									* invHalfSum,
					((e1.pA.z + e1.pB.z) * b1 + (e2.pA.z + e2.pB.z) * b2 + (e3.pA.z + e3.pB.z) * b3) * invHalfSum);
		} else {
			final float invHalfSum = 0.5f / sum;
			c.getPointA()
					.set(((e0.pA.x + e0.pB.x) * b0 + (e1.pA.x + e1.pB.x) * b1 + (e2.pA.x + e2.pB.x) * b2
							+ (e3.pA.x + e3.pB.x) * b3) * invHalfSum,
					((e0.pA.y + e0.pB.y) * b0 + (e1.pA.y + e1.pB.y) * b1 + (e2.pA.y + e2.pB.y) * b2
							+ (e3.pA.y + e3.pB.y) * b3) * invHalfSum,
					((e0.pA.z + e0.pB.z) * b0 + (e1.pA.z + e1.pB.z) * b1 + (e2.pA.z + e2.pB.z) * b2
							+ (e3.pA.z + e3.pB.z) * b3) * invHalfSum);
		}
		c.getPointB().set(c.getPointA());
	}

	protected static void expandPortal(Element e) {
		temp.setCross(e.v, e0.v);
		if (e1.v.dot(temp) > 0) {
			if (e2.v.dot(temp) > 0) {
				set(e1, e);
				v2v1.setSubtract(e2.v, e1.v);
				v3v1.setSubtract(e3.v, e1.v);
			} else {
				set(e3, e);
				v3v1.set(v4v1);
			}
		} else {
			if (e3.v.dot(temp) > 0) {
				set(e2, e);
				v2v1.set(v4v1);
			} else {
				set(e1, e);
				v2v1.setSubtract(e2.v, e1.v);
				v3v1.setSubtract(e3.v, e1.v);
			}
		}
	}

	protected static boolean portalReachTolerance(Vector3f dir) {
		final float min = v4v1.setSubtract(e4.v, e1.v).dot(dir);
		return min <= 0 || min * min <= CollideEpsilon_2 * dir.dot();
	}

	protected static void set(Element a, Element b) {
		a.v.set(b.v);
		a.pA.set(b.pA);
		a.pB.set(b.pB);
	}

}
