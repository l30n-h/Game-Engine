package com.brocorporation.gameengine.elements.collision;

import com.brocorporation.gameengine.utils.Vector3f;

public class GJK {
	private final static int MAX_ITERATIONS = 20;
	private final static float EPSILON = 1e-4f;
	private final static float EPSILON_2 = EPSILON * EPSILON;

	private final static Simplex simplex = new Simplex();
	private final static Vector3f AB = new Vector3f();
	private final static Vector3f AC = new Vector3f();

	private final static Vector3f v = new Vector3f();
	private final static Vector3f v0 = new Vector3f();
	private final static Vector3f p = new Vector3f();
	private final static Vector3f w = new Vector3f();
	private final static Vector3f s1 = new Vector3f();
	private final static Vector3f s2 = new Vector3f();
	private static float s;
	private static float t;

	public static float distance(Contact contact, IShape shape1, IShape shape2) {
		v.setSubtract(shape1.getPosition(), shape2.getPosition());
		maxPointInMinkDiffAlongDir(v, shape1, shape2, v);
		simplex.add(v, s1, s2);
		float d_2 = v.dot(v);
		int i = 0;
		if (d_2 > EPSILON_2) {
			float d0_2;
			do {
				minPointInMinkDiffAlongDir(w, shape1, shape2, v);
				v0.set(v);
				d0_2 = d_2;
				if (simplex.contains(w)) {
					break;
				}
				simplex.add(w, s1, s2);
				d_2 = closestPointToOrigin(v, simplex);
				if (d_2 >= d0_2) {
					v.set(v0);
					d_2 = d0_2;
					break;
				}
			} while (d_2 > EPSILON_2 && i++ < MAX_ITERATIONS
					&& (d0_2 - 2 * v0.dot(v) + d_2) > EPSILON_2);
		}
		if (d_2 <= EPSILON_2) {
			simplex.clear();
			contact.setDistance(0);
			contact.getNormal().set(0, 0, 0);
			return 0;
		}
		simplex.getClosestPoints(contact, v);
		simplex.clear();
		d_2 = (float) Math.sqrt(d_2);
		contact.setDistance(d_2);
		contact.getNormal().setScale(v, -1 / d_2);
		return d_2;
	}

	public static boolean intersects(Contact contact, IShape shape1,
			IShape shape2, float r) {
		v.setSubtract(shape1.getPosition(), shape2.getPosition());
		maxPointInMinkDiffAlongDir(v, shape1, shape2, v);
		simplex.add(v, s1, s2);
		float d_2 = v.dot(v);
		int i = 0;
		boolean checkIntersection = true;
		final float r2 = 2 * r;
		if (d_2 > EPSILON_2) {
			final float r2_2 = r2 * r2;
			float d0_2;
			do {
				minPointInMinkDiffAlongDir(w, shape1, shape2, v);
				if (checkIntersection) {
					final float vw = v.dot(w);
					if (vw > 0 && vw * vw > r2_2 * d_2) {
						checkIntersection = false;
					}
				}
				v0.set(v);
				d0_2 = d_2;
				if (simplex.contains(w)) {
					break;
				}
				simplex.add(w, s1, s2);
				d_2 = closestPointToOrigin(v, simplex);
				if (d_2 >= d0_2) {
					v.set(v0);
					d_2 = d0_2;
					break;
				}
			} while (d_2 > EPSILON_2 && i++ < MAX_ITERATIONS
					&& (d0_2 - 2 * v0.dot(v) + d_2) > EPSILON_2);
		}
		if (d_2 <= EPSILON_2) {
			simplex.clear();
			contact.setDistance(0);
			contact.getNormal().set(0, 0, 0);
			// EPA
			return true;
		}
		simplex.getClosestPoints(contact, v);
		simplex.clear();
		d_2 = (float) Math.sqrt(d_2);
		if (checkIntersection) {
			contact.setDistance(-(r2 - d_2));
			contact.getNormal().setScale(v, -1 / d_2);
			return true;
		} else {
			contact.setDistance(d_2);
			contact.getNormal().setScale(v, -1 / d_2);
			return false;
		}
	}

	public static boolean intersects(final IShape shape1, final Vector3f vertex) {
		v.setSubtract(shape1.getPosition(), vertex);
		maxPointInMinkDiffAlongDir(v, shape1, vertex, v);
		simplex.add(v);
		float d_2 = v.dot(v);
		int i = 0;
		while (d_2 > EPSILON_2 && i++ < MAX_ITERATIONS) {
			minPointInMinkDiffAlongDir(w, shape1, vertex, v);
			if (v.dot(w) > 0) {
				simplex.clear();
				return false;
			}
			if (simplex.contains(w))
				break;
			simplex.add(w);
			d_2 = closestPointToOrigin(v, simplex);
		}
		simplex.clear();
		return true;
	}

	public static boolean intersects(IShape shape1, IShape shape2) {
		v.setSubtract(shape1.getPosition(), shape2.getPosition());
		maxPointInMinkDiffAlongDir(v, shape1, shape2, v);
		simplex.add(v);
		float d_2 = v.dot(v);
		int i = 0;
		while (d_2 > EPSILON_2 && i++ < MAX_ITERATIONS) {
			minPointInMinkDiffAlongDir(w, shape1, shape2, v);
			if (v.dot(w) > 0) {
				simplex.clear();
				return false;
			}
			if (simplex.contains(w))
				break;
			simplex.add(w);
			d_2 = closestPointToOrigin(v, simplex);
		}
		simplex.clear();
		return true;
	}

	public static boolean rayCast(RaycastHit hit, Vector3f normal,
			Vector3f from, Vector3f direction, IShape stcShape) {
		float s = 0;
		final Vector3f x = hit.getPoint().set(from);
		simplex.setRef(x);
		v.setSubtract(x, stcShape.getPosition());
		int i = 0;
		float d_2 = v.dot(v);
		while (d_2 > EPSILON_2) {
			if (i > MAX_ITERATIONS) {
				simplex.clear();
				return false;
			}
			i++;
			stcShape.getMaxAlongDirection(p, v);
			w.setSubtract(x, p);
			final float vdotw = v.dot(w);
			if (vdotw > 0) {
				final float vdotdirection = v.dot(direction);
				if (vdotdirection >= 0) {
					simplex.clear();
					return false;
				} else {
					s -= vdotw / vdotdirection;
					x.setAddScaled(from, direction, s);
					normal.set(v);
				}
			}
			if (!simplex.contains(p))
				simplex.add(p);
			d_2 = closestPointToOrigin(v, simplex);
		}
		simplex.clear();
		hit.setScalar(s);
		return true;
	}

	public static boolean rayCast(RaycastHit hit, Vector3f normal,
			Vector3f from, Vector3f direction, IShape dynShape, IShape stcShape) {
		float s = 0;
		final Vector3f x = hit.getPoint().set(from);
		simplex.setRef(x);
		v.setSubtract(x, v.setSubtract(from, stcShape.getPosition()));
		int i = 0;
		float d_2 = v.dot(v);
		while (d_2 > EPSILON_2) {
			if (i >= MAX_ITERATIONS) {
				simplex.clear();
				return false;
			}
			i++;
			maxPointInMinkDiffAlongDir(p, stcShape, dynShape, v);
			w.setSubtract(x, p);
			final float vdotw = v.dot(w);
			if (vdotw > 0) {
				final float vdotdirection = v.dot(direction);
				if (vdotdirection >= 0) {
					simplex.clear();
					return false;
				} else {
					s -= vdotw / vdotdirection;
					x.setAddScaled(from, direction, s);
					normal.set(v);
				}
			}
			if (!simplex.contains(p))
				simplex.add(p);
			d_2 = closestPointToOrigin(v, simplex);
		}
		simplex.clear();
		hit.setScalar(s);
		return true;
	}

	private static float closestPointToLineSegment(Vector3f result,
			Simplex simplex) {
		final Vector3f A = simplex.get(1);
		final Vector3f B = simplex.get(0);
		AB.setSubtract(B, A);
		final float ABdotA = AB.dot(A);
		if (ABdotA >= 0) {
			simplex.removeB();
			result.set(A);
		} else {
			final float ABdotAB = -AB.dot(AB);
			if (ABdotA <= ABdotAB) {
				simplex.removeA();
				result.set(B);
			} else {
				s = ABdotA / ABdotAB;
				result.setAddScaled(A, AB, s);
			}
		}
		return result.dot(result);
	}

	private static float closestPointToTriangle(Vector3f result, Simplex simplex) {
		final Vector3f A = simplex.get(2);
		final Vector3f B = simplex.get(1);
		final Vector3f C = simplex.get(0);
		AB.setSubtract(B, A);
		AC.setSubtract(C, A);
		final float abab = AB.dot(AB);
		final float abac = AB.dot(AC);
		final float acac = AC.dot(AC);
		final float aab = A.dot(AB);
		final float aac = A.dot(AC);
		final float det = Math.abs(abab * acac - abac * abac);
		s = abac * aac - acac * aab;
		t = abac * aab - abab * aac;
		if (s + t <= det) {
			if (s < 0) {
				if (t < 0) {
					if (aab < 0) {
						simplex.removeC();
						if (-aab >= abab) {
							simplex.removeA();
							result.set(B);
						} else {
							s = -aab / abab;
							result.setAddScaled(A, AB, s);
						}
					} else {
						simplex.removeB();
						if (aac >= 0) {
							simplex.removeC();
							result.set(A);
						} else if (-aac >= acac) {
							simplex.removeA();
							result.set(C);
						} else {
							t = -aac / acac;
							result.setAddScaled(A, AC, t);
						}
					}
				} else {
					simplex.removeB();
					if (aac >= 0) {
						simplex.removeC();
						result.set(A);
					} else if (-aac >= acac) {
						simplex.removeA();
						result.set(C);
					} else {
						t = -aac / acac;
						result.setAddScaled(A, AC, t);
					}
				}
			} else if (t < 0) {
				simplex.removeC();
				if (aab >= 0) {
					simplex.removeB();
					result.set(A);
				} else if (-aab >= abab) {
					simplex.removeA();
					result.set(B);
				} else {
					s = -aab / abab;
					result.setAddScaled(A, AB, s);

				}
			} else {
				float invDet = 1 / det;
				s *= invDet;
				t *= invDet;
				result.setAddScaled(A, AB, s).addScaled(AC, t);
			}
		} else {
			if (s < 0) {
				final float tmp0 = abac + aab;
				final float tmp1 = acac + aac;
				if (tmp1 > tmp0) {
					final float numer = tmp1 - tmp0;
					final float denom = abab - 2 * abac + acac;
					if (numer >= denom) {
						simplex.removeC();
						simplex.removeA();
						result.set(B);
					} else {
						s = numer / denom;
						t = 1 - s;
						result.setAddScaled(A, AB, s).addScaled(AC, t);
					}
				} else {
					simplex.removeB();
					if (tmp1 <= 0) {
						simplex.removeA();
						result.set(C);
					} else if (aac >= 0) {
						simplex.removeC();
						result.set(A);
					} else {
						t = -aac / acac;
						result.setAddScaled(A, AC, t);
					}
				}
			} else if (t < 0) {
				final float tmp0 = abac + aac;
				final float tmp1 = abab + aab;
				if (tmp1 > tmp0) {
					final float numer = tmp1 - tmp0;
					final float denom = abab - 2 * abac + acac;
					if (numer >= denom) {
						simplex.removeB();
						simplex.removeA();
						result.set(C);
					} else {
						t = numer / denom;
						s = 1 - t;
						result.setAddScaled(A, AB, s).addScaled(AC, t);
					}
				} else {
					simplex.removeC();
					if (tmp1 <= 0) {
						simplex.removeA();
						result.set(B);
					} else if (aab >= 0) {
						simplex.removeB();
						result.set(A);
					} else {
						s = -aab / abab;
						result.setAddScaled(A, AB, s);
					}
				}
			} else {
				final float numer = acac + aac - abac - aab;
				if (numer <= 0) {
					simplex.removeB();
					simplex.removeA();
					result.set(C);
				} else {
					final float denom = abab - 2 * abac + acac;
					if (numer >= denom) {
						simplex.removeC();
						simplex.removeA();
						result.set(B);
					} else {
						s = numer / denom;
						t = 1 - s;
						result.setAddScaled(A, AB, s).addScaled(AC, t);
					}
				}
			}
		}
		return result.dot(result);
	}

	private static float closestPointToOrigin(Vector3f result, Simplex simplex) {
		final int size = simplex.size();
		if (size == 1) {
			result.set(simplex.get(0));
			return result.dot(result);
		} else if (size == 2) {
			return closestPointToLineSegment(result, simplex);
		} else if (size == 3) {
			return closestPointToTriangle(result, simplex);
		} else {
			final Vector3f A = simplex.get(3);
			final Vector3f B = simplex.get(2);
			final Vector3f C = simplex.get(1);
			final Vector3f D = simplex.get(0);
			final float aordx = B.y * C.z - B.z * C.y;
			final float aorbx = C.y * D.z - C.z * D.y;
			final float aorcx = B.y * D.z - B.z * D.y;
			final float borcx = A.z * D.y - A.y * D.z;
			final float bordx = A.z * C.y - A.y * C.z;
			final float cordx = A.y * B.z - A.z * B.y;
			final float dA = -B.x * aorbx + C.x * aorcx - D.x * aordx;
			final float dB = A.x * aorbx + C.x * borcx - D.x * bordx;
			final float dC = -A.x * aorcx - B.x * borcx - D.x * cordx;
			final float dD = A.x * aordx + B.x * bordx + C.x * cordx;
			final float dO = dA + dB + dC + dD;
			final float dOsign = Math.signum(dO);
			final boolean samedOdB = dOsign == Math.signum(dB);
			final boolean samedOdC = dOsign == Math.signum(dC);
			final boolean samedOdD = dOsign == Math.signum(dD);
			if (samedOdB && samedOdC && samedOdD)
				return 0;
			final boolean notsamedOdBdC = !samedOdB && !samedOdC;
			if (notsamedOdBdC && !samedOdD) {
				final float absdB = Math.abs(dB);
				final float absdC = Math.abs(dC);
				if (absdB > absdC) {
					if (absdB > Math.abs(dD)) {
						simplex.removeB();
					} else {
						simplex.removeD();
					}
				} else if (absdC > Math.abs(dD)) {
					simplex.removeC();
				} else {
					simplex.removeD();
				}
			} else if (notsamedOdBdC) {
				if (Math.abs(dB) > Math.abs(dC)) {
					simplex.removeB();
				} else {
					simplex.removeC();
				}
			} else if (!samedOdB && !samedOdD) {
				if (Math.abs(dB) > Math.abs(dD)) {
					simplex.removeB();
				} else {
					simplex.removeD();
				}
			} else if (!samedOdC && !samedOdD) {
				if (Math.abs(dC) > Math.abs(dD)) {
					simplex.removeC();
				} else {
					simplex.removeD();
				}
			} else if (!samedOdD) {
				simplex.removeD();
			} else if (!samedOdC) {
				simplex.removeC();
			} else {
				simplex.removeB();
			}
			return closestPointToOrigin(result, simplex);
		}
	}

	private static void maxPointInMinkDiffAlongDir(final Vector3f result,
			final IShape shape1, final Vector3f vertex, final Vector3f direction) {
		shape1.getMaxAlongDirection(s1, direction);
		s2.set(vertex);
		result.setSubtract(s1, s2);
	}

	private static void minPointInMinkDiffAlongDir(final Vector3f result,
			final IShape shape1, final Vector3f vertex, final Vector3f direction) {
		shape1.getMinAlongDirection(s1, direction);
		s2.set(vertex);
		result.setSubtract(s1, s2);
	}

	private static void maxPointInMinkDiffAlongDir(final Vector3f result,
			final IShape shape1, final IShape shape2, final Vector3f direction) {
		shape1.getMaxAlongDirection(s1, direction);
		shape2.getMinAlongDirection(s2, direction);
		result.setSubtract(s1, s2);
	}

	private static void minPointInMinkDiffAlongDir(final Vector3f result,
			final IShape shape1, final IShape shape2, final Vector3f direction) {
		shape1.getMinAlongDirection(s1, direction);
		shape2.getMaxAlongDirection(s2, direction);
		result.setSubtract(s1, s2);
	}

	private static class Simplex {

		private Element[] elements = { new Element(), new Element(),
				new Element(), new Element() };
		private float lastS, lastT;
		private byte size = 0;
		private Vector3f ref;

		public Simplex() {
		}

		public void setRef(Vector3f v) {
			ref = v;
		}

		public byte size() {
			return size;
		}

		public void add(Vector3f v) {
			if (size < 4) {
				elements[size].vector.set(v);
				size++;
			}
		}

		public void add(Vector3f v, Vector3f s1, Vector3f s2) {
			if (size < 4) {
				lastS = s;
				if (size == 3) {
					lastT = t;
				}
				final Element e = elements[size];
				e.vector.set(v);
				e.pointS1.set(s1);
				e.pointS2.set(s2);
				size++;
			}
		}

		public boolean contains(Vector3f v) {
			for (int i = 0; i < size; i++) {
				if (elements[i].vector.equals(v)) {
					return true;
				}
			}
			return false;
		}

		public Vector3f get(int i) {
			if (0 <= i && i < size) {
				if (ref == null) {
					return elements[i].vector;
				} else {
					final Element e = elements[i];
					return e.ref.setSubtract(ref, e.vector);
				}
			}
			return null;
		}

		public void removeA() {
			remove(size - 1);
		}

		public void removeB() {
			remove(size - 2);
		}

		public void removeC() {
			remove(size - 3);
		}

		public void removeD() {
			remove(0);
		}

		public void remove(int i) {
			if (i < size && 0 <= i) {
				if (--size > i) {
					final Element ei = elements[i];
					while (i < size) {
						elements[i] = elements[++i];
					}
					elements[i] = ei;
				}
			}
		}

		/*
		 * public Vector3f getNormal(){ return normal; }
		 * 
		 * public Vector3f getDistance(){ return distance; }
		 * 
		 * public float getSquaredDistance(){ return sqrtDist; }
		 * 
		 * public float getDistance(){ return distance; }
		 */

		public void getClosestPoints(Contact c, Vector3f normal) {
			if (size == 1) {
				final Element e0 = elements[0];
				c.getPointA().set(e0.pointS1);
				c.getPointB().set(e0.pointS2);
			} else if (size == 2) {
				final Element eA = elements[0];
				final Element eB = elements[1];
				float r2 = 1 - lastS;
				c.getPointA().setScale(eA.pointS1, lastS)
						.addScaled(eB.pointS1, r2);
				if (normal == null) {
					c.getPointB().set(
							eA.pointS2.scale(lastS).addScaled(eB.pointS2, r2));
				} else
					c.getPointB().setSubtract(c.getPointA(), normal);
			} else if (size == 3) {
				final Element eA = elements[0];
				final Element eB = elements[1];
				final Element eC = elements[2];
				float r3 = 1 - lastT - lastS;
				c.getPointA().setScale(eA.pointS1, lastT)
						.addScaled(eB.pointS1, lastS).addScaled(eC.pointS1, r3);
				if (normal == null) {
					c.getPointB().set(
							eA.pointS2.scale(lastT)
									.addScaled(eB.pointS2, lastS)
									.addScaled(eC.pointS2, r3));
				} else
					c.getPointB().setSubtract(c.getPointA(), normal);
			}
		}

		public void clear() {
			size = 0;
			ref = null;
		}

		private static class Element {
			Vector3f vector = new Vector3f();
			Vector3f ref = new Vector3f();
			Vector3f pointS1 = new Vector3f();
			Vector3f pointS2 = new Vector3f();
		}
	}
}
