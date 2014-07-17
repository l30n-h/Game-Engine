package com.brocorporation.gameengine.utils;

import com.brocorporation.gameengine.elements.collision.MPR;

public class Distance {

	protected final static Vector3f AB = new Vector3f();
	protected final static Vector3f AC = new Vector3f();

	public static float closestPointToLineSegment(Vector3f result, Vector3f A,
			Vector3f B) {
		AB.setSubtract(B, A);
		final float ABdotA = AB.dot(A);
		if (ABdotA >= 0) {
			result.set(A);
		} else {
			final float ABdotAB = AB.dot(AB);
			if (-ABdotA >= ABdotAB) {
				result.set(B);
			} else {
				result.setAddScaled(A, AB, -ABdotA / ABdotAB);
			}
		}
		return result.dot(result);
	}

	public static float distanceToPlane(Vector3f result, Vector3f A,
			Vector3f B, Vector3f C) {
		AB.setSubtract(B, A);
		AC.setSubtract(C, A);
		result.setCross(AB, AC);
		final float an = A.dot(result);
		if (an < 0) {
			result.invert();
		}
		return (an * an) / result.dot(result);
	}

	public static float closestPointToTriangle(Vector3f result, Vector3f A,
			Vector3f B, Vector3f C) {
		AB.setSubtract(B, A);
		AC.setSubtract(C, A);
		final float abab = AB.dot(AB);
		final float abac = AB.dot(AC);
		final float acac = AC.dot(AC);
		final float aab = A.dot(AB);
		final float aac = A.dot(AC);
		final float det = Math.abs(abab * acac - abac * abac);
		float s = abac * aac - acac * aab;
		float t = abac * aab - abab * aac;
		if (s + t <= det) {
			if (s < 0) {
				if (t < 0) {
					if (aab < 0) {
						if (-aab >= abab) {
							result.set(B);
						} else {
							s = -aab / abab;
							result.setAddScaled(A, AB, s);
						}
					} else {
						if (aac >= 0) {
							result.set(A);
						} else if (-aac >= acac) {
							result.set(C);
						} else {
							t = -aac / acac;
							result.setAddScaled(A, AC, t);
						}
					}
				} else {
					if (aac >= 0) {
						result.set(A);
					} else if (-aac >= acac) {
						result.set(C);
					} else {
						t = -aac / acac;
						result.setAddScaled(A, AC, t);
					}
				}
			} else if (t < 0) {
				if (aab >= 0) {
					result.set(A);
				} else if (-aab >= abab) {
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
						result.set(B);
					} else {
						s = numer / denom;
						t = 1 - s;
						result.setAddScaled(A, AB, s).addScaled(AC, t);
					}
				} else {
					if (tmp1 <= 0) {
						result.set(C);
					} else if (aac >= 0) {
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
						result.set(C);
					} else {
						t = numer / denom;
						s = 1 - t;
						result.setAddScaled(A, AB, s).addScaled(AC, t);
					}
				} else {
					if (tmp1 <= 0) {
						result.set(B);
					} else if (aab >= 0) {
						result.set(A);
					} else {
						s = -aab / abab;
						result.setAddScaled(A, AB, s);
					}
				}
			} else {
				final float numer = acac + aac - abac - aab;
				if (numer <= 0) {
					result.set(C);
				} else {
					final float denom = abab - 2 * abac + acac;
					if (numer >= denom) {
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

	static Vector3f d1 = new Vector3f();
	static Vector3f d2 = new Vector3f();
	static Vector3f a = new Vector3f();
	static Vector3f witness2 = new Vector3f();

	public static float ccdVec3PointTriDist2(Vector3f P, Vector3f x0,
			Vector3f B, Vector3f C, Vector3f witness) {
		// Computation comes from analytic expression for triangle (x0, B, C)
		// T(s, t) = x0 + s.d1 + t.d2, where d1 = B - x0 and d2 = C - x0 and
		// Then equation for distance is:
		// D(s, t) = | T(s, t) - P |^2
		// This leads to minimization of quadratic function of two variables.
		// The solution from is taken only if s is between 0 and 1, t is
		// between 0 and 1 and t + s < 1, otherwise distance from segment is
		// computed.
		float u, v, w, p, q, r;
		float s, t, dist, dist2;

		d1.setSubtract(B, x0);
		d2.setSubtract(C, x0);
		a.setSubtract(x0, P);
		u = a.dot(a);
		v = d1.dot(d1);
		w = d2.dot(d2);
		p = a.dot(d1);
		q = a.dot(d2);
		r = d1.dot(d2);

		s = (q * r - w * p) / (w * v - r * r);
		t = (-s * r - q) / w;

		if (s >= 0 && s <= 1 && t >= 0 && t <= 1 && (t + s) <= 1) {

			if (witness != null) {
				d1.scale(s);
				d2.scale(t);
				witness.setAdd(x0, d1).add(d2);
				AB.setSubtract(witness, P);
				dist = AB.dot(AB);
			} else {
				dist = s * s * v;
				dist += t * t * w;
				dist += 2 * s * t * r;
				dist += 2 * s * p;
				dist += 2 * t * q;
				dist += u;
			}
		} else {
			dist = __ccdVec3PointSegmentDist2(P, x0, B, witness);

			dist2 = __ccdVec3PointSegmentDist2(P, x0, C, witness2);
			if (dist2 < dist) {
				dist = dist2;
				if (witness != null)
					witness.set(witness2);
			}

			dist2 = __ccdVec3PointSegmentDist2(P, B, C, witness2);
			if (dist2 < dist) {
				dist = dist2;
				if (witness != null)
					witness.set(witness2);
			}
		}

		return dist;
	}

	public static float __ccdVec3PointSegmentDist2(Vector3f P, Vector3f x0,
			Vector3f b, Vector3f witness) {
		float dist, t;
		d1.setSubtract(b, x0);
		a.setSubtract(x0, P);

		t = -a.dot(d1) / d1.dot(d1);

		if (t <= 0) {
			AB.setSubtract(x0, P);
			dist = AB.dot(AB);
			if (witness != null)
				witness.set(x0);
		} else if (t >= 1) {
			AB.setSubtract(b, P);
			dist = AB.dot(AB);
			if (witness != null)
				witness.set(b);
		} else {
			if (witness != null) {
				witness.setScale(d1, t).add(x0);
				AB.setSubtract(witness, P);
				dist = AB.dot(AB);
			} else {
				// recycling variables
				d1.scale(t).add(a);
				dist = d1.dot(d1);
			}
		}

		return dist;
	}
	
	public static int removeFromTriangle(Vector3f A,
			Vector3f B, Vector3f C) {
		AB.setSubtract(B, A);
		AC.setSubtract(C, A);
		CB.setSubtract(B, C);
		dir.setCross(AB, AC);
		int front = 0;
		if (temp.setCross(AB, dir).dot(A) < 0) {
			front += 1;
		}
		if (temp.setCross(dir, AC).dot(A) < 0) {
			front += 2;
		}
		if (temp.setCross(dir, CB).dot(B) < 0) {
			front += 4;
		}
		if (front == 0) {
			return 0;
		} else if (front == 1) {
			return 3;
		} else if (front == 2) {
			return 2;
		} else if (front == 3) {
			if (A.dot(AB) < 0)
				return 3;
			return 2;
		} else if (front == 4) {
			return 1;
		} else if (front == 5) {
			if (B.dot(CB) > 0)
				return 1;
			return 3;
		} else {
			if (C.dot(AC) > 0)
				return 2;
			return 1;
		}
	}

	protected final static Vector3f CB = new Vector3f();
	protected final static Vector3f temp = new Vector3f();

	public static float distanceToTriangle(Vector3f result, Vector3f A,
			Vector3f B, Vector3f C) {
		AB.setSubtract(B, A);
		AC.setSubtract(C, A);
		CB.setSubtract(B, C);
		result.setCross(AB, AC);
		int front = 0;
		if (temp.setCross(AB, result).dot(A) < 0) {
			front += 1;
		}
		if (temp.setCross(result, AC).dot(A) < 0) {
			front += 2;
		}
		if (temp.setCross(result, CB).dot(B) < 0) {
			front += 4;
		}
		if (front == 0) {
			final float an = A.dot(result);
			if (an < 0) {
				result.invert();
			}
			return (an * an) / result.dot(result);
		} else if (front == 1) {
			return distanceToLineSegment(result, A, B);
		} else if (front == 2) {
			return distanceToLineSegment(result, A, C);
		} else if (front == 3) {
			if (A.dot(AB) < 0)
				return distanceToLineSegment(result, A, B);
			return distanceToLineSegment(result, A, C);
		} else if (front == 4) {
			return distanceToLineSegment(result, B, C);
		} else if (front == 5) {
			if (B.dot(CB) > 0)
				return distanceToLineSegment(result, B, C);
			return distanceToLineSegment(result, A, B);
		} else {
			if (C.dot(AC) > 0)
				return distanceToLineSegment(result, A, C);
			return distanceToLineSegment(result, B, C);
		}
	}

	public static float distanceToLineSegment(Vector3f result, Vector3f A,
			Vector3f B) {
		AB.setSubtract(B, A);
		final float ABdotA = AB.dot(A);
		if (ABdotA >= 0) {
			result.set(A);
		} else {
			final float ABdotAB = -AB.dot(AB);
			if (ABdotA <= ABdotAB) {
				result.set(B);
			} else {
				result.setAddScaled(A, AB, ABdotA / ABdotAB);
			}
		}
		return result.dot(result);
	}

	static Vector3f dir = new Vector3f();

	public static boolean intersectsLineTriangle(Vector3f from,
			Vector3f to, Vector3f normal, Vector3f A, Vector3f B, Vector3f C,boolean segment) {
		dir.setSubtract(to, from);
		float r = normal.dot(dir);
		if (r <= 0) {
			return false;
		}
		CB.setSubtract(A, from);
		r = normal.dot(CB) / r;
		if ((segment && r > 1) || r < 0) {
			return false;
		}
		CB.setAddScaled(from, dir, r).subtract(A);
		AB.setSubtract(B, A);
		AC.setSubtract(C, A);
		float abab = AB.dot(AB);
		float acac = AC.dot(AC);
		float abac = AB.dot(AC);
		float pab = CB.dot(AB);
		float pac = CB.dot(AC);
		float denom = abac * abac - abab * acac;
		float s = (abac * pac - acac * pab) / denom;
		if (s < 0 || s > 1)
			return false;
		float t = (abac * pab - abab * pac) / denom;
		return t >= 0 && s + t <= 1;
	}
	
	public static boolean intersectsLineTriangle(Vector3f from,
			Vector3f to, Vector3f normal, Vector3f A, Vector3f B, Vector3f C) {
		dir.setSubtract(to, from);
		float r = normal.dot(dir);
		if (r == 0) {
			return false;
		}
		CB.setSubtract(A, from);
		r = normal.dot(CB) / r;
		CB.setAddScaled(from, dir, r).subtract(A);
		AB.setSubtract(B, A);
		AC.setSubtract(C, A);
		float abab = AB.dot(AB);
		float acac = AC.dot(AC);
		float abac = AB.dot(AC);
		float pab = CB.dot(AB);
		float pac = CB.dot(AC);
		float denom = abac * abac - abab * acac;
		float s = (abac * pac - acac * pab) / denom;
		MPR.bu.append("s "+s+"\n");
		if (s < 0 || s > 1)
			return false;
		float t = (abac * pab - abab * pac) / denom;
		MPR.bu.append("t "+t+"\n");
		return t >= 0 && s + t <= 1;
	}
}
