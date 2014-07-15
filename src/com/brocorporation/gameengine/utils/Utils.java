package com.brocorporation.gameengine.utils;

public class Utils {

	public static float[] lerp(final float[] start, final float[] end,
			final float alpha) {
		if (end == null) {
			return start;
		}
		for (int i = 0; i < start.length; i++) {
			end[i] += alpha * (start[i] - end[i]);
		}
		return end;
	}

	public static float lerp(final float start, final float end,
			final float alpha) {
		return end + alpha * (start - end);
	}

	public static float parseFloat(final CharSequence f, final int start,
			final int length) {
		final int len = length;
		float ret = 0f;
		int pos = start;
		int part = 0;
		boolean neg = false;
		char charAtPos;
		while (pos < len
				&& ((charAtPos = f.charAt(pos)) < '0' || charAtPos > '9')
				&& charAtPos != '-' && charAtPos != '.') {
			pos++;
		}

		if ((charAtPos = f.charAt(pos)) == '-') {
			neg = true;
			pos++;
		}

		while (pos < len
				&& !((charAtPos = f.charAt(pos)) > '9' || charAtPos < '0')) {
			part = part * 10 + (charAtPos - '0');
			pos++;
		}
		ret = neg ? -part : part;

		if (pos < len && f.charAt(pos) == '.') {
			pos++;
			int mul = 1;
			part = 0;
			while (pos < len
					&& !((charAtPos = f.charAt(pos)) > '9' || charAtPos < '0')) {
				part = part * 10 + (charAtPos - '0');
				mul *= 10;
				pos++;
			}
			ret = neg ? ret - (float) part / (float) mul : ret + (float) part
					/ (float) mul;
		}

		if (pos < len
				&& ((charAtPos = f.charAt(pos)) == 'e' || charAtPos == 'E')) {
			part = 0;
			while (++pos < len
					&& !((charAtPos = f.charAt(pos)) > '9' || charAtPos < '0')) {
				part = part * 10 + (charAtPos - '0');
			}
			ret *= (float) Math.pow(10, part);
		}
		return ret;
	}

	public static float parseFloat(final char[] f, final int start,
			final int length) {
		final int len = length;
		float ret = 0f;
		int pos = start;
		int part = 0;
		boolean neg = false;
		char charAtPos;
		while (pos < len && ((charAtPos = f[pos]) < '0' || charAtPos > '9')
				&& charAtPos != '-' && charAtPos != '.') {
			pos++;
		}

		if ((charAtPos = f[pos]) == '-') {
			neg = true;
			pos++;
		}

		while (pos < len && !((charAtPos = f[pos]) > '9' || charAtPos < '0')) {
			part = part * 10 + (charAtPos - '0');
			pos++;
		}
		ret = neg ? -part : part;

		if (pos < len && f[pos] == '.') {
			pos++;
			int mul = 1;
			part = 0;
			while (pos < len
					&& !((charAtPos = f[pos]) > '9' || charAtPos < '0')) {
				part = part * 10 + (charAtPos - '0');
				mul *= 10;
				pos++;
			}
			ret = neg ? ret - (float) part / (float) mul : ret + (float) part
					/ (float) mul;
		}

		if (pos < len && ((charAtPos = f[pos]) == 'e' || charAtPos == 'E')) {
			part = 0;
			while (++pos < len
					&& !((charAtPos = f[pos]) > '9' || charAtPos < '0')) {
				part = part * 10 + (charAtPos - '0');
			}
			ret *= (float) Math.pow(10, part);
		}
		return ret;
	}
}
