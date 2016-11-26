class RGB {
   public:
	double R;
	double G;
	double B;

	RGB(double r, double g, double b) {
		R = r;
		G = g;
		B = b;
	}

	bool Equals(RGB rgb) { return (R == rgb.R) && (G == rgb.G) && (B == rgb.B); }
};

class HSL {
   public:
	int H;
	float S;
	float L;

	HSL(int h, float s, float l) {
		H = h;
		S = s;
		L = l;
	}

	bool Equals(HSL hsl) { return (H == hsl.H) && (S == hsl.S) && (L == hsl.L); }
};

static float HueToRGB(float v1, float v2, float vH) {
	if (vH < 0) vH += 1;

	if (vH > 1) vH -= 1;

	if ((6 * vH) < 1) return (v1 + (v2 - v1) * 6 * vH);

	if ((2 * vH) < 1) return v2;

	if ((3 * vH) < 2) return (v1 + (v2 - v1) * ((2.0f / 3) - vH) * 6);

	return v1;
}

static RGB HSLToRGB(HSL hsl) {
	double r = 0;
	double g = 0;
	double b = 0;

	if (hsl.S == 0) {
		r = g = b = (unsigned char)(hsl.L * 255);
	} else {
		float v1, v2;
		float hue = (float)hsl.H / 360;

		v2 = (hsl.L < 0.5) ? (hsl.L * (1 + hsl.S)) : ((hsl.L + hsl.S) - (hsl.L * hsl.S));
		v1 = 2 * hsl.L - v2;

		r = HueToRGB(v1, v2, hue + (1.0 / 3));
		g = HueToRGB(v1, v2, hue);
		b = HueToRGB(v1, v2, hue - (1.0 / 3));
	}

	return RGB(r, g, b);
}
