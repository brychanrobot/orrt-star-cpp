#pragma once

class HaltonSampler {
   private:
	unsigned long currentIndex = 0;
	unsigned long base;

   public:
	HaltonSampler(unsigned long base) { this->base = base; }

	double next() {
		double result = 0.0;
		double f = 1.0;
		auto i = this->currentIndex;
		while (i > 0) {
			f /= base;
			result += f * (i % this->base);
			i = i / this->base;
		}

		this->currentIndex++;

		return result;
	}
};
