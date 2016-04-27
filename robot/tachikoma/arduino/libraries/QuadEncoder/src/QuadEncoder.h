#ifndef QuadEncoder_h
#define QuadEncoder_h

class QuadEncoder {
	public:
		QuadEncoder(void);
		void attach(int pin1, int pin2);
		long long read(void); // updates the quad encoder
		void reset(void);
		void reverse(bool rev);

		long long pos;
		bool reversed; // set
		char pin[2];

	private:
		void update();
		char pin_state[2];
		long long velocity;	// estimated
};

#endif
