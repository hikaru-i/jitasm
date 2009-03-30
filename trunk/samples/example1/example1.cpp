#include "stdio.h"
#include "jitasm.h"

class example1 : public jitasm::function<int, int>
{
private:
	int val_;
public:
	example1(int val) : val_(val) {}

	Result main(Arg a1)
	{
		mov(ecx, dword_ptr[a1]);
		add(ecx, val_);
		return ecx;
	}
};

int main()
{
	example1 plus5(5);
	int result = plus5(10);
	printf("Result : %d\n", result);
	return 0;
}
