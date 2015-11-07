# jitasm
jitasm is C++ library for runtime code generation of x86/x64. You can write the code like a inline assembler.

# Features
- Header only library.
- Support for x86, x64, mmx, sse, sse2, sse3, ssse3, sse4.1, sse4.2, avx, fma, xop, fma4.
- Automatic code generation of prolog and epilog according to function calling convention.
- Register allocation.
- Support for Windows, Linux, FreeBSD, Mac

# Example
```C++
// int plus(int a, int b)
// {
//   return a + b;
// }
struct Plus : public jitasm::function<int, Plus, int, int>
{
  Result main(Reg32 a, Reg32 b)
  {
    add(a, b);
    return a;
  }
};

// Generate plus function and call.
Plus plus;
int c = plus(1, 2);
```
