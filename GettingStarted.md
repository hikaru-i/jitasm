# Build #

jitasm is header-only library. You just copy and include _jitasm.h_ to use. Please build when you want to try jitasm sample or test.

jitasm supports following compilers:
  * Microsoft Visual C++ 2005 or later
  * GCC 4.1 or later


## Microsoft Visual C++ ##
To build samples, please open _samples/samples.sln_. The _pixel\_calc_ sample needs [Boost C++ Library](http://www.boost.org/).

To build test, please open _test/test.sln_. It needs Microsoft Visual C++ 2012 and [yasm](http://www.tortall.net/projects/yasm/).

## GCC ##
jitasm needs some compiler options:
  * **-fno-operator-names**
  * **-march=i686** for 32bit
  * **-mmmx**/**-msse**/**-msse2** when you use `__m64`/`__m128`/`__m128d`/`__m128i` as jitasm function argument or result.

To build samples, please _make_ in each subdirectories. The _pixel\_calc_ sample doe's not support gcc.


# Assembler #

```
#include "stdio.h"
#include "jitasm.h"

// int add1(int arg1)
// {
//   return arg1 + 1;
// }
struct add1 : jitasm::function<int, add1, int>
{
  Result main(Addr arg1)
  {
    // arg1 refers to address of first argument value on stack.
    mov(ecx, dword_ptr[arg1]);
    add(ecx, 1);
    return ecx;
  }
};

int main()
{
  // Make function instance
  add1 f;

  // Runtime code genaration and run
  int result = f(99);

  printf("Result : %d\n", result);

  return 0;
}
```

This is a small example how to use jitasm as assembler. add1 is JIT assembled function class and call it in main().
add1 class is derived from jitasm::function template class. First template argument is result type of function. Second one is derived class itself and third one is argument type of function.
If you want to pass more arguments, you can add template parameters until 10th argument.

```
push ebp
mov ebp,esp
mov eax,dword ptr [ebp+8]
add eax,1
mov eax,ecx
pop ebp
ret
```

This is a generated code as add1 function. jitasm generates function prolog and epilog automatically.
If you don't need prolog and epilog, you can write naked\_main.
```
struct add1 : jitasm::function<int, add1, int>
{
  void naked_main()
  {
    mov(eax, dword_ptr[esp + 4]);
    add(eax, 1);
    ret();
  }
};

mov eax,dword ptr [esp+4] 
add eax,1 
ret
```