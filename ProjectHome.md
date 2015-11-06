# jitasm #
`[`_En_`]`<br>
jitasm is C++ library for runtime code generation of x86/x64. You can write the code like a inline assembler.<br>
<br>
<code>[</code><i>Ja</i><code>]</code><br>
jitasmはx86/x64のコードを動的に生成するためのC++ライブラリです。<br>
インラインアセンブラのような直感的な記述で動的なコード生成・実行を行うことができます。<br>

<h1>Features</h1>
<code>[</code><i>En</i><code>]</code>
<ul><li>Header only library<br>
</li><li>Support for x86, x64, mmx, sse, sse2, sse3, ssse3, sse4.1, sse4.2, avx, fma, xop, fma4<br>
</li><li>Automatic code generation of prolog and epilog according to function calling convention<br>
</li><li>Register allocation<br>
</li><li>Support for Windows, Linux, FreeBSD, Mac<br>
<code>[</code><i>Ja</i><code>]</code>
</li><li>単一のヘッダファイルのみのライブラリ<br>
</li><li>x86, x64, mmx, sse, sse2, sse3, ssse3, sse4.1, sse4.2, avx, fma, xop, fma4サポート<br>
</li><li>関数呼び出し規約に従ったプロローグ・エピローグの自動生成<br>
</li><li>レジスタアロケーション<br>
</li><li>Windows, Linux, FreeBSD, Macサポート</li></ul>

<h1>Example</h1>

<pre><code>// int plus(int a, int b)<br>
// {<br>
//   return a + b;<br>
// }<br>
struct Plus : public jitasm::function&lt;int, Plus, int, int&gt;<br>
{<br>
  Result main(Reg32 a, Reg32 b)<br>
  {<br>
    add(a, b);<br>
    return a;<br>
  }<br>
};<br>
<br>
// Generate plus function and call.<br>
Plus plus;<br>
int c = plus(1, 2);<br>
</code></pre>