{ pkgs ? import <nixpkgs> {} }:
pkgs.mkShell {
  buildInputs = with pkgs;
    [
      stdenv
      pkg-config
      git
      lcov
      bear
      gtest
      valgrind
      ghostscript
      clang-tools_19
      graphviz
    ];
  shellHook =
  ''
    # When compiling on a non-machine, switch off these options.
    export ARM_COMPILE_FLAGS=""
  '';
}
