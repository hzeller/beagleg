# This is a nix-shell for use with the nix package manager.
# If you have nix installed, you may simply run `nix-shell`
# in this repo, and have all dependencies ready in the new shell.

{ pkgs ? import <nixpkgs> {} }:
pkgs.mkShell {
  buildInputs = with pkgs;
    [
      pkg-config
      git
      lcov
      bear
      gtest
      valgrind

      yosys
      surelog
      yosys-synlig

      nextpnr
      icestorm
      symbiyosys
      verilator

      clang-tools_11
    ];
  shellHook =
  ''
    # When compiling on a non-machine, switch off these options.
    export ARM_COMPILE_FLAGS=""
  '';
}
