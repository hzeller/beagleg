{ pkgs ? import <nixpkgs> {} }:
let
  # Want a gtest version that is compatible with c++11
  gtest_1_12_1 = pkgs.gtest.overrideAttrs (oldAttrs: rec {
    version = "1.12.1";

    src = pkgs.fetchFromGitHub {
      owner = "google";
      repo = "googletest";
      rev = "release-${version}";
      hash = "sha256-W+OxRTVtemt2esw4P7IyGWXOonUN5ZuscjvzqkYvZbM=";
    };
    patches = [];
  });
  pru = pkgs.callPackage ./nix/pru-toolchain.nix {};
in
pkgs.mkShell {
  buildInputs = with pkgs;
    [
      stdenv
      pkg-config
      git
      lcov
      bear
      gtest_1_12_1
      valgrind
      ghostscript
      llvmPackages_19.clang-tools
      graphviz
      pru.binutils-pru
    ];
  shellHook =
  ''
    # When compiling on a non-machine, switch off these options.
    export ARM_COMPILE_FLAGS=""
  '';
}
