# BeagleG host package. Builds machine-control and its companion tools
# from the project Makefile. Supports native builds (for tests / dev) and
# cross-compilation to armv7 for BeagleBone deployment via pkgsCross.
{ stdenv, lib, pkg-config, gtest, pkgsBuildBuild }:
stdenv.mkDerivation {
  pname = "beagleg";
  version = "unstable";

  # Pass the project root directly (not lib.cleanSource) so the
  # am335x_pru_package submodule is included in the build sandbox.
  src = ./..;

  # pasm's linuxbuild needs a build-platform C compiler to produce the
  # PRU assembler binary that then runs in this sandbox.
  nativeBuildInputs = [ pkg-config pkgsBuildBuild.gcc ];
  CC_FOR_BUILD = "${pkgsBuildBuild.gcc}/bin/gcc";
  buildInputs = [ gtest ];

  # Patches for the vendored am335x_pru_package:
  #  - K&R-style ppCleanup() / definition pair rejected by strict GCC.
  #  - linuxbuild calls bare `gcc`, which under nix cross-builds is only
  #    available with the build-platform prefix; use $CC_FOR_BUILD.
  postPatch = ''
    substituteInPlace am335x_pru_package/pru_sw/utils/pasm_source/pasm.h \
      --replace-fail 'void ppCleanup();' 'void ppCleanup(int);'
    substituteInPlace am335x_pru_package/pru_sw/utils/pasm_source/pasmpp.c \
      --replace-fail 'void ppCleanup()' 'void ppCleanup(int Pass)'
    substituteInPlace am335x_pru_package/pru_sw/utils/pasm_source/linuxbuild \
      --replace-fail 'gcc -Wall' "''${CC_FOR_BUILD:-gcc} -Wall"
  '';

  # The Makefile's ARM_COMPILE_FLAGS default targets cortex-a8/armv7-a,
  # which is what we want for BBB cross builds. Clear it on any host that
  # isn't a 32-bit ARM (nix's stdenv has already set the right CC/CXX).
  ARM_COMPILE_FLAGS = lib.optionalString (!stdenv.hostPlatform.isAarch32) "";

  enableParallelBuilding = true;

  # Tests are gtest-based and run on the build machine; skip when cross-
  # compiling because the binaries can't execute on the build host.
  doCheck = stdenv.buildPlatform.canExecute stdenv.hostPlatform;
  checkTarget = "test";

  installPhase = ''
    runHook preInstall
    mkdir -p $out/bin
    install -m 755 machine-control gcode-print-stats $out/bin/
    install -m 755 src/gcode2ps $out/bin/
    runHook postInstall
  '';

  meta = with lib; {
    description = "Step-motor controller for CNC-like devices using the BeagleBone PRU";
    homepage = "https://github.com/hzeller/beagleg";
    license = licenses.gpl3Plus;
    platforms = platforms.linux;
    mainProgram = "machine-control";
  };
}
