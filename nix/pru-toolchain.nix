# GNU PRU binutils (--target=pru-elf), used to package the remoteproc
# firmware ELF. Built from the upstream release because nixpkgs'
# lib.systems.parse does not know "pru" as a CPU family, which rules
# out the usual pkgsCross path. Debian ships the same tools as
# binutils-pru.
{ stdenv, fetchurl, texinfo, bison }:
{
  binutils-pru = stdenv.mkDerivation rec {
    pname = "binutils-pru";
    version = "2.43";

    src = fetchurl {
      url = "mirror://gnu/binutils/binutils-${version}.tar.xz";
      sha256 = "0rck6wqdc95ink0abkhlkvcj5wragx4kk77wsp8h33xc8gs0cdmm";
    };

    nativeBuildInputs = [ texinfo bison ];

    configureFlags = [
      "--target=pru-elf"
      # Release tarballs trip new warnings on newer host compilers;
      # don't let -Werror break the build.
      "--disable-werror"
      # No gettext message translations: drops that dependency,
      # English-only diagnostics are fine for a build tool.
      "--disable-nls"
      # Don't record timestamps/uid in ar archives, so outputs are
      # bit-reproducible as nix expects.
      "--enable-deterministic-archives"
    ];

    enableParallelBuilding = true;
  };
}
