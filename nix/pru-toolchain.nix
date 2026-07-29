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
      "--disable-werror"
      "--disable-nls"
      "--disable-multilib"
      "--enable-deterministic-archives"
    ];

    enableParallelBuilding = true;
  };
}
